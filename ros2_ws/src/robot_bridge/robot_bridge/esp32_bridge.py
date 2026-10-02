"""Bridge between ROS 2 and the ESP32 low-level controller over USB serial.

The robot is tracked (skid-steer): each side's drive sprocket is treated as a
wheel of radius `sprocket_radius`. Tracks slip sideways when turning, so the
effective track width used for kinematics is `track_separation * turn_slip_factor`,
calibrated by spinning the robot in place (see README).

See firmware/esp32/src/main.cpp for the serial protocol.
"""

import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.port = self.declare_parameter('port', '/dev/esp32').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.sprocket_radius = self.declare_parameter('sprocket_radius', 0.02).value
        track_separation = self.declare_parameter('track_separation', 0.15).value
        turn_slip_factor = self.declare_parameter('turn_slip_factor', 1.0).value
        self.effective_width = track_separation * turn_slip_factor
        self.counts_per_rev = self.declare_parameter('counts_per_rev', 682.0).value
        self.max_linear = self.declare_parameter('max_linear_speed', 0.5).value
        self.max_angular = self.declare_parameter('max_angular_speed', 2.0).value
        self.cmd_rate = self.declare_parameter('cmd_rate', 20.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.publish_tf = self.declare_parameter('publish_tf', False).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.left_joint = self.declare_parameter('left_joint', 'left_sprocket_joint').value
        self.right_joint = self.declare_parameter('right_joint', 'right_sprocket_joint').value

        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / self.cmd_rate, self.send_command)

        self.x = self.y = self.theta = 0.0
        self.prev_ticks = None
        self.prev_time = None
        self.target = (0.0, 0.0)
        self.last_cmd_time = self.get_clock().now()

        self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
        self.serial_lock = threading.Lock()
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'Connected to ESP32 on {self.port} @ {self.baud}')

    # -- commands ------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist):
        v = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        w = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        half = self.effective_width / 2.0
        self.target = ((v - w * half) / self.sprocket_radius,
                       (v + w * half) / self.sprocket_radius)
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        left, right = self.target if age < self.cmd_timeout else (0.0, 0.0)
        self.write_line(f'V {left:.4f} {right:.4f}')

    def write_line(self, line: str):
        with self.serial_lock:
            try:
                self.serial.write((line + '\n').encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}', throttle_duration_sec=2.0)

    # -- telemetry -----------------------------------------------------------

    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                raw = self.serial.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}', throttle_duration_sec=2.0)
                time.sleep(0.5)
                continue
            line = raw.decode(errors='ignore').strip()
            if line.startswith('E '):
                self.handle_telemetry(line.split()[1:])
            elif line.startswith('I '):
                self.get_logger().info(f'ESP32: {line[2:]}')

    def handle_telemetry(self, fields):
        try:
            left_ticks, right_ticks = int(fields[0]), int(fields[1])
            accel = [float(f) for f in fields[2:5]]
            gyro = [float(f) for f in fields[5:8]]
        except (ValueError, IndexError):
            return
        if len(gyro) != 3:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        rad_per_count = 2.0 * math.pi / self.counts_per_rev
        left_pos = left_ticks * rad_per_count
        right_pos = right_ticks * rad_per_count

        if self.prev_ticks is None:
            self.prev_ticks = (left_ticks, right_ticks)
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        d_left = (left_ticks - self.prev_ticks[0]) * rad_per_count * self.sprocket_radius
        d_right = (right_ticks - self.prev_ticks[1]) * rad_per_count * self.sprocket_radius
        self.prev_ticks = (left_ticks, right_ticks)
        self.prev_time = now

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.effective_width
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        self.publish_odom(stamp, d_center / dt, d_theta / dt)
        self.publish_joints(stamp, left_pos, right_pos,
                            d_left / self.sprocket_radius / dt,
                            d_right / self.sprocket_radius / dt)
        self.publish_imu(stamp, accel, gyro)

    def publish_odom(self, stamp, v, w):
        qz, qw = math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Planar robot: large variance on unobserved axes. Track slip makes the
        # encoder-derived yaw unreliable, so it gets a much larger variance.
        for i, var in enumerate([0.01, 0.01, 1e6, 1e6, 1e6, 0.5]):
            odom.pose.covariance[i * 7] = var
        for i, var in enumerate([0.01, 1e6, 1e6, 1e6, 1e6, 0.5]):
            odom.twist.covariance[i * 7] = var
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def publish_joints(self, stamp, left_pos, right_pos, left_vel, right_vel):
        js = JointState()
        js.header.stamp = stamp
        js.name = [self.left_joint, self.right_joint]
        js.position = [left_pos, right_pos]
        js.velocity = [left_vel, right_vel]
        self.joint_pub.publish(js)

    def publish_imu(self, stamp, accel, gyro):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        # MPU6050 gives no absolute orientation: mark it as unavailable
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel
        for i in range(3):
            imu.angular_velocity_covariance[i * 4] = 0.0004
            imu.linear_acceleration_covariance[i * 4] = 0.05
        self.imu_pub.publish(imu)

    def destroy_node(self):
        self.running = False
        self.write_line('V 0 0')
        self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

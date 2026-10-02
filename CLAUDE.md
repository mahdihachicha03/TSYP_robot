# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Autonomous tracked (skid-steer) robot. Raspberry Pi 4 (Ubuntu 24.04, aarch64, ROS 2 Jazzy from apt) runs the ROS stack; an ESP32 does motor control, encoders and the MPU6050 IMU; a YDLIDAR G4 provides `/scan`. Hardware details, wiring and calibration procedures are in README.md.

## Commands

All ROS commands need `source /opt/ros/jazzy/setup.bash` (and `source ros2_ws/install/setup.bash` after building).

```bash
# Build the ROS workspace. The YDLidar SDK lives in ~/.local, so both env vars are required
# or ydlidar_ros2_driver fails to configure/link. The Pi has ~1.3 GB free RAM: keep workers low.
cd ros2_ws && CMAKE_PREFIX_PATH=$HOME/.local:$CMAKE_PREFIX_PATH LIBRARY_PATH=$HOME/.local/lib \
  colcon build --symlink-install --parallel-workers 1
colcon build --packages-select robot_bridge     # single package

# Firmware (PlatformIO is in a venv, not on PATH)
cd firmware/esp32 && ~/.platformio-venv/bin/pio run
~/.platformio-venv/bin/pio run -t upload --upload-port /dev/esp32

# Run
ros2 launch robot_bringup bringup.launch.py      # description + bridge + EKF + lidar (use_lidar:=false)
ros2 launch robot_bringup slam.launch.py
ros2 launch robot_bringup navigation.launch.py map:=<abs path to map.yaml>
```

There are no automated tests. The bridge has been verified by running it against a fake ESP32 on a pseudo-terminal (Python `pty.openpty()`, write `E ...` lines, pass the slave path via `--ros-args -p port:=/dev/pts/N`). The fake should wait for the bridge's first `V` command before streaming, because `ros2 run` takes several seconds to start on the Pi.

## Environment constraints

- `sudo` requires a password, so apt installs, udev rules and the `dialout` group cannot be done by Claude. These are in `scripts/install_deps.sh` for the user to run. Until it has been run, `slam_toolbox`, `nav2_bringup`, `robot_localization` and `xacro` may be missing, and launch files that use them fail with `PackageNotFoundError`.
- `python3-venv` lacks ensurepip; the PlatformIO venv was bootstrapped with `venv --without-pip` + `get-pip.py`.
- `ros2_ws/src/ydlidar_ros2_driver` is an external clone (gitignored) and must be the **`humble` branch**: `master` uses a pre-Humble rclcpp API and does not compile on Jazzy.
- Background jobs in non-interactive shells ignore SIGINT; stop test nodes with SIGTERM. Avoid `pkill -f <pattern>` with a pattern that also matches the running shell command.

## Architecture

Data flow: Nav2/teleop → `/cmd_vel` → `esp32_bridge` → serial `V` lines → ESP32 PID → L298N → motors. ESP32 → serial `E` lines (50 Hz) → bridge → `/wheel/odom`, `/imu/data_raw`, `/joint_states` → `robot_localization` EKF → `odom → base_link` TF → slam_toolbox (`map → odom`) / Nav2.

**Serial protocol** (ASCII, 115200 baud) is defined in the header comment of `firmware/esp32/src/main.cpp` and parsed in `robot_bridge/esp32_bridge.py`. Changing it means editing both sides and the protocol table in README.md:
- Pi → ESP32: `V <left> <right>` (motor output shaft rad/s), `P <kp> <ki> <kd> <kf>`, `R`
- ESP32 → Pi: `E <left_ticks> <right_ticks> <ax> <ay> <az> <gx> <gy> <gz>` (cumulative ticks, m/s², rad/s), `I <text>`

**Parameters duplicated across files** that must stay consistent:
- Counts per output revolution (682 = 11 PPR × 4 × 15.5 gear ratio): `firmware/esp32/include/config.h` and `counts_per_rev` in `ros2_ws/src/robot_bringup/config/robot.yaml`
- `sprocket_radius`, `track_separation`: `robot.yaml` and `ros2_ws/src/robot_description/urdf/robot.urdf.xacro`
- Joint names `left_sprocket_joint` / `right_sprocket_joint`: bridge defaults and the URDF

**Skid-steer kinematics:** the bridge treats each track as a wheel of radius `sprocket_radius` (an *effective* radius: track travel per radian of the motor output shaft, absorbing any chain reduction), with effective width `track_separation × turn_slip_factor`.

**Sensor fusion design (`config/ekf.yaml`):** because tracks slip sideways when turning, the EKF fuses only `vx`/`vy` from `/wheel/odom` and `vyaw` from the MPU6050 gyro. Odometry yaw is given a large covariance. The IMU message marks orientation as unavailable (covariance[0] = -1). The bridge publishes no TF by default (`publish_tf: false`); the EKF owns `odom → base_link`.

**Frames:** `base_link` is the root of the URDF, on the ground at the center of the track contact patches (the turning center). There is no `base_footprint`; nav configs reference `base_link`.

**Firmware:** Arduino framework on `espressif32@^6.9.0` (core 2.x; `ledc` calls have a `#if ESP_ARDUINO_VERSION_MAJOR >= 3` branch). All pins, inversions, PID gains and limits are in `include/config.h`. Encoder pins avoid strapping (0, 2, 5, 12, 15) and input-only (34–39) pins because the encoders rely on internal pull-ups. L298N PWM is kept at 1 kHz on purpose.

`tools/motor_encoder_control.py` is a Pi-only gpiozero bench test with the original Pi GPIO wiring; it is not part of the robot runtime.

Values marked TBD in configs (dimensions, slip factor, motor/encoder inversions) are placeholders awaiting calibration on hardware; nothing has been tested on the physical robot yet.

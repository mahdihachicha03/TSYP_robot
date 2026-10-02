"""Robot base: description, ESP32 bridge, EKF, and (optionally) the lidar."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('robot_bringup')
    description = get_package_share_directory('robot_description')
    urdf = os.path.join(description, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='Also start the YDLIDAR G4 driver'),
        DeclareLaunchArgument('robot_params',
                              default_value=os.path.join(bringup, 'config', 'robot.yaml')),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='robot_bridge', executable='esp32_bridge', name='esp32_bridge',
             output='screen', parameters=[LaunchConfiguration('robot_params')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[os.path.join(bringup, 'config', 'ekf.yaml')]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'lidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_lidar'))),
    ])

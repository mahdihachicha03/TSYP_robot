"""Nav2 with AMCL localization on a saved map.

    ros2 launch robot_bringup navigation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'nav2_params.yaml')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map yaml file'),
        DeclareLaunchArgument('params_file', default_value=params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),
    ])

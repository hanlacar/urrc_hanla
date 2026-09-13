#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('lidar_parking_planner'))
    config = str(package_share / 'config' / 'parking_planner.yaml')
    obstacle_config = str(package_share / 'config' / 'obstacle_motion.yaml')
    drive_value = LaunchConfiguration('drive_value')
    encoder_source_verified = LaunchConfiguration('encoder_source_verified')

    return LaunchDescription([
        DeclareLaunchArgument('drive_value', default_value='2.0'),
        DeclareLaunchArgument('encoder_source_verified', default_value='false'),
        Node(
            package='lidar_parking_planner',
            executable='parking_planner_node',
            name='parking_planner_node',
            output='screen',
            parameters=[config, obstacle_config, {
                'constant_drive_test_enabled': True,
                'encoder_source_verified': ParameterValue(
                    encoder_source_verified, value_type=bool),
            }],
        ),
        Node(
            package='lidar_parking_planner',
            executable='constant_drive_test_node',
            name='constant_drive_test_node',
            output='screen',
            parameters=[{
                'drive_value': ParameterValue(drive_value, value_type=float),
                'publish_rate_hz': 20.0,
            }],
        ),
    ])

#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('lidar_parking_planner'))
        / 'config' / 'parking_planner.yaml'
    )
    obstacle_config = str(
        Path(get_package_share_directory('lidar_parking_planner'))
        / 'config' / 'obstacle_motion.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('localization_mode', default_value='scan_only'),
        DeclareLaunchArgument('front_scan_topic', default_value='/front/scan'),
        DeclareLaunchArgument('rear_scan_topic', default_value='/rear/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='lidar_parking_planner',
            executable='parking_planner_node',
            name='parking_planner_node',
            output='screen',
            parameters=[config, obstacle_config, {
                'localization_mode': LaunchConfiguration('localization_mode'),
                'front_scan_topic': LaunchConfiguration('front_scan_topic'),
                'rear_scan_topic': LaunchConfiguration('rear_scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])

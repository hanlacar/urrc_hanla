#!/usr/bin/env python3
"""Standalone real-vehicle avoidance entry point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real = PathJoinSubstitution([
        FindPackageShare('lidar_ws_plus_bringup'),
        'launch', 'real_vehicle.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('route_file', default_value=''),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real), launch_arguments={
                'use_sim_time': 'false',
                'front_serial_port': LaunchConfiguration('front_serial_port'),
                'route_file': LaunchConfiguration('route_file'),
                'avoidance_auto_start': LaunchConfiguration('auto_start'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'enable_avoidance': 'true',
                'enable_mux': 'true',
                'enable_rear_lidar': 'false',
                'enable_rear_tf': 'false',
            }.items()),
    ])

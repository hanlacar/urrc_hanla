#!/usr/bin/env python3
"""Start only the integrated LiDAR owner(s) and their static TF owner(s)."""

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
        DeclareLaunchArgument('enable_rear_lidar', default_value='false'),
        DeclareLaunchArgument('enable_rear_tf', default_value='false'),
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rear_serial_port', default_value='/dev/ttyUSB1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real), launch_arguments={
                'use_sim_time': 'false',
                'enable_rear_lidar': LaunchConfiguration('enable_rear_lidar'),
                'enable_rear_tf': LaunchConfiguration('enable_rear_tf'),
                'front_serial_port': LaunchConfiguration('front_serial_port'),
                'rear_serial_port': LaunchConfiguration('rear_serial_port'),
                'enable_motion_detector': 'false',
                'enable_avoidance': 'false',
                'enable_mux': 'false',
            }.items()),
    ])

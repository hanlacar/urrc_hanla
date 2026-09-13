#!/usr/bin/env python3
"""Integrated saved-map T-parking entry point for the real vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('lidar_ws_plus_bringup')
    parking_share = FindPackageShare('t_parking_sim')
    real_vehicle = PathJoinSubstitution([
        bringup_share, 'launch', 'real_vehicle.launch.py'])
    real_parking = PathJoinSubstitution([
        parking_share, 'launch', 'real_t_parking.launch.py'])
    default_map = PathJoinSubstitution([
        parking_share, 'maps', 'combined_parking_map_real_vehicle.yaml'])
    default_config = PathJoinSubstitution([
        parking_share, 'config', 't_parking_auto.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('parking_config', default_value=default_config),
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rear_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('enable_rear_lidar', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('target_slot', default_value='auto'),
        DeclareLaunchArgument('slot_observation_enabled', default_value='true'),
        DeclareLaunchArgument('fast_planning', default_value='false'),
        DeclareLaunchArgument('return_to_entrance', default_value='true'),
        DeclareLaunchArgument('stop_when_all_wheels_inside', default_value='true'),
        DeclareLaunchArgument('exit_mode', default_value='forward_right'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_vehicle), launch_arguments={
                'use_sim_time': 'false',
                'front_serial_port': LaunchConfiguration('front_serial_port'),
                'rear_serial_port': LaunchConfiguration('rear_serial_port'),
                'enable_rear_lidar': LaunchConfiguration('enable_rear_lidar'),
                'enable_rear_tf': LaunchConfiguration('enable_rear_lidar'),
                'enable_motion_detector': 'true',
                'enable_avoidance': 'false',
                'enable_mux': 'true',
                'use_rviz': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_parking), launch_arguments={
                'map_mode': 'saved',
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'front_scan_topic': '/front/scan',
                'rear_scan_topic': '/rear/scan',
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                'auto_start': LaunchConfiguration('auto_start'),
                'execute': LaunchConfiguration('execute'),
                'target_slot': LaunchConfiguration('target_slot'),
                'slot_observation_enabled': LaunchConfiguration(
                    'slot_observation_enabled'),
                'parking_config': LaunchConfiguration('parking_config'),
                'fast_planning': LaunchConfiguration('fast_planning'),
                'return_to_entrance': LaunchConfiguration('return_to_entrance'),
                'stop_when_all_wheels_inside': LaunchConfiguration(
                    'stop_when_all_wheels_inside'),
                'exit_mode': LaunchConfiguration('exit_mode'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                # Common bringup is the only LiDAR TF owner.
                'start_robot_state_publisher': 'false',
            }.items()),
    ])

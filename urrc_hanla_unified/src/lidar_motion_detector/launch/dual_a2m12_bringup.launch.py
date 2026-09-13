#!/usr/bin/env python3

"""Competition bringup for front/rear RPLIDAR A2M12 sensors.

Prefer stable /dev/serial/by-id or /dev/serial/by-path device names in competition.
After connecting both sensors, verify scans, frame IDs, TF, and detector status.
The shared /lidar_drive command is owned by t_parking_ws, not this detector.
The current detector has no scan-timeout
watchdog; adding one remains a separate core-code safety TODO.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('lidar_motion_detector'))
    front_config = str(package_share / 'config' / 'motion_detector.yaml')
    rear_config = str(package_share / 'config' / 'rear_motion_detector.yaml')

    front_serial_port = LaunchConfiguration('front_serial_port')
    rear_serial_port = LaunchConfiguration('rear_serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    base_frame = LaunchConfiguration('base_frame')
    front_laser_frame = LaunchConfiguration('front_laser_frame')
    rear_laser_frame = LaunchConfiguration('rear_laser_frame')

    front_pose_names = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    front_pose = {
        name: LaunchConfiguration(f'front_laser_{name}')
        for name in front_pose_names
    }
    rear_pose = {
        name: LaunchConfiguration(f'rear_laser_{name}')
        for name in front_pose_names
    }

    declarations = [
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rear_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('front_laser_frame', default_value='front_laser'),
        DeclareLaunchArgument('rear_laser_frame', default_value='rear_laser'),
        DeclareLaunchArgument('front_laser_x', default_value='0.730'),
        DeclareLaunchArgument('front_laser_y', default_value='0.0'),
        DeclareLaunchArgument('front_laser_z', default_value='0.105'),
        DeclareLaunchArgument('front_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('front_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('front_laser_yaw', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_x', default_value='-0.680'),
        DeclareLaunchArgument('rear_laser_y', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_z', default_value='0.155'),
        DeclareLaunchArgument('rear_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'rear_laser_yaw',
            default_value='3.14159265359',
            description='Use 0.0 if the rear sensor faces the same way as the front.',
        ),
        DeclareLaunchArgument('start_front_lidar', default_value='true'),
        DeclareLaunchArgument('start_rear_lidar', default_value='true'),
        DeclareLaunchArgument('start_front_detector', default_value='true'),
        DeclareLaunchArgument('start_rear_detector', default_value='true'),
        DeclareLaunchArgument('start_tf', default_value='true'),
    ]

    front_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='front_rplidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_front_lidar')),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': front_serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': front_laser_frame,
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[('/scan', '/front/scan')],
    )

    rear_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rear_rplidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rear_lidar')),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': rear_serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': rear_laser_frame,
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[('/scan', '/rear/scan')],
    )

    front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_laser_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tf')),
        arguments=[
            '--x', front_pose['x'], '--y', front_pose['y'],
            '--z', front_pose['z'], '--roll', front_pose['roll'],
            '--pitch', front_pose['pitch'], '--yaw', front_pose['yaw'],
            '--frame-id', base_frame, '--child-frame-id', front_laser_frame,
        ],
    )

    rear_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rear_laser_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_tf')),
        arguments=[
            '--x', rear_pose['x'], '--y', rear_pose['y'],
            '--z', rear_pose['z'], '--roll', rear_pose['roll'],
            '--pitch', rear_pose['pitch'], '--yaw', rear_pose['yaw'],
            '--frame-id', base_frame, '--child-frame-id', rear_laser_frame,
        ],
    )

    front_detector = Node(
        package='lidar_motion_detector',
        executable='motion_detector_node',
        name='motion_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_front_detector')),
        parameters=[front_config, {
            'input_scan_topic': '/front/scan',
            'source_scan_frame': front_laser_frame,
            'target_frame': base_frame,
            'output_frame': base_frame,
            'lidar_role': 'front',
            'output_namespace': '/lidar',
        }],
    )

    rear_detector = Node(
        package='lidar_motion_detector',
        executable='motion_detector_node',
        name='rear_motion_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rear_detector')),
        parameters=[rear_config, {
            'input_scan_topic': '/rear/scan',
            'source_scan_frame': rear_laser_frame,
            'target_frame': base_frame,
            'output_frame': base_frame,
            'lidar_role': 'rear',
            'output_namespace': '/rear_lidar',
        }],
    )

    return LaunchDescription(declarations + [
        front_lidar,
        rear_lidar,
        front_tf,
        rear_tf,
        front_detector,
        rear_detector,
    ])


# Verification commands after building and sourcing the workspace:
#   ls -l /dev/serial/by-id/; ls -l /dev/serial/by-path/
#   ros2 topic hz /front/scan; ros2 topic hz /rear/scan
#   ros2 topic echo --once /front/scan; ros2 topic echo --once /rear/scan
#   ros2 run tf2_ros tf2_echo base_link front_laser
#   ros2 run tf2_ros tf2_echo base_link rear_laser
#   ros2 topic echo /lidar_drive_text
#   ros2 topic echo /lidar/final_status_text
#   ros2 topic echo /rear_lidar/final_status_text
#   ros2 topic list | grep -E "front|rear|lidar|scan"
#   ros2 topic info -v /lidar_drive  # detector must not be listed as publisher

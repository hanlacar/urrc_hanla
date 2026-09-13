#!/usr/bin/env python3

# Hardware parameters mirror the official rplidar_a1 launch.
# Only the node name, frame ID, and scan topic remap are front-specific.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='front_laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying USB port connected to the front lidar.',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying USB port baud rate.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Frame ID assigned to front laser scans.',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data.',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether to enable angle compensation.',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar.',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='front_rplidar_node',
            parameters=[
                {
                    'channel_type': channel_type,
                    'serial_port': serial_port,
                    'serial_baudrate': serial_baudrate,
                    'frame_id': frame_id,
                    'inverted': inverted,
                    'angle_compensate': angle_compensate,
                }
            ],
            remappings=[
                ('/scan', '/front/scan'),
            ],
            output='screen',
        ),
    ])

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    flip_x_axis = LaunchConfiguration('flip_x_axis')
    scan_mode = LaunchConfiguration('scan_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='RPLIDAR channel type',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device connected to the front RPLIDAR',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='256000',
            description='Serial baud rate for the RPLIDAR A2M12',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='LaserScan frame ID',
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Reverse the scan ordering when required by the mount',
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Enable angular compensation',
        ),
        DeclareLaunchArgument(
            'flip_x_axis',
            default_value='true',
            description='Rotate the scan indexing by 180 degrees',
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description='RPLIDAR scan mode',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='front_rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': ParameterValue(
                    serial_baudrate, value_type=int),
                'frame_id': frame_id,
                'inverted': ParameterValue(inverted, value_type=bool),
                'angle_compensate': ParameterValue(
                    angle_compensate, value_type=bool),
                'flip_x_axis': ParameterValue(
                    flip_x_axis, value_type=bool),
                'scan_mode': scan_mode,
            }],
        ),
    ])

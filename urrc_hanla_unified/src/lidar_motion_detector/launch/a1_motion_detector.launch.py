#!/usr/bin/env python3

"""Launch the front motion detector together with its base_link->laser static TF.

The front RPLIDAR A1 is physically mounted reversed (180 deg from the vehicle's
actual front), the same situation the rear sensor already handles in
rear_a1_motion_detector.launch.py. Following that design: the mounting flip is
expressed once as a static TF (yaw=pi). The detector uses that TF for scan
points and the physical ROI origin; because points are in base_link, front
drive safety still follows the vehicle's +x direction instead of reusing the
scanner's inverted yaw.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('lidar_motion_detector'))
    detector_config = str(package_share / 'config' / 'motion_detector.yaml')

    target_frame = LaunchConfiguration('target_frame')
    source_scan_frame = LaunchConfiguration('source_scan_frame')
    front_laser_x = LaunchConfiguration('front_laser_x')
    front_laser_y = LaunchConfiguration('front_laser_y')
    front_laser_z = LaunchConfiguration('front_laser_z')
    front_laser_roll = LaunchConfiguration('front_laser_roll')
    front_laser_pitch = LaunchConfiguration('front_laser_pitch')
    front_laser_yaw = LaunchConfiguration('front_laser_yaw')

    arguments = [
        DeclareLaunchArgument('input_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('source_scan_frame', default_value='laser'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('output_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_role', default_value='front'),
        DeclareLaunchArgument('output_namespace', default_value='/lidar'),
        DeclareLaunchArgument('front_laser_x', default_value='0.75'),
        DeclareLaunchArgument('front_laser_y', default_value='0.0'),
        DeclareLaunchArgument('front_laser_z', default_value='0.10'),
        DeclareLaunchArgument('front_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('front_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'front_laser_yaw',
            default_value='3.14159265359',
            description=(
                'Yaw of laser relative to base_link [rad]. front_laser(=laser) '
                'is mounted reversed; +x aligns with base_link -x.'
            ),
        ),
    ]

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_laser_static_tf',
        output='screen',
        arguments=[
            '--x', front_laser_x,
            '--y', front_laser_y,
            '--z', front_laser_z,
            '--roll', front_laser_roll,
            '--pitch', front_laser_pitch,
            '--yaw', front_laser_yaw,
            '--frame-id', target_frame,
            '--child-frame-id', source_scan_frame,
        ],
    )

    detector = Node(
        package='lidar_motion_detector',
        executable='motion_detector_node',
        name='motion_detector_node',
        output='screen',
        parameters=[
            detector_config,
            {
                'input_scan_topic': LaunchConfiguration('input_scan_topic'),
                'source_scan_frame': source_scan_frame,
                'target_frame': target_frame,
                'output_frame': LaunchConfiguration('output_frame'),
                'lidar_role': LaunchConfiguration('lidar_role'),
                'output_namespace': LaunchConfiguration('output_namespace'),
            },
        ],
    )

    return LaunchDescription(arguments + [static_tf, detector])

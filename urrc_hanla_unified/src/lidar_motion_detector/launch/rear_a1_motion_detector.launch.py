#!/usr/bin/env python3

# Dual-LiDAR default: the rear detector uses /rear/scan and rear_laser.
# Keep the front and rear scan topics and TF frames unique when both sensors run.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('lidar_motion_detector'))
    rear_detector_config = str(
        package_share / 'config' / 'rear_motion_detector.yaml'
    )

    base_frame = LaunchConfiguration('base_frame')
    rear_laser_frame = LaunchConfiguration('rear_laser_frame')
    rear_laser_x = LaunchConfiguration('rear_laser_x')
    rear_laser_y = LaunchConfiguration('rear_laser_y')
    rear_laser_z = LaunchConfiguration('rear_laser_z')
    rear_laser_roll = LaunchConfiguration('rear_laser_roll')
    rear_laser_pitch = LaunchConfiguration('rear_laser_pitch')
    rear_laser_yaw = LaunchConfiguration('rear_laser_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'rear_laser_frame',
            default_value='rear_laser',
            description='Unique rear LiDAR frame for dual-sensor operation.',
        ),
        DeclareLaunchArgument('rear_laser_x', default_value='-0.75'),
        DeclareLaunchArgument('rear_laser_y', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_z', default_value='0.10'),
        DeclareLaunchArgument('rear_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'rear_laser_yaw',
            default_value='3.14159265359',
            description=(
                'Yaw of rear_laser relative to base_link [rad]. '
                'rear_laser +x must align with base_link -x.'
            ),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_rear_laser_static_tf',
            output='screen',
            arguments=[
                '--x', rear_laser_x,
                '--y', rear_laser_y,
                '--z', rear_laser_z,
                '--roll', rear_laser_roll,
                '--pitch', rear_laser_pitch,
                '--yaw', rear_laser_yaw,
                '--frame-id', base_frame,
                '--child-frame-id', rear_laser_frame,
            ],
        ),
        Node(
            package='lidar_motion_detector',
            executable='motion_detector_node',
            name='rear_motion_detector_node',
            output='screen',
            parameters=[
                rear_detector_config,
                {
                    'lidar_role': 'rear',
                    'output_namespace': '/rear_lidar',
                    'source_scan_frame': rear_laser_frame,
                    'target_frame': base_frame,
                    'output_frame': base_frame,
                },
            ],
        ),
    ])

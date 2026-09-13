#!/usr/bin/env python3
"""Own the real LiDAR devices, LiDAR TFs, perception, and command mux."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        '1', 'true', 'yes', 'on')


def _validate(context):
    if _as_bool(context, 'use_sim_time'):
        raise RuntimeError('real_vehicle requires use_sim_time:=false')
    if (_as_bool(context, 'enable_lidar')
            and _as_bool(context, 'enable_rear_lidar')):
        front = LaunchConfiguration('front_serial_port').perform(context)
        rear = LaunchConfiguration('rear_serial_port').perform(context)
        if front == rear:
            raise RuntimeError(
                'front_serial_port and rear_serial_port must be different')
    return []


def generate_launch_description():
    share = get_package_share_directory('lidar_ws_plus_bringup')
    motion_share = get_package_share_directory('lidar_motion_detector')
    avoidance_lidar_share = get_package_share_directory('avoidance_lidar')
    avoidance_planner_share = get_package_share_directory('avoidance_planner')
    avoidance_route_share = get_package_share_directory('avoidance_route')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_rear_lidar = LaunchConfiguration('enable_rear_lidar')
    enable_lidar_tf = LaunchConfiguration('enable_lidar_tf')
    enable_rear_tf = LaunchConfiguration('enable_rear_tf')
    enable_motion_detector = LaunchConfiguration('enable_motion_detector')
    enable_avoidance = LaunchConfiguration('enable_avoidance')
    enable_mux = LaunchConfiguration('enable_mux')
    front_frame = LaunchConfiguration('front_laser_frame')
    rear_frame = LaunchConfiguration('rear_laser_frame')
    base_frame = LaunchConfiguration('base_frame')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_rear_lidar', default_value='false'),
        DeclareLaunchArgument('enable_lidar_tf', default_value='true'),
        DeclareLaunchArgument('enable_rear_tf', default_value='false'),
        DeclareLaunchArgument('enable_motion_detector', default_value='true'),
        DeclareLaunchArgument('enable_avoidance', default_value='false'),
        DeclareLaunchArgument('enable_mux', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('route_file', default_value=''),
        DeclareLaunchArgument('avoidance_auto_start', default_value='false'),
        DeclareLaunchArgument(
            'front_serial_port', default_value='/dev/ttyUSB0',
            description=(
                'Front RPLIDAR device fallback; pass /dev/serial/by-id/... '
                'on the vehicle.')),
        DeclareLaunchArgument(
            'rear_serial_port', default_value='/dev/ttyUSB1',
            description=(
                'Rear RPLIDAR device fallback; pass a distinct by-id path.')),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
        DeclareLaunchArgument('front_laser_frame', default_value='front_laser'),
        DeclareLaunchArgument('rear_laser_frame', default_value='rear_laser'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('front_laser_inverted', default_value='true'),
        DeclareLaunchArgument('front_laser_flip_x_axis', default_value='false'),
        DeclareLaunchArgument('rear_laser_inverted', default_value='false'),
        DeclareLaunchArgument('rear_laser_flip_x_axis', default_value='false'),
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
            'rear_laser_yaw', default_value='3.14159265359'),
    ]

    common_lidar = {
        'channel_type': 'serial',
        'serial_baudrate': ParameterValue(
            LaunchConfiguration('serial_baudrate'), value_type=int),
        'angle_compensate': True,
        'scan_mode': LaunchConfiguration('scan_mode'),
        'use_sim_time': use_sim_time,
    }
    front_lidar = Node(
        package='rplidar_ros', executable='rplidar_node',
        name='front_rplidar_node', output='screen',
        condition=IfCondition(enable_lidar), parameters=[common_lidar, {
            'serial_port': LaunchConfiguration('front_serial_port'),
            'frame_id': front_frame,
            'inverted': ParameterValue(
                LaunchConfiguration('front_laser_inverted'), value_type=bool),
            'flip_x_axis': ParameterValue(
                LaunchConfiguration('front_laser_flip_x_axis'), value_type=bool),
            'topic_name': '/front/scan',
        }])
    rear_lidar = Node(
        package='rplidar_ros', executable='rplidar_node',
        name='rear_rplidar_node', output='screen',
        condition=IfCondition(enable_rear_lidar), parameters=[common_lidar, {
            'serial_port': LaunchConfiguration('rear_serial_port'),
            'frame_id': rear_frame,
            'inverted': ParameterValue(
                LaunchConfiguration('rear_laser_inverted'), value_type=bool),
            'flip_x_axis': ParameterValue(
                LaunchConfiguration('rear_laser_flip_x_axis'), value_type=bool),
            'topic_name': '/rear/scan',
        }])

    def static_tf(name, frame, prefix, condition):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=name, output='screen', condition=IfCondition(condition),
            arguments=[
                '--x', LaunchConfiguration(f'{prefix}_x'),
                '--y', LaunchConfiguration(f'{prefix}_y'),
                '--z', LaunchConfiguration(f'{prefix}_z'),
                '--roll', LaunchConfiguration(f'{prefix}_roll'),
                '--pitch', LaunchConfiguration(f'{prefix}_pitch'),
                '--yaw', LaunchConfiguration(f'{prefix}_yaw'),
                '--frame-id', base_frame, '--child-frame-id', frame,
            ])

    front_detector = Node(
        package='lidar_motion_detector', executable='motion_detector_node',
        name='front_motion_detector', output='screen',
        condition=IfCondition(enable_motion_detector), parameters=[
            os.path.join(motion_share, 'config', 'motion_detector.yaml'), {
                'use_sim_time': use_sim_time,
                'input_scan_topic': '/front/scan',
                'source_scan_frame': front_frame,
                'target_frame': base_frame,
                'output_frame': base_frame,
                'lidar_role': 'front',
                'output_namespace': '/lidar',
                'publish_lidar_drive_command': False,
            }])

    avoidance_nodes = [
        Node(
            package='avoidance_lidar', executable='front_lidar_detector',
            name='front_lidar_detector', output='screen',
            condition=IfCondition(enable_avoidance), parameters=[
                os.path.join(avoidance_lidar_share, 'config', 'front_lidar.yaml'),
                {'use_sim_time': use_sim_time}]),
        Node(
            package='avoidance_planner', executable='avoidance_coordinator',
            name='avoidance_coordinator', output='screen',
            condition=IfCondition(enable_avoidance), parameters=[
                os.path.join(
                    avoidance_planner_share, 'config',
                    'avoidance_planner.yaml'),
                {'use_sim_time': use_sim_time,
                 'mode_topic': '/drive_mode',
                 'reference_path_topic': '/dr_viz/reference_path'}]),
        Node(
            package='avoidance_route', executable='route_follower',
            name='route_follower', output='screen',
            condition=IfCondition(enable_avoidance), parameters=[
                os.path.join(
                    avoidance_route_share, 'config', 'route_follower.yaml'), {
                        'use_sim_time': use_sim_time,
                        'route_file': LaunchConfiguration('route_file'),
                        'mode_topic': '/drive_mode',
                        'reference_path_topic': '/dr_viz/reference_path',
                        'auto_start': ParameterValue(
                            LaunchConfiguration('avoidance_auto_start'),
                            value_type=bool),
                    }], remappings=[('/scan_front', '/front/scan')]),
        Node(
            package='avoidance_lidar', executable='lidar_safety',
            name='lidar_safety', output='screen',
            condition=IfCondition(enable_avoidance), parameters=[
                os.path.join(avoidance_lidar_share, 'config', 'safety.yaml'),
                {'use_sim_time': use_sim_time,
                 'mode_topic': '/drive_mode'}]),
        Node(
            package='rviz2', executable='rviz2', name='avoidance_rviz',
            output='screen', condition=IfCondition(
                LaunchConfiguration('use_rviz')),
            arguments=['-d', os.path.join(
                avoidance_lidar_share, 'rviz', 'avoidance_lidar.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]),
    ]

    mux = Node(
        package='lidar_ws_plus_bringup', executable='command_mux',
        name='command_mux', output='screen', condition=IfCondition(enable_mux),
        parameters=[os.path.join(share, 'config', 'command_mux.yaml'), {
            'use_sim_time': use_sim_time,
            'mode_topic': '/drive_mode',
        }])

    return LaunchDescription(declarations + [
        OpaqueFunction(function=_validate),
        front_lidar,
        rear_lidar,
        static_tf(
            'base_to_front_laser_static_tf', front_frame,
            'front_laser', enable_lidar_tf),
        static_tf(
            'base_to_rear_laser_static_tf', rear_frame,
            'rear_laser', enable_rear_tf),
        front_detector,
        *avoidance_nodes,
        mux,
    ])

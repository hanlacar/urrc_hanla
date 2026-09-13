#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('mission_manager'))
    network = str(share / 'routes' / 'route_network_segmented_10.csv')
    display_route = str(share / 'routes' / 'all_a_sim.csv')
    open_rviz = LaunchConfiguration('open_rviz')
    auto_start = LaunchConfiguration('auto_start')
    return LaunchDescription([
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        Node(package='mission_manager', executable='dr_odom_sim',
             name='dr_odom_sim', output='screen', parameters=[{
                 'odom_topic': '/odom', 'drive_topic': '/gps_drive',
                 'wheel_topic': '/gps_wheel', 'wheelbase_m': 0.73,
                 'max_steer_deg': 22.0, 'rate_hz': 30.0,
                 # Exact first pose of START_A.  All simulation nodes remain
                 # in the route network's native coordinate frame.
                 'initial_x': 1.081892, 'initial_y': -0.506308,
                 'initial_yaw_deg': 160.936049786}]),
        Node(package='mission_manager', executable='dr_all_a_sim_inputs',
             name='dr_all_a_sim_inputs', output='screen'),
        Node(package='mission_manager', executable='dr_real_segmented_follower',
             name='dr_real_segmented_follower', output='screen', parameters=[{
                 'network_path': network, 'start_segment': 'START_A',
                 'auto_start': ParameterValue(auto_start, value_type=bool),
                 'align_route_to_start': False,
                 'end_branch_topic': '/dr/end_branch',
                 # In standalone DR simulation there is one virtual vehicle.
                 # Feed both GPS-owned and lidar-owned mission commands to it.
                 'gps_drive_topic': '/gps_drive',
                 'gps_wheel_topic': '/gps_wheel',
                 'lidar_drive_topic': '/gps_drive',
                 'lidar_wheel_topic': '/gps_wheel',
                 # Simulator consumes ROS +left directly. Real MCU uses -1.
                 'steering_sign': 1, 'intersection_wait_sec': 0.1,
                 'end_wait_sec': 0.1, 'stop_line_min_hold_sec': 3.0}]),
        Node(package='mission_manager', executable='dr_route_visualizer',
             name='dr_route_visualizer', output='screen', parameters=[{
                 'route_path': display_route, 'odom_topic': '/odom',
                 'status_topic': '/dr_navigation/status', 'frame_id': 'odom',
                 'align_route_to_start': False, 'max_actual_points': 40000}]),
        Node(package='rviz2', executable='rviz2', name='dr_all_a_rviz',
             output='screen', condition=IfCondition(open_rviz),
             arguments=['-d', str(share / 'rviz' / 'dr_route_live.rviz')]),
    ])

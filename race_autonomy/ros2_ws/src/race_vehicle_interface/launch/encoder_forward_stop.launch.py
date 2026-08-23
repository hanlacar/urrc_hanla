import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_vehicle_interface"),
        "config",
        "encoder_direction.yaml",
    )
    return LaunchDescription([
        Node(
            package="race_vehicle_interface",
            executable="encoder_serial_bridge_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="race_vehicle_interface",
            executable="measured_motion_state_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="race_vehicle_interface",
            executable="forward_stop_controller_node",
            output="screen",
            parameters=[config],
        ),
    ])

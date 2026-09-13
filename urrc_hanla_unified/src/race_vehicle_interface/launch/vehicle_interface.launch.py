import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_vehicle_interface"),
        "config",
        "vehicle_interface.yaml",
    )
    return LaunchDescription([
        Node(
            package="race_vehicle_interface",
            executable="vehicle_interface_node",
            name="vehicle_interface_node",
            output="screen",
            parameters=[config],
        )
    ])

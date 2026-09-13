import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("race_vehicle_interface")
    bridge_config = os.path.join(share, "config", "arduino_bridge.yaml")
    interface_config = os.path.join(share, "config", "vehicle_interface.yaml")

    return LaunchDescription([
        Node(
            package="race_vehicle_interface",
            executable="arduino_serial_bridge_node",
            name="arduino_serial_bridge_node",
            output="screen",
            parameters=[bridge_config],
        ),
        Node(
            package="race_vehicle_interface",
            executable="vehicle_interface_node",
            name="vehicle_interface_node",
            output="screen",
            parameters=[interface_config],
        ),
    ])

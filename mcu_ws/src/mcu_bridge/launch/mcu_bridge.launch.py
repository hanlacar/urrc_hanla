from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("mcu_bridge")) / "config" / "mcu_bridge.yaml"
    return LaunchDescription([
        Node(
            package="mcu_bridge",
            executable="mcu_bridge_node",
            name="mcu_bridge",
            output="screen",
            parameters=[str(config)],
        )
    ])

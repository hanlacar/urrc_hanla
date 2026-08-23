from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("mcu_manager"),
        "config",
        "mcu_manager.yaml",
    )
    return LaunchDescription([
        Node(
            package="mcu_manager",
            executable="mcu_manager_node",
            name="mcu_manager",
            output="screen",
            parameters=[config],
        )
    ])

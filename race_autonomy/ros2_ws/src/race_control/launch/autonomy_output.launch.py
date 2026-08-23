import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_control"), "config", "autonomy_output.yaml"
    )
    return LaunchDescription([
        Node(
            package="race_control",
            executable="autonomy_output",
            name="autonomy_output_node",
            output="screen",
            parameters=[config],
        )
    ])

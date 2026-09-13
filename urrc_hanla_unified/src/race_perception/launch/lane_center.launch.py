import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_perception"),
        "config",
        "lane_center.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="race_perception",
                executable="lane_center",
                name="lane_center",
                output="screen",
                parameters=[config],
            )
        ]
    )




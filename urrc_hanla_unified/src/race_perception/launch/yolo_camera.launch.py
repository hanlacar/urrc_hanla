from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("race_perception"),
        "config",
        "yolo_camera.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="race_perception",
                executable="yolo_camera",
                name="yolo_camera",
                output="screen",
                parameters=[config],
            )
        ]
    )

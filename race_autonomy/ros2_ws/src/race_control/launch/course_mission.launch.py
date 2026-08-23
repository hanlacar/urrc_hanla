from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("race_control")) / "config" / "course_mission.yaml")
    return LaunchDescription([
        Node(package="race_control", executable="course_mission", name="course_mission_node",
             parameters=[config], output="screen"),
    ])

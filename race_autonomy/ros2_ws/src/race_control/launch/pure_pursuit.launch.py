import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("race_control"), "config", "pure_pursuit.yaml")
    return LaunchDescription([Node(
        package="race_control", executable="pure_pursuit", name="pure_pursuit",
        output="screen", parameters=[config],
    )])

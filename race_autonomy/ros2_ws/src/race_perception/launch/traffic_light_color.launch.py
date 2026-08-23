import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("race_perception"), "config", "traffic_light_color.yaml")
    return LaunchDescription([Node(
        package="race_perception", executable="traffic_light_color",
        name="traffic_light_color", output="screen", parameters=[config],
    )])

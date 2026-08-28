from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("race_control")) / "config" / "course_mission.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "vehicle_speed_topic", default_value="/vehicle/speed_mps"),
        DeclareLaunchArgument(
            "vehicle_speed_valid_topic", default_value="/vehicle/speed_valid"),
        Node(package="race_control", executable="course_mission", name="course_mission_node",
             parameters=[config, {
                 "vehicle_speed_topic": LaunchConfiguration("vehicle_speed_topic"),
                 "vehicle_speed_valid_topic": LaunchConfiguration(
                     "vehicle_speed_valid_topic"),
             }], output="screen"),
    ])

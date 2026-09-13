"""Compatibility entry point; control remains implemented by race_control."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="race_perception", executable="yolo_camera", output="screen"),
        Node(package="race_control", executable="pure_pursuit", output="screen"),
    ])

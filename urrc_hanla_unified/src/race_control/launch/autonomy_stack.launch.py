import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception = get_package_share_directory("race_perception")
    control = get_package_share_directory("race_control")
    vehicle = get_package_share_directory("race_vehicle_interface")
    return LaunchDescription([
        Node(
            package="race_perception", executable="yolo_camera", name="yolo_camera",
            output="screen", parameters=[os.path.join(perception, "config", "yolo_camera.yaml")],
        ),
        Node(
            package="race_control", executable="pure_pursuit", name="pure_pursuit",
            output="screen", parameters=[os.path.join(control, "config", "pure_pursuit.yaml")],
        ),
        Node(
            package="race_perception", executable="traffic_light_color",
            name="traffic_light_color", output="screen",
            parameters=[os.path.join(perception, "config", "traffic_light_color.yaml")],
        ),
        Node(
            package="race_vehicle_interface", executable="vehicle_interface_node",
            name="vehicle_interface_node", output="screen",
            parameters=[os.path.join(vehicle, "config", "vehicle_interface.yaml")],
        ),
        Node(
            package="race_vehicle_interface", executable="arduino_serial_bridge_node",
            name="arduino_serial_bridge_node", output="screen",
            parameters=[os.path.join(vehicle, "config", "arduino_bridge.yaml")],
        ),
    ])

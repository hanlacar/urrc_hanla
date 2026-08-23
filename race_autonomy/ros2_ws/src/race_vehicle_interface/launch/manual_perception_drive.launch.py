"""Perception stack plus a disarmed MCU bridge for manual keyboard driving."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control = get_package_share_directory("race_control")
    vehicle = get_package_share_directory("race_vehicle_interface")
    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(control, "launch", "camera_pure_pursuit.launch.py")),
        launch_arguments={"commanded_speed_mps": "0.0"}.items())
    bridge = Node(
        package="race_vehicle_interface", executable="arduino_serial_bridge_node",
        name="arduino_serial_bridge_node", output="screen", parameters=[
            os.path.join(vehicle, "config", "arduino_bridge.yaml"),
            {"allow_transmit": True, "maximum_abs_stage": 3,
             "maximum_steering_deg": 27.0, "require_fresh_feedback": True}])
    return LaunchDescription([
        LogInfo(msg=("MANUAL PERCEPTION DRIVE: perception runs continuously; "
                     "only manual_keyboard_drive_node may publish vehicle commands; "
                     "MCU bridge starts DISARMED and stage is limited to 3.")),
        perception, bridge])

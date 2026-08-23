"""Camera Pure Pursuit plus manually armed steering-only Arduino output."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_share=get_package_share_directory("race_control")
    vehicle_share=get_package_share_directory("race_vehicle_interface")
    perception=IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(control_share,"launch","camera_pure_pursuit.launch.py")))
    bridge=Node(package="race_vehicle_interface",executable="arduino_serial_bridge_node",
        name="arduino_serial_bridge_node",output="screen",parameters=[
            os.path.join(vehicle_share,"config","arduino_bridge.yaml"),
            {"allow_transmit":True,"steering_only":True,"maximum_abs_stage":1}])
    interface=Node(package="race_vehicle_interface",executable="vehicle_interface_node",
        name="vehicle_interface_node",output="screen",parameters=[
            os.path.join(vehicle_share,"config","vehicle_interface.yaml"),
            {"allow_actuation":True,"steering_only":True,"maximum_abs_stage":1}])
    return LaunchDescription([
        LogInfo(msg="STEERING-ONLY TEST: propulsion is hard-locked to stage 0; both command gates start disarmed"),
        perception,bridge,interface])

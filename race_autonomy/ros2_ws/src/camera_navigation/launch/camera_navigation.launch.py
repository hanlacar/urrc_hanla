from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config=str(Path(get_package_share_directory("camera_navigation"))/"config"/"camera_navigation.yaml")
    return LaunchDescription([DeclareLaunchArgument("input_mode",default_value="external",choices=["external"]),Node(package="camera_navigation",executable="camera_path_planner_node",parameters=[config,{"input_mode":LaunchConfiguration("input_mode")}],output="screen"),Node(package="camera_navigation",executable="camera_path_controller_node",parameters=[config],output="screen")])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    control_share = get_package_share_directory("race_control")
    return LaunchDescription([
        DeclareLaunchArgument(
            "vehicle_speed_topic", default_value="/vehicle/speed_mps"),
        DeclareLaunchArgument(
            "vehicle_speed_kph_topic", default_value="/vehicle/speed_kph"),
        DeclareLaunchArgument(
            "vehicle_speed_valid_topic", default_value="/vehicle/speed_valid"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        LogInfo(msg=("Camera perception and path outputs enabled. "
                     "The integrated workspace owns mission decisions and final commands.")),
        include("race_control", "camera_pure_pursuit.launch.py", {
            "commanded_speed_mps": "0.0",
            "speed_feedback_topic": LaunchConfiguration("vehicle_speed_topic"),
            "launch_rqt": LaunchConfiguration("launch_rqt"),
        }),
        Node(
            package="race_control",
            executable="autonomy_output",
            name="autonomy_output_node",
            parameters=[os.path.join(control_share, "config", "autonomy_output.yaml")],
            remappings=[
                ("/vehicle/speed_kph", LaunchConfiguration("vehicle_speed_kph_topic")),
                ("/vehicle/speed_valid", LaunchConfiguration("vehicle_speed_valid_topic")),
            ],
            output="screen",
        ),
    ])

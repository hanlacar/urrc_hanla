import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
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
    vehicle_share = get_package_share_directory("race_vehicle_interface")
    return LaunchDescription([
        DeclareLaunchArgument("use_internal_cmd_mux", default_value="false"),
        DeclareLaunchArgument("initial_control_mode", default_value="IDLE"),
        DeclareLaunchArgument(
            "vehicle_speed_topic", default_value="/vehicle/speed_mps"),
        DeclareLaunchArgument(
            "vehicle_speed_valid_topic", default_value="/vehicle/speed_valid"),
        LogInfo(msg=("11-section camera/Depth/IMU mission decision enabled. "
                     "This launch does not arm or start the Arduino bridge.")),
        include("race_control", "camera_pure_pursuit.launch.py", {"commanded_speed_mps": "0.0"}),
        include("race_control", "course_mission.launch.py", {
            "vehicle_speed_topic": LaunchConfiguration("vehicle_speed_topic"),
            "vehicle_speed_valid_topic": LaunchConfiguration(
                "vehicle_speed_valid_topic"),
        }),
        Node(
            package="race_vehicle_interface",
            executable="cmd_mux_node",
            name="cmd_mux_node",
            parameters=[
                os.path.join(vehicle_share, "config", "cmd_mux.yaml"),
                {"initial_mode": LaunchConfiguration("initial_control_mode")},
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_internal_cmd_mux")),
        ),
    ])

"""Saved-map + required BEV following with a fail-closed MCU_PAD v6 bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = Path(get_package_share_directory("race_control")) / "launch"
    default_model = str(Path(get_package_share_directory("camera_yolo_inference")) /
                        "models" / "hanla_competition_11class_best.engine")
    default_manifest = str(Path(get_package_share_directory("camera_yolo_inference")) /
                           "config" / "class_manifest.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("database"),
        DeclareLaunchArgument("route_file"),
        DeclareLaunchArgument("serial_no", default_value="338122302896"),
        DeclareLaunchArgument("mcu_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("target_speed_mps", default_value="0.15"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        DeclareLaunchArgument("segmentation_model_path", default_value=default_model),
        DeclareLaunchArgument("class_manifest_path", default_value=default_manifest),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("require_cuda", default_value="true"),
        DeclareLaunchArgument("python_executable", default_value="python3"),
        LogInfo(msg=("PAD v6 REAL VEHICLE: starts disabled; valid saved-map, BEV, "
                     "Pure Pursuit and curvature plan are all required; first drive is PWM 40")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / "d456_rtabmap.launch.py")),
            launch_arguments={
                "mode": "localization", "database": LaunchConfiguration("database"),
                "serial_no": LaunchConfiguration("serial_no"),
                "rviz": LaunchConfiguration("rviz")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / "visual_slam_bev_follow.launch.py")),
            launch_arguments={
                "route_file": LaunchConfiguration("route_file"),
                "commanded_speed_mps": LaunchConfiguration("target_speed_mps"),
                "require_bev": "true",
                "align_route_to_start": "true",
                "segmentation_model_path": LaunchConfiguration("segmentation_model_path"),
                "class_manifest_path": LaunchConfiguration("class_manifest_path"),
                "device": LaunchConfiguration("device"),
                "require_cuda": LaunchConfiguration("require_cuda"),
                "python_executable": LaunchConfiguration("python_executable"),
                "launch_rqt": LaunchConfiguration("launch_rqt")}.items()),
        Node(
            package="race_vehicle_interface",
            executable="pad_v6_autonomy_bridge_node",
            name="pad_v6_autonomy_bridge_node", output="screen",
            parameters=[{
                "port": LaunchConfiguration("mcu_port"),
                "maximum_target_speed_mps": ParameterValue(
                    LaunchConfiguration("target_speed_mps"), value_type=float),
            }]),
    ])

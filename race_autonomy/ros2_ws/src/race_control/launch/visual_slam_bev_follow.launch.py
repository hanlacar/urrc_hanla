"""D455/RTAB-Map topics -> YOLO BEV correction -> saved-route Pure Pursuit.

The D456 and RTAB-Map localization are intentionally not launched here. Start
the localization launch first so only one camera process owns the device.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation = get_package_share_directory("camera_navigation")
    control = get_package_share_directory("race_control")
    yolo = get_package_share_directory("camera_yolo_inference")
    default_model = os.path.join(
        yolo, "models", "hanla_competition_11class_best.engine")
    default_manifest = os.path.join(yolo, "config", "class_manifest.yaml")

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            yolo, "launch", "yolo_inference.launch.py")),
        launch_arguments={
            "input_image_topic": "/camera/camera/color/image_raw",
            "input_camera_info_topic": "/camera/camera/color/camera_info",
            "input_width": "640", "input_height": "480",
            "expected_image_width": "640", "expected_image_height": "480",
            "inference_fps": "60.0", "detections_image_fps": "60.0",
            "segmentation_model_path": LaunchConfiguration("segmentation_model_path"),
            "class_manifest_path": LaunchConfiguration("class_manifest_path"),
            "device": LaunchConfiguration("device"),
            "require_cuda": LaunchConfiguration("require_cuda"),
            "python_executable": LaunchConfiguration("python_executable"),
            "launch_rqt": LaunchConfiguration("launch_rqt"),
        }.items())
    planner = Node(
        package="camera_navigation", executable="camera_path_planner_node",
        output="screen", parameters=[
            os.path.join(navigation, "config", "camera_navigation.yaml"),
            {"input_mode": "external",
             "camera_info_topic": "/camera/camera/color/camera_info",
             "camera_x_m": 0.38, "camera_y_m": 0.0, "camera_z_m": 0.97,
             "camera_mount_pitch_deg": -5.0}],
        remappings=[
            ("/camera/path", "/camera/bev/path"),
            ("/camera/path_valid", "/camera/bev/path_valid"),
            ("/camera/path_confidence", "/camera/bev/path_confidence")])
    route = Node(
        package="race_control", executable="visual_slam_route", output="screen",
        parameters=[os.path.join(control, "config", "visual_slam_route.yaml"), {
            "mode": "follow", "route_file": LaunchConfiguration("route_file"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "align_route_to_start": ParameterValue(
                LaunchConfiguration("align_route_to_start"), value_type=bool),
            "require_bev": ParameterValue(LaunchConfiguration("require_bev"), value_type=bool)}])
    pursuit = Node(
        package="race_control", executable="pure_pursuit",
        name="visual_slam_pure_pursuit", output="screen",
        parameters=[os.path.join(control, "config", "pure_pursuit.yaml"), {
            "path_input_type": "nav_path", "path_topic": "/camera/path",
            "nav_path_valid_topic": "/camera/path_valid",
            "nav_path_confidence_topic": "/camera/path_confidence",
            "nav_path_lateral_to_right_sign": -1.0,
            "commanded_speed_mps": LaunchConfiguration("commanded_speed_mps")}])
    speed = Node(
        package="race_control", executable="curvature_speed_planner",
        output="screen", parameters=[
            os.path.join(control, "config", "curvature_speed_planner.yaml"), {
                "path_valid_topic": "/camera/path_valid",
                "path_confidence_topic": "/camera/path_confidence"}])
    return LaunchDescription([
        DeclareLaunchArgument("route_file", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("commanded_speed_mps", default_value="0.0"),
        DeclareLaunchArgument("require_bev", default_value="true"),
        DeclareLaunchArgument("align_route_to_start", default_value="false"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        DeclareLaunchArgument("segmentation_model_path", default_value=default_model),
        DeclareLaunchArgument("class_manifest_path", default_value=default_manifest),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("require_cuda", default_value="true"),
        DeclareLaunchArgument("python_executable", default_value="python3"),
        LogInfo(msg="Hybrid follower starts locked at 0 m/s; D455 RTAB localization must already be running"),
        perception, planner, route, pursuit, speed,
    ])

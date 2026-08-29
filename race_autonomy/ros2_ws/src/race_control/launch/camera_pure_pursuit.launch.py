import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = get_package_share_directory("camera_bringup")
    imu_share = get_package_share_directory("imu_manager")
    yolo_share = get_package_share_directory("camera_yolo_inference")
    navigation_share = get_package_share_directory("camera_navigation")
    control_share = get_package_share_directory("race_control")
    perception_share = get_package_share_directory("race_perception")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, "launch", "d456_bringup.launch.py")
        ),
        launch_arguments={
            "color_width": "640",
            "color_height": "480",
            "color_fps": "60",
        }.items(),
    )
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, "launch", "yolo_inference.launch.py")
        ),
        launch_arguments={
            "input_width": "640",
            "input_height": "480",
            "inference_fps": "40.0",
            "detections_image_fps": "30.0",
            "navigation_bottom_exclusion_ratio": "0.0",
            "expected_image_width": "640",
            "expected_image_height": "480",
            "device": "cuda:0",
            "require_cuda": "true",
        }.items(),
    )
    planner = Node(
        package="camera_navigation",
        executable="camera_path_planner_node",
        name="camera_path_planner_node",
        output="screen",
        parameters=[
            os.path.join(navigation_share, "config", "camera_navigation.yaml"),
            {"input_mode": "external"},
        ],
    )
    imu_pitch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_share, "launch", "imu_manager.launch.py")
        )
    )
    pure_pursuit = Node(
        package="race_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "pure_pursuit.yaml"),
            {
                "path_input_type": "nav_path",
                "path_topic": "/camera/path",
                "nav_path_valid_topic": "/camera/path_valid",
                "nav_path_confidence_topic": "/camera/path_confidence",
                "nav_path_lateral_to_right_sign": -1.0,
                "speed_feedback_topic": LaunchConfiguration("speed_feedback_topic"),
                "commanded_speed_mps": LaunchConfiguration("commanded_speed_mps"),
            },
        ],
    )
    curvature_speed_planner = Node(
        package="race_control",
        executable="curvature_speed_planner",
        name="curvature_speed_planner_node",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "curvature_speed_planner.yaml"),
        ],
    )
    traffic_light = Node(
        package="race_perception",
        executable="traffic_light_color",
        name="traffic_light_color",
        output="screen",
        parameters=[os.path.join(perception_share,"config","traffic_light_color.yaml")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("commanded_speed_mps",default_value="0.0",description="Pure Pursuit target speed; default is propulsion locked"),
        DeclareLaunchArgument("speed_feedback_topic",default_value="/vehicle/speed_mps",description="Measured speed topic for dynamic lookahead"),
        LogInfo(msg="Camera + IMU + YOLO + metric path + Pure Pursuit; vehicle actuation is not launched"),
        camera,
        imu_pitch,
        yolo,
        traffic_light,
        planner,
        curvature_speed_planner,
        pure_pursuit,
    ])

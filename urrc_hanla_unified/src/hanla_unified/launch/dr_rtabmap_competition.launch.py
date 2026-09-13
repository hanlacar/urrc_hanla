"""DR primary control with saved-map localization and signal-only camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package)) / "launch" / filename)),
        launch_arguments=(arguments or {}).items(),
    )


def dr_rviz_nodes(context, mission_share):
    start = LaunchConfiguration("start_segment").perform(context).strip().upper()
    if start not in ("START_A", "START_B"):
        raise RuntimeError("start_segment must be START_A or START_B")
    route = mission_share / "routes" / (
        "all_a_sim.csv" if start == "START_A" else "all_b_sim.csv")
    return [
        Node(
            package="mission_manager",
            executable="dr_route_visualizer",
            name="dr_route_visualizer",
            output="screen",
            parameters=[{
                "route_path": str(route),
                "odom_topic": "/odom",
                "status_topic": "/dr_navigation/status",
                "frame_id": "odom",
                "align_route_to_start": True,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="dr_route_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_dr_rviz")),
            arguments=["-d", str(mission_share / "rviz" / "dr_route_map_live.rviz")],
        ),
    ]


def generate_launch_description():
    yolo = Path(get_package_share_directory("camera_yolo_inference"))
    perception = Path(get_package_share_directory("race_perception"))
    mission_share = Path(get_package_share_directory("mission_manager"))
    python_executable = str(yolo.parents[3] / ".yolo_runtime" / "bin" / "python")

    return LaunchDescription([
        DeclareLaunchArgument("database"),
        DeclareLaunchArgument("start_segment", default_value="START_A"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("fixed_t_branch", default_value=""),
        DeclareLaunchArgument("fixed_v_branch", default_value=""),
        DeclareLaunchArgument("fixed_end_branch", default_value=""),
        DeclareLaunchArgument("serial_no", default_value="338122302896"),
        DeclareLaunchArgument("rtabmap_rviz", default_value="false"),
        DeclareLaunchArgument("launch_dr_rviz", default_value="true"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        DeclareLaunchArgument("enable_lidar_avoidance", default_value="false"),
        DeclareLaunchArgument("front_lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("front_lidar_baud", default_value="256000"),

        # One RealSense process owns the camera and feeds both RTAB-Map and YOLO.
        include("race_control", "d456_rtabmap.launch.py", {
            "mode": "localization",
            "database": LaunchConfiguration("database"),
            "serial_no": LaunchConfiguration("serial_no"),
            "rviz": LaunchConfiguration("rtabmap_rviz"),
            # MCU wheel odometry is the only /odom source used by control.
            # RTAB-Map consumes it and publishes map->odom localization.
            "visual_odometry": "false",
            "odom_topic": "/odom",
            "odom_frame_id": "odom",
        }),
        include("camera_yolo_inference", "yolo_inference.launch.py", {
            "python_executable": python_executable,
            "segmentation_model_path": str(
                yolo / "models" / "hanla_competition_11class_best_rtx5060_fp16.engine"),
            "class_manifest_path": str(yolo / "config" / "class_manifest.yaml"),
            "input_image_topic": "/camera/camera/color/image_raw",
            "input_camera_info_topic": "/camera/camera/color/camera_info",
            "input_width": "640",
            "input_height": "480",
            "expected_image_width": "640",
            "expected_image_height": "480",
            "inference_fps": "30.0",
            "device": "cuda:0",
            "require_cuda": "true",
            "launch_rqt": LaunchConfiguration("launch_rqt"),
        }),
        Node(
            package="race_perception",
            executable="traffic_light_color",
            name="traffic_light_color",
            output="screen",
            parameters=[str(perception / "config" / "traffic_light_color.yaml")],
        ),
        # Publishes /imu/pitch_deg used by the mode-2 5-degree ramp check.
        include("imu_manager", "imu_manager.launch.py"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("lidar_ws_plus_bringup"))
                / "launch" / "real_vehicle.launch.py")),
            condition=IfCondition(LaunchConfiguration("enable_lidar_avoidance")),
            launch_arguments={
                "enable_lidar": "true",
                "enable_rear_lidar": "false",
                "enable_motion_detector": "false",
                "enable_avoidance": "true",
                "enable_mux": "true",
                "avoidance_auto_start": "true",
                "route_file": "",
                "use_rviz": "false",
                "front_serial_port": LaunchConfiguration("front_lidar_port"),
                "serial_baudrate": LaunchConfiguration("front_lidar_baud"),
            }.items(),
        ),
        include("mission_manager", "dr_real_integrated.launch.py", {
            "start_segment": LaunchConfiguration("start_segment"),
            "auto_start": LaunchConfiguration("auto_start"),
            "fixed_t_branch": LaunchConfiguration("fixed_t_branch"),
            "fixed_v_branch": LaunchConfiguration("fixed_v_branch"),
            "fixed_end_branch": LaunchConfiguration("fixed_end_branch"),
        }),
        OpaqueFunction(function=dr_rviz_nodes, args=[mission_share]),
    ])

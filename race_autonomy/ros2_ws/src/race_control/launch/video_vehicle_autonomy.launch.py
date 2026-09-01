"""Drive the T870 from a recorded video through the production MCU stack.

The video test owns the camera topics; no physical RealSense node is started.
The live lidar scan guards the recorded-video camera steering.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = Path(get_package_share_directory("race_control"))
    video_share = Path(get_package_share_directory("camera_yolo_inference"))
    mcu_share = Path(get_package_share_directory("t870_mcu"))

    video_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(video_share / "launch" / "video_cuda_test.launch.py")
        ),
        launch_arguments={
            "video_path": LaunchConfiguration("video_path"),
            "fps": LaunchConfiguration("fps"),
            "loop": LaunchConfiguration("loop"),
            "initial_section": LaunchConfiguration("initial_section"),
            "maximum_drive_stage": LaunchConfiguration("maximum_drive_stage"),
            "launch_rqt": LaunchConfiguration("launch_rqt"),
        }.items(),
    )

    mcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(mcu_share / "launch" / "t870_mcu.launch.py")
        ),
        launch_arguments={
            "port": LaunchConfiguration("mcu_port"),
            "bridge": "true",
            "manager": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_mcu")),
    )

    lidar_guard = Node(
        package="race_perception",
        executable="lidar_camera_guard",
        name="lidar_camera_guard",
        output="screen",
        parameters=[{
            "scan_topic": LaunchConfiguration("scan_topic"),
            "avoid_distance_m": LaunchConfiguration("avoid_distance_m"),
            "clear_distance_m": LaunchConfiguration("clear_distance_m"),
            "clear_confirm_sec": LaunchConfiguration("clear_confirm_sec"),
            "hard_stop_distance_m": LaunchConfiguration("hard_stop_distance_m"),
        }],
    )

    default_video = (
        "/home/parkjinwoo/urrc_hanla/recordings/raw/20260829_170657.mp4"
    )
    return LaunchDescription([
        DeclareLaunchArgument("video_path", default_value=default_video),
        DeclareLaunchArgument("fps", default_value="30.0"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("initial_section", default_value="5"),
        DeclareLaunchArgument("maximum_drive_stage", default_value="3"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        DeclareLaunchArgument("mcu_port", default_value="/dev/t870_mcu"),
        DeclareLaunchArgument("launch_mcu", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("avoid_distance_m", default_value="1.50"),
        DeclareLaunchArgument("clear_distance_m", default_value="1.80"),
        DeclareLaunchArgument("clear_confirm_sec", default_value="0.80"),
        DeclareLaunchArgument("hard_stop_distance_m", default_value="0.45"),
        LogInfo(msg=(
            "RECORDED-VIDEO VEHICLE MODE: camera steering with live lidar "
            "avoidance/stop guard, T870 MCU bridge and manager enabled."
        )),
        video_stack,
        lidar_guard,
        mcu_stack,
    ])

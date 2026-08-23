"""Camera autonomy through the unified external ``t870_mcu`` package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("race_control")
    mcu_share = get_package_share_directory("t870_mcu")
    mcu_config = os.path.join(mcu_share, "config", "t870_mcu.yaml")

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, "launch", "course_autonomy.launch.py")
        ),
        launch_arguments={"use_internal_cmd_mux": "false"}.items(),
    )

    manager = Node(
        package="t870_mcu",
        executable="manager",
        name="mcu_manager",
        parameters=[
            mcu_config,
            {
                # course_mission publishes ramp drive and steering on the
                # camera source while selecting vehicle mode SLOPE.
                "mode_map": [
                    "IDLE:none:none",
                    "NORMAL:camera:camera|center",
                    "INTERSECTION:gps:gps|camera|center",
                    "T_PARK:lidar:lidar|center",
                    "PARALLEL_PARK:lidar:lidar|center",
                    "SLOPE:camera:camera|center",
                    "ACCELERATION:camera:camera|center",
                    "MANUAL:manual:manual|center",
                ],
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("manager")),
    )
    bridge = Node(
        package="t870_mcu",
        executable="bridge",
        name="mcu_bridge",
        parameters=[
            mcu_config,
            {
                "port": LaunchConfiguration("arduino_port"),
                "counts_per_meter": 1073.4,
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("bridge")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "arduino_port",
            default_value="/dev/ttyACM0",
            description="Arduino v28 serial port",
        ),
        DeclareLaunchArgument(
            "manager", default_value="true", description="Start t870 manager"
        ),
        DeclareLaunchArgument(
            "bridge", default_value="true", description="Start t870 serial bridge"
        ),
        LogInfo(msg=(
            "PHYSICAL CAMERA AUTONOMY: D456, IMU, YOLO, path estimation, "
            "Pure Pursuit, mission decision, and unified t870_mcu manager/bridge. "
            "SLOPE mode is wired to camera drive/wheel commands. "
            "The vehicle can move as soon as NORMAL mode and fresh camera "
            "commands are available."
        )),
        autonomy,
        manager,
        bridge,
    ])

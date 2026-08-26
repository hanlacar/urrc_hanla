"""Bring up perception, mission decision, control, and the MCU bridge.

The Arduino bridge starts disarmed and auto-arms once after MCU feedback and
both command topics remain fresh for the configured stability interval.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("race_control")
    vehicle_share = get_package_share_directory("race_vehicle_interface")

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, "launch", "course_autonomy.launch.py")
        ),
        launch_arguments={
            "use_internal_cmd_mux": "true",
            "initial_control_mode": "NORMAL",
        }.items(),
    )
    bridge = Node(
        package="race_vehicle_interface",
        executable="arduino_serial_bridge_node",
        name="arduino_serial_bridge_node",
        output="screen",
        parameters=[
            os.path.join(vehicle_share, "config", "arduino_bridge.yaml"),
            {
                "port": LaunchConfiguration("arduino_port"),
                "allow_transmit": True,
                "maximum_abs_stage": LaunchConfiguration("maximum_abs_stage"),
                "maximum_steering_deg": 27.0,
                "require_fresh_feedback": True,
                "startup_straight_duration_sec": 0.0,
                "startup_straight_stage": 2,
                "startup_auto_center_enabled": False,
                "auto_arm_when_ready": LaunchConfiguration("auto_arm"),
                "auto_arm_stable_sec": 1.0,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "arduino_port",
            default_value=(
                "/dev/serial/by-id/"
                "usb-Arduino__www.arduino.cc__0042_14533303731351115201-if00"
            ),
        ),
        DeclareLaunchArgument("maximum_abs_stage", default_value="3"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        LogInfo(msg=(
            "FULL VEHICLE AUTONOMY: camera/Depth/YOLO, IMU, metric path, "
            "Pure Pursuit, 13-section decision, encoder feedback and MCU bridge. "
            "Arduino command TX auto-arms into normal perception/path control; "
            "the failing startup straight auto-center routine is disabled."
        )),
        autonomy,
        bridge,
    ])

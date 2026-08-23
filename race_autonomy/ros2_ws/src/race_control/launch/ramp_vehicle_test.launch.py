"""Outdoor ramp test with perception, path following, mission logic and MCU drive."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
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
        launch_arguments={"use_internal_cmd_mux": "true"}.items(),
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
                "maximum_abs_stage": 2,
                "maximum_steering_deg": 27.0,
                "require_fresh_feedback": True,
                "startup_straight_duration_sec": 0.0,
                "startup_auto_center_enabled": False,
            },
        ],
    )

    select_ramp_section = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "--once", "/mission/section",
                "std_msgs/msg/Int8", "{data: 2}",
            ],
            output="screen",
        )],
    )

    auto_arm = TimerAction(
        period=LaunchConfiguration("auto_arm_delay_sec"),
        condition=IfCondition(LaunchConfiguration("auto_arm")),
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "service", "call", "/arduino_bridge/set_tx_enabled",
                "std_srvs/srv/SetBool", "{data: true}",
            ],
            output="screen",
        )],
    )

    timed_disarm = TimerAction(
        period=LaunchConfiguration("test_duration_sec"),
        actions=[
            LogInfo(msg="RAMP TEST TIME LIMIT: disarming MCU command TX"),
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/arduino_bridge/set_tx_enabled",
                    "std_srvs/srv/SetBool", "{data: false}",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("arduino_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_arm_delay_sec", default_value="7.0"),
        DeclareLaunchArgument("test_duration_sec", default_value="60.0"),
        LogInfo(msg=(
            "RAMP VEHICLE TEST: section 2 is selected before arming; camera, YOLO, "
            "IMU, path planning, Pure Pursuit, mission control and Arduino are active. "
            "Only active ramp crossing uses stage 2; all other camera driving "
            "is limited to stage 1. Both MCU command and auto-arm are limited "
            "for this test. MCU command TX is automatically disarmed after 60 seconds."
        )),
        autonomy,
        bridge,
        select_ramp_section,
        auto_arm,
        timed_disarm,
    ])

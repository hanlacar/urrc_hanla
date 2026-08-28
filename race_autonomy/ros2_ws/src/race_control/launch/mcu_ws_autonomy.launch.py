"""Run perception and decision outputs for an external T870_MCU workspace.

This launch intentionally starts neither cmd_mux_node nor a serial bridge.
The external computer owns arbitration, MCU feedback and the Arduino port.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_share = get_package_share_directory("race_control")
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, "launch", "course_autonomy.launch.py")
        ),
        launch_arguments={
            "use_internal_cmd_mux": "false",
            "vehicle_speed_topic": "/mcu/speed_mps",
            "vehicle_speed_valid_topic": "/mcu/speed_valid",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id", default_value="10",
            description="ROS 2 DDS domain shared with the external MCU PC"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        LogInfo(msg=(
            "EXTERNAL MCU_WS MODE: publishing /camera_drive, /camera_wheel "
            "and /camera_stop. Local cmd_mux and Arduino bridge are disabled."
        )),
        autonomy,
    ])

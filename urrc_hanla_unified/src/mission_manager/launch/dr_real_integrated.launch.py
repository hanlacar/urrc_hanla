#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    share = get_package_share_directory("mission_manager")
    hanla_share = get_package_share_directory("hanla_unified")
    launch_dir = os.path.join(share, "launch")

    start_segment = LaunchConfiguration("start_segment")
    auto_start = LaunchConfiguration("auto_start")

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "dr_real_segmented_follower.launch.py",
            )
        ),
        launch_arguments={
            "start_segment": start_segment,
            "auto_start": auto_start,
            "end_branch_topic": "/dr/end_branch",
            "fixed_t_branch": LaunchConfiguration("fixed_t_branch"),
            "fixed_v_branch": LaunchConfiguration("fixed_v_branch"),
            "fixed_end_branch": LaunchConfiguration("fixed_end_branch"),
        }.items(),
    )

    parking_selector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "real_parking_slot_selector.launch.py",
            )
        )
    )

    end_branch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                "end_branch_adapter.launch.py",
            )
        )
    )

    final_command = Node(
        package="hanla_unified",
        executable="mission_decision",
        name="mission_decision",
        output="screen",
        parameters=[
            os.path.join(hanla_share, "config", "mission_decision.yaml"),
            # Segmented follower applies the tested real-MCU sign already.
            {"lidar_steering_sign": 1.0},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_segment",
            default_value="START_A",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
        ),
        DeclareLaunchArgument("fixed_t_branch", default_value=""),
        DeclareLaunchArgument("fixed_v_branch", default_value=""),
        DeclareLaunchArgument("fixed_end_branch", default_value=""),

        follower,
        parking_selector,
        end_branch,
        final_command,
    ])

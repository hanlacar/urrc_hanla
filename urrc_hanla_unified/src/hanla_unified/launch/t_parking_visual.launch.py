#!/usr/bin/env python3
from pathlib import Path

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():

    hanla_share = get_package_share_directory("hanla_unified")
    parking_share = get_package_share_directory("t_parking_sim")

    parking_map = LaunchConfiguration("parking_map")

    system_launch = os.path.join(
        hanla_share,
        "launch",
        "system.launch.py",
    )

    rviz_config = os.path.join(
        parking_share,
        "config",
        "t_parking_live.rviz",
    )

    # ------------------------------------------------------------
    # 통합 T주차 시스템
    # ------------------------------------------------------------
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_launch),
        launch_arguments={
            "enable_t_parking": "true",
            "enable_parallel_parking": "false",
            "parking_map": parking_map,
        }.items(),
    )

    # ------------------------------------------------------------
    # T주차 Map Server
    # ------------------------------------------------------------
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="t_parking_map_server",
        namespace="",
        output="screen",
        parameters=[{
            "yaml_filename": parking_map,
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    # map_server가 inactive가 되면 자동 activate
    activate_map = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # 실행 후 1초 뒤 configure
    configure_map = TimerAction(
        period=1.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_server),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    # ------------------------------------------------------------
    # RViz 자동 실행
    # ------------------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="t_parking_rviz",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "parking_map",
            default_value=(
                str(Path.home() / "t_parking_maps") + "/"
                "real_cart_slam_school_T_clean.yaml"
            ),
        ),

        map_server,
        activate_map,
        configure_map,

        system,
        rviz,
    ])

#!/usr/bin/env python3
from pathlib import Path

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():

    parking_share = get_package_share_directory("t_parking_sim")
    lidar_share = get_package_share_directory("lidar_motion_detector")

    parking_map = LaunchConfiguration("parking_map")

    # ==========================================================
    # MAP SERVER
    # ==========================================================

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

    # map_server 자동 configure + activate
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="t_parking_map_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "t_parking_map_server",
            ],
        }],
    )

    # ==========================================================
    # FRONT + REAR RPLIDAR A2M12
    #
    # FRONT : /dev/ttyUSB0 -> /front/scan
    # REAR  : /dev/ttyUSB1 -> /rear/scan
    # ==========================================================

    dual_lidar_launch = os.path.join(
        lidar_share,
        "launch",
        "dual_a2m12_bringup.launch.py",
    )

    dual_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            dual_lidar_launch
        ),
        launch_arguments={
            "front_serial_port": "/dev/ttyUSB0",
            "rear_serial_port": "/dev/ttyUSB1",
        }.items(),
    )

    # ==========================================================
    # RVIZ
    # ==========================================================

    rviz_config = os.path.join(
        parking_share,
        "config",
        "t_parking_live.rviz",
    )

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
        lifecycle_manager,

        dual_lidar,

        rviz,
    ])

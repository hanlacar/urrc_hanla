"""Start camera, LiDAR, DR, and integrated decision (MCU is separate)."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package)) / "launch" / filename)),
        launch_arguments=(arguments or {}).items(), condition=condition)


def validate(context):
    truthy = ("1", "true", "yes", "on")
    t_enabled = LaunchConfiguration("enable_t_parking").perform(context).lower() in truthy
    p_enabled = LaunchConfiguration("enable_parallel_parking").perform(context).lower() in truthy
    dr_enabled = LaunchConfiguration("enable_dr").perform(context).lower() in truthy
    if t_enabled and p_enabled:
        raise RuntimeError("T parking and parallel parking cannot run together")
    route_csv = LaunchConfiguration("dr_route_csv").perform(context).strip()
    route_choice = LaunchConfiguration("dr_route_choice").perform(context).strip().lower()
    if dr_enabled and not route_csv and route_choice not in ("a-c", "a-d", "b-c", "b-d"):
        raise RuntimeError(
            "enable_dr:=true requires dr_route_choice:=a-c|a-d|b-c|b-d "
            "or dr_route_csv:=/absolute/route.csv"
        )
    if (t_enabled or p_enabled) and not LaunchConfiguration("parking_map").perform(context).strip():
        raise RuntimeError("parking mode requires parking_map:=/absolute/map.yaml")
    return []


def launch_dr_follower(context, share, mission_share):
    truthy = ("1", "true", "yes", "on")
    if LaunchConfiguration("enable_dr").perform(context).lower() not in truthy:
        return []

    route_csv = LaunchConfiguration("dr_route_csv").perform(context).strip()
    if route_csv:
        route = Path(route_csv).expanduser()
    else:
        choice = LaunchConfiguration("dr_route_choice").perform(context).strip().lower()
        route = share / "routes" / f"{choice}_dr.csv"
    if not route.is_file():
        raise RuntimeError(f"DR route CSV not found: {route}")

    nodes = [
        Node(
            package="mission_manager", executable="dr_route_follower",
            name="dr_route_follower", output="screen",
            parameters=[{"route_path": str(route), "odom_topic": "/odom",
                         "drive_topic": "/gps_drive", "wheel_topic": "/gps_wheel",
                         "max_steer_deg": 22.0,
                         "steering_sign": -1,
                         "auto_start": LaunchConfiguration("dr_auto_start")}],
        ),
        Node(
            package="mission_manager", executable="dr_route_visualizer",
            name="dr_route_visualizer", output="screen",
            parameters=[{"route_path": str(route), "odom_topic": "/odom",
                         "status_topic": "/dr_navigation/status",
                         "frame_id": "odom", "align_route_to_start": True}],
        ),
    ]
    if LaunchConfiguration("dr_launch_rviz").perform(context).lower() in truthy:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="dr_route_rviz",
            output="screen",
            arguments=["-d", str(mission_share / "rviz" / "dr_route_live.rviz")],
        ))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("hanla_unified"))
    mission_share = Path(get_package_share_directory("mission_manager"))
    fastdds_profile = share / "config" / "fastdds_shm_64mb.xml"
    enable_camera = LaunchConfiguration("enable_camera")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_dr = LaunchConfiguration("enable_dr")
    enable_t_parking = LaunchConfiguration("enable_t_parking")
    enable_parallel_parking = LaunchConfiguration("enable_parallel_parking")
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", str(fastdds_profile)),
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_dr", default_value="false"),
        DeclareLaunchArgument("enable_t_parking", default_value="false"),
        DeclareLaunchArgument("enable_parallel_parking", default_value="false"),
        DeclareLaunchArgument("parking_map", default_value=""),
        DeclareLaunchArgument("dr_route_csv", default_value=""),
        DeclareLaunchArgument("dr_route_choice", default_value="a-c",
                              description="Bundled DR route: a-c, a-d, b-c, or b-d"),
        DeclareLaunchArgument("dr_auto_start", default_value="true",
                              description="Start following the DR route as soon as odometry arrives"),
        DeclareLaunchArgument("dr_launch_rviz", default_value="true",
                              description="Show CSV reference and actual odom paths in RViz"),
        DeclareLaunchArgument("front_lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        OpaqueFunction(function=validate),
        include("race_control", "course_autonomy.launch.py", {
                    "launch_rqt": LaunchConfiguration("launch_rqt"),
                },
                condition=IfCondition(enable_camera)),
        include("lidar_ws_plus_bringup", "real_vehicle.launch.py", {
            "front_serial_port": LaunchConfiguration("front_lidar_port"),
            "enable_lidar": enable_lidar, "enable_motion_detector": enable_lidar,
            "enable_avoidance": enable_lidar, "enable_mux": "false",
            "enable_rear_lidar": "false", "enable_rear_tf": "false",
        }, IfCondition(enable_lidar)),
        OpaqueFunction(function=launch_dr_follower, args=[share, mission_share]),
        include("t_parking_sim", "real_t_parking.launch.py", {
            "map": LaunchConfiguration("parking_map"), "map_mode": "saved",
            "front_scan_topic": "/front/scan", "rear_scan_topic": "/rear/scan",
            "auto_start": "false", "execute": "false", "target_slot": "auto",
            "start_rviz": "false",
        }, IfCondition(enable_t_parking)),
        include("t_parking_sim", "real_parallel_parking.launch.py", {
            "map": LaunchConfiguration("parking_map"), "map_mode": "saved",
            "front_scan_topic": "/front/scan", "rear_scan_topic": "/rear/scan",
            "auto_start": "false", "execute": "false", "target_slot": "auto",
            "start_rviz": "false",
        }, IfCondition(enable_parallel_parking)),
        Node(package="hanla_unified", executable="mission_decision",
             name="mission_decision", output="screen",
             parameters=[str(share / "config" / "mission_decision.yaml")],
             remappings=[("/lidar_drive", "/avoidance/drive_cmd"),
                         ("/lidar_wheel", "/avoidance/wheel_cmd"),
                         ("/lidar_stop", "/avoidance/stop_cmd")]),
    ])

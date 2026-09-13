"""Drive a bundled or explicit CSV route using wheel odometry only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context, share, mission_share):
    route_csv = LaunchConfiguration("route_csv").perform(context).strip()
    choice = LaunchConfiguration("route_choice").perform(context).strip().lower()
    if route_csv:
        route = Path(route_csv).expanduser()
    else:
        if choice not in ("a-c", "a-d"):
            raise RuntimeError("A-course CSV mode requires route_choice:=a-c or a-d")
        route = share / "routes" / f"{choice}_dr.csv"
    if not route.is_file():
        raise RuntimeError(f"DR route CSV not found: {route}")

    nodes = [
        Node(
            package="mission_manager",
            executable="dr_route_follower",
            name="dr_route_follower",
            output="screen",
            parameters=[{
                "route_path": str(route),
                "odom_topic": "/odom",
                "drive_topic": "/gps_drive",
                "wheel_topic": "/gps_wheel",
                "max_steer_deg": 22.0,
                "steering_sign": -1,
                "auto_start": LaunchConfiguration("auto_start"),
                # Match the supplied follower that recovered to the recorded
                # route reliably during the earlier vehicle test.
                "lookahead_forward_m": 0.80,
                "lookahead_reverse_m": 0.60,
                "steering_filter_alpha": 0.65,
                "max_steering_rate_deg_s": 45.0,
                "recovery_cte_m": 0.35,
            }],
        ),
        Node(
            package="hanla_unified",
            executable="csv_only_command",
            name="csv_only_command",
            output="screen",
            parameters=[{"maximum_steering_rate_deg_s": 45.0}],
        ),
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
    ]
    if LaunchConfiguration("launch_rviz").perform(context).lower() in (
            "1", "true", "yes", "on"):
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="dr_route_rviz",
            output="screen",
            arguments=["-d", str(mission_share / "rviz" / "dr_route_live.rviz")],
        ))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("hanla_unified"))
    mission_share = Path(get_package_share_directory("mission_manager"))
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            str(share / "config" / "fastdds_shm_64mb.xml"),
        ),
        DeclareLaunchArgument("route_choice", default_value="a-c"),
        DeclareLaunchArgument("route_csv", default_value=""),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_nodes, args=[share, mission_share]),
    ])

"""Start camera, LiDAR, DR, and integrated decision (MCU is separate)."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    bench_fake_odom = LaunchConfiguration("bench_fake_odom").perform(
        context).lower() in truthy
    simple_compat = LaunchConfiguration("enable_mcu_simple_compat").perform(
        context).lower() in truthy
    if bench_fake_odom and not simple_compat:
        raise RuntimeError(
            "bench_fake_odom requires enable_mcu_simple_compat:=true")
    if (bench_fake_odom and int(LaunchConfiguration(
            "simple_max_forward_drive_level").perform(context)) > 1):
        raise RuntimeError(
            "lifted bench permits simple_max_forward_drive_level <= 1")
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
        DeclareLaunchArgument("avoidance_route_file", default_value=""),
        DeclareLaunchArgument("avoidance_auto_start", default_value="false"),
        DeclareLaunchArgument("avoidance_debug_visualization", default_value="false"),
        DeclareLaunchArgument("avoidance_publish_rejected_points", default_value="false"),
        DeclareLaunchArgument("avoidance_launch_debug_rviz", default_value="false"),
        DeclareLaunchArgument(
            "avoidance_left_curb_inner_y_m", default_value="1.095"),
        DeclareLaunchArgument(
            "avoidance_right_curb_inner_y_m", default_value="-1.095"),
        DeclareLaunchArgument(
            "avoidance_replan_trigger_distance_m", default_value="2.0"),
        DeclareLaunchArgument("enable_mcu_simple_compat", default_value="true"),
        DeclareLaunchArgument("bench_fake_odom", default_value="false"),
        DeclareLaunchArgument(
            "enable_mcu_odom_adapter", default_value="true",
            description=(
                "Real SIMPLE MCU wheel odometry; automatically disabled by "
                "bench_fake_odom.")),
        DeclareLaunchArgument(
            "mcu_odom_counts_per_meter", default_value="199.8",
            description="Initial estimate only; calibrate on the vehicle."),
        DeclareLaunchArgument("mcu_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("mcu_odom_frame", default_value="odom"),
        DeclareLaunchArgument("mcu_odom_base_frame", default_value="base_link"),
        DeclareLaunchArgument("mcu_odom_wheelbase_m", default_value="0.73"),
        DeclareLaunchArgument("mcu_odom_publish_tf", default_value="true"),
        DeclareLaunchArgument(
            "mcu_odom_encoder_topic", default_value="/mcu/encoder"),
        DeclareLaunchArgument(
            "mcu_odom_steering_topic", default_value="/mcu/steer_deg"),
        DeclareLaunchArgument(
            "mcu_odom_drive_topic", default_value="/mcu/applied_drive"),
        DeclareLaunchArgument(
            "mcu_odom_max_encoder_delta_counts", default_value="1000"),
        DeclareLaunchArgument(
            "mcu_odom_encoder_counts_are_signed", default_value="false",
            description=(
                "False for current monotonic ENC_A RISING firmware; true only "
                "when encoder deltas already carry direction.")),
        DeclareLaunchArgument(
            "simple_max_forward_drive_level", default_value="3",
            description=(
                "Production SIMPLE limit; bench_fake_odom requires <= 1")),
        DeclareLaunchArgument("mission_initial_section", default_value="1"),
        DeclareLaunchArgument("launch_rqt", default_value="true"),
        OpaqueFunction(function=validate),
        include("race_control", "course_autonomy.launch.py", {
                    "launch_rqt": LaunchConfiguration("launch_rqt"),
                },
                condition=IfCondition(enable_camera)),
        GroupAction(
            scoped=True,
            actions=[
                include(
                 "lidar_ws_plus_bringup",
                 "real_vehicle.launch.py",
                 {
                     "front_serial_port": LaunchConfiguration(
                            "front_lidar_port"),
                        "enable_lidar": enable_lidar,
                        "enable_motion_detector": enable_lidar,
                        "enable_avoidance": enable_lidar,
                        "enable_mux": "false",

                     # Nested LiDAR bringup에서는 중복 SIMPLE compat 금지
                     "enable_mcu_simple_compat": "false",
                     "bench_fake_odom": "false",

                        "route_file": LaunchConfiguration(
                         "avoidance_route_file"),
                     "avoidance_auto_start": LaunchConfiguration(
                         "avoidance_auto_start"),
                        "debug_visualization": LaunchConfiguration(
                          "avoidance_debug_visualization"),
                     "publish_rejected_points": LaunchConfiguration(
                         "avoidance_publish_rejected_points"),
                     "launch_debug_rviz": LaunchConfiguration(
                          "avoidance_launch_debug_rviz"),
                     "left_curb_inner_y_m": LaunchConfiguration(
                         "avoidance_left_curb_inner_y_m"),
                     "right_curb_inner_y_m": LaunchConfiguration(
                         "avoidance_right_curb_inner_y_m"),
                      "replan_trigger_distance_m": LaunchConfiguration(
                          "avoidance_replan_trigger_distance_m"),
                      "enable_rear_lidar": "false",
                      "enable_rear_tf": "false",
                    },
                    IfCondition(enable_lidar),
              )
         ],
        ),
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
             parameters=[str(share / "config" / "mission_decision.yaml"), {
                 "initial_section": ParameterValue(
                     LaunchConfiguration("mission_initial_section"),
                     value_type=int),
             }],
             remappings=[("/lidar_drive", "/avoidance/drive_cmd"),
                         ("/lidar_wheel", "/avoidance/wheel_cmd"),
                         ("/lidar_stop", "/avoidance/stop_cmd")]),
        Node(
            package="hanla_unified", executable="mcu_odom_adapter",
            name="mcu_odom_adapter", output="screen",
            condition=IfCondition(AndSubstitution(
                LaunchConfiguration("enable_mcu_odom_adapter"),
                NotSubstitution(LaunchConfiguration("bench_fake_odom")))),
            parameters=[{
                "odom_topic": LaunchConfiguration("mcu_odom_topic"),
                "odom_frame": LaunchConfiguration("mcu_odom_frame"),
                "base_frame": LaunchConfiguration("mcu_odom_base_frame"),
                "wheelbase_m": ParameterValue(
                    LaunchConfiguration("mcu_odom_wheelbase_m"),
                    value_type=float),
                "counts_per_meter": ParameterValue(
                    LaunchConfiguration("mcu_odom_counts_per_meter"),
                    value_type=float),
                "publish_tf": ParameterValue(
                    LaunchConfiguration("mcu_odom_publish_tf"),
                    value_type=bool),
                "encoder_topic": LaunchConfiguration(
                    "mcu_odom_encoder_topic"),
                "steering_topic": LaunchConfiguration(
                    "mcu_odom_steering_topic"),
                "drive_topic": LaunchConfiguration("mcu_odom_drive_topic"),
                "max_encoder_delta_counts": ParameterValue(
                    LaunchConfiguration(
                        "mcu_odom_max_encoder_delta_counts"), value_type=int),
                "encoder_counts_are_signed": ParameterValue(
                    LaunchConfiguration(
                        "mcu_odom_encoder_counts_are_signed"), value_type=bool),
            }]),
        Node(
            package="lidar_ws_plus_bringup", executable="mcu_simple_compat",
            name="integrated_mcu_simple_compat", output="screen",
            condition=IfCondition(LaunchConfiguration(
                "enable_mcu_simple_compat")),
            parameters=[{
                "input_drive_topic": "/cmd_drive",
                "input_wheel_topic": "/cmd_wheel",
                "input_stop_topic": "/cmd_stop",
                "drive_input_unit": "mps",
                "wheel_input_type": "float32",
                "stage_per_mps": 4.3956043956,
                "wheel_sign_multiplier": -1,
                "wheel_limit_deg": 22,
                "max_forward_drive_level": ParameterValue(
                    LaunchConfiguration("simple_max_forward_drive_level"),
                    value_type=int),
                "publish_mode_5": False,
                "bench_fake_odom": ParameterValue(
                    LaunchConfiguration("bench_fake_odom"), value_type=bool),
            }]),
    ])

"""Real-car test preset: START_A -> T_A -> V_A -> END_AA."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = Path(get_package_share_directory("hanla_unified")) / "launch" / \
        "dr_rtabmap_competition.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument(
            "database",
            default_value="/home/werwerwer/Downloads/competition_338.db"),
        DeclareLaunchArgument("serial_no", default_value="338122302896"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments={
                "database": LaunchConfiguration("database"),
                "serial_no": LaunchConfiguration("serial_no"),
                "auto_start": LaunchConfiguration("auto_start"),
                "start_segment": "START_A",
                "fixed_t_branch": "T_A",
                "fixed_v_branch": "V_A",
                "fixed_end_branch": "END_AA",
                "launch_dr_rviz": "true",
                "rtabmap_rviz": "false",
                "launch_rqt": "true",
                "enable_lidar_avoidance": "true",
            }.items(),
        ),
    ])

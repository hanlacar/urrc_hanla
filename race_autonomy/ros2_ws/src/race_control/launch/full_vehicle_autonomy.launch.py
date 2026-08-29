"""Run the complete real-vehicle stack on one computer.

This combines camera autonomy with the current T870_MCU manager and bridge.
The legacy cmd_mux and Arduino bridge are intentionally not started, so there
is exactly one command arbiter and one owner of the serial port.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_share = Path(get_package_share_directory("race_control"))
    mcu_share = Path(get_package_share_directory("t870_mcu"))

    ros_domain_id = LaunchConfiguration("ros_domain_id")
    mcu_port = LaunchConfiguration("mcu_port")

    camera_autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(control_share / "launch" / "mcu_ws_autonomy.launch.py")
        ),
        launch_arguments={"ros_domain_id": ros_domain_id}.items(),
    )

    t870_mcu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(mcu_share / "launch" / "t870_mcu.launch.py")
        ),
        launch_arguments={
            "ros_domain_id": ros_domain_id,
            "port": mcu_port,
            "bridge": "true",
            "manager": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="10",
            description="DDS domain shared by autonomy and T870 MCU nodes",
        ),
        DeclareLaunchArgument(
            "mcu_port",
            default_value="auto",
            description="T870 serial port, for example /dev/ttyUSB0",
        ),
        LogInfo(msg=(
            "FULL VEHICLE (single PC): D456 RGB/IMU, TensorRT YOLO, BEV path, "
            "Pure Pursuit, 11-section mission, T870 manager and T870 bridge."
        )),
        camera_autonomy,
        t870_mcu,
    ])

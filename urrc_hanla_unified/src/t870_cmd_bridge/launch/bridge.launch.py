from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('t870_cmd_bridge')
    cfg = os.path.join(share, 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='auto',
            description=('MCU serial device; prefer /dev/serial/by-id/... or '
                         '/dev/t870_mcu on the vehicle.')),
        Node(
            package='t870_cmd_bridge',
            executable='bridge',
            name='t870_cmd_bridge',
            output='screen',
            parameters=[cfg, {'port': LaunchConfiguration('port')}],
        )
    ])

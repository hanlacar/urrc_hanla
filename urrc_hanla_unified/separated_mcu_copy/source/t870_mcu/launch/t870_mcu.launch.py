"""t870_mcu 통합 런치.

기본 (브릿지 + 매니저 둘 다):
    ros2 launch t870_mcu t870_mcu.launch.py

브릿지만 (팀원이 /mcu_drive 로 직접 쏘는 단독 테스트):
    ros2 launch t870_mcu t870_mcu.launch.py manager:=false

매니저만 (차량 없이 중재 로직만 확인):
    ros2 launch t870_mcu t870_mcu.launch.py bridge:=false

포트 변경:
    ros2 launch t870_mcu t870_mcu.launch.py

  포트는 기본값 "auto" 로, USB 장치 정보를 보고 아두이노를 스스로 찾는다.
  특정 포트를 강제할 때만 인자를 준다:
    ros2 launch t870_mcu t870_mcu.launch.py port:=/dev/t870_mcu

다른 설정 파일:
    ros2 launch t870_mcu t870_mcu.launch.py config:=/path/to/my.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("t870_mcu")) / "config" / "t870_mcu.yaml")

    config = LaunchConfiguration("config")
    port = LaunchConfiguration("port")
    use_bridge = LaunchConfiguration("bridge")
    use_manager = LaunchConfiguration("manager")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config,
                              description="통합 파라미터 yaml 경로"),
        DeclareLaunchArgument("port", default_value="auto",
                              description="Arduino 시리얼 포트"),
        DeclareLaunchArgument("bridge", default_value="true",
                              description="시리얼 브릿지 실행 여부"),
        DeclareLaunchArgument("manager", default_value="true",
                              description="명령 중재기 실행 여부"),

        Node(
            package="t870_mcu",
            executable="bridge",
            name="mcu_bridge",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(use_bridge),
            parameters=[config, {"port": port}],
        ),
        Node(
            package="t870_mcu",
            executable="manager",
            name="mcu_manager",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(use_manager),
            parameters=[config],
        ),
    ])

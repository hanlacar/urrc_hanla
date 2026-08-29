"""t870_frames.launch.py — 센서 장착 위치 static TF 발행.

    ros2 launch t870_mcu t870_frames.launch.py

odom 과는 다른 것이다.
    odom       차가 어디까지 왔나 (t870_mcu.launch.py 의 브릿지가 발행)
    이 런치     센서가 차의 어디에 붙어 있나 (고정값)

값은 config/t870_frames.yaml 에서 읽는다. 실측해서 그 파일을 고칠 것.

다른 파일을 쓰려면:
    ros2 launch t870_mcu t870_frames.launch.py frames:=/path/to/my.yaml

확인:
    ros2 run tf2_tools view_frames        # 트리 그림을 PDF 로
    ros2 run tf2_ros tf2_echo base_link laser
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    path = LaunchConfiguration("frames").perform(context)
    with open(path, "r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}

    frames = data.get("frames") or {}
    if not frames:
        raise RuntimeError("%s 에 frames 항목이 없다" % path)

    nodes = []
    for child, cfg in frames.items():
        parent = str(cfg.get("parent", "base_link"))
        # static_transform_publisher 인자 순서:
        #   --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_%s" % child,
            output="log",
            arguments=[
                "--x", str(float(cfg.get("x", 0.0))),
                "--y", str(float(cfg.get("y", 0.0))),
                "--z", str(float(cfg.get("z", 0.0))),
                "--roll", str(float(cfg.get("roll", 0.0))),
                "--pitch", str(float(cfg.get("pitch", 0.0))),
                "--yaw", str(float(cfg.get("yaw", 0.0))),
                "--frame-id", parent,
                "--child-frame-id", str(child),
            ],
        ))
    return nodes


def generate_launch_description():
    default_frames = str(
        Path(get_package_share_directory("t870_mcu")) / "config" / "t870_frames.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("frames", default_value=default_frames,
                              description="센서 장착 위치 yaml"),
        OpaqueFunction(function=_make_nodes),
    ])

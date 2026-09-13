"""Camera entry points describe observations and paths, not mission arbitration."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch_ros.actions import Node


SRC = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def isolated_launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))


@pytest.mark.parametrize("relative_path,expected", [
    ("race_control/launch/course_autonomy.launch.py", {"autonomy_output"}),
    ("race_control/launch/camera_pure_pursuit.launch.py", {
        "camera_path_planner_node", "pure_pursuit", "curvature_speed_planner",
        "traffic_light_color",
    }),
    ("race_control/launch/mcu_ws_autonomy.launch.py", set()),
    ("camera_yolo_inference/launch/video_cuda_test.launch.py", {
        "video_publisher_node", "traffic_light_color", "camera_path_planner_node",
        "camera_path_controller_node", "rqt_image_view", "curvature_speed_planner",
        "autonomy_output",
    }),
])
def test_camera_launches_only_start_perception_and_path_nodes(relative_path, expected):
    spec = importlib.util.spec_from_file_location("camera_launch_under_test", SRC / relative_path)
    module = importlib.util.module_from_spec(spec)
    with patch("launch_ros.actions.Node", wraps=Node) as node:
        spec.loader.exec_module(module)
        module.generate_launch_description()
    assert {call.kwargs["executable"] for call in node.call_args_list} == expected


def test_observation_speed_remapping_preserves_kph_units():
    spec = importlib.util.spec_from_file_location(
        "camera_launch_units", SRC / "race_control/launch/course_autonomy.launch.py")
    module = importlib.util.module_from_spec(spec)
    with patch("launch_ros.actions.Node", wraps=Node) as node:
        spec.loader.exec_module(module)
        module.generate_launch_description()
    mappings = dict(node.call_args.kwargs["remappings"])
    assert "/vehicle/speed_kph" in mappings
    assert "/vehicle/speed_mps" not in mappings

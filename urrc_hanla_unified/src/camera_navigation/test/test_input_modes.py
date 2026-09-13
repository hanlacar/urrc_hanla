import ast
from pathlib import Path
import pytest
from camera_navigation.mock_scenario_generator import external_mask_topics

ROOT=Path(__file__).parents[1]
def test_mock_has_no_external_mask_subscriptions():assert external_mask_topics("mock")==()
def test_external_enables_only_external_masks():assert len(external_mask_topics("external"))==3
def test_invalid_input_mode_fails():
    with pytest.raises(ValueError):external_mask_topics("both")
def test_mock_generator_is_not_ros_node():
    tree=ast.parse((ROOT/"camera_navigation"/"mock_scenario_generator.py").read_text());assert not any(isinstance(n,ast.ClassDef) for n in ast.walk(tree))
def test_planner_has_no_mask_publishers():
    text=(ROOT/"camera_navigation"/"camera_path_planner_node.py").read_text();assert 'create_publisher(Image' not in text
def test_no_actuator_topics_in_sources():
    text="".join(p.read_text() for p in (ROOT/"camera_navigation").glob("*.py"));assert "/cam_drive" not in text and "/cam_wheel" not in text

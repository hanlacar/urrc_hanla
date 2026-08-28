import pytest

from camera_navigation.bev_roi import lateral_extent_m
from pathlib import Path


def test_normal_sections_use_narrow_lateral_roi():
    assert lateral_extent_m(1, 0) == pytest.approx(1.2)
    assert lateral_extent_m(2, 0) == pytest.approx(1.2)


def test_turn_direction_expands_an_ordinary_section():
    assert lateral_extent_m(3, -1) == pytest.approx(1.5)
    assert lateral_extent_m(3, 1) == pytest.approx(1.5)


def test_s_curve_preserves_full_avoidance_width():
    assert lateral_extent_m(5, 0) == pytest.approx(1.5)


@pytest.mark.parametrize("section", (3, 5))
def test_curve_sections_always_use_curve_width(section):
    assert lateral_extent_m(section, 0) == pytest.approx(1.5)
    assert lateral_extent_m(section, -1) == pytest.approx(1.5)
    assert lateral_extent_m(section, 1) == pytest.approx(1.5)


@pytest.mark.parametrize("section", (1, 2, 7, 9, 10, 11))
def test_fixed_normal_sections_ignore_turn_direction(section):
    assert lateral_extent_m(section, -1) == pytest.approx(1.2)
    assert lateral_extent_m(section, 1) == pytest.approx(1.2)


@pytest.mark.parametrize("section", (4, 6, 8))
def test_intersections_use_intersection_width(section):
    assert lateral_extent_m(section, 0) == pytest.approx(1.0)
    assert lateral_extent_m(section, 1) == pytest.approx(1.0)


def test_path_planner_publishes_rqt_roi_and_path_debug_image():
    text=(Path(__file__).parents[1]/"camera_navigation"/
          "camera_path_planner_node.py").read_text()
    assert '"/camera/path_debug_image"' in text
    assert "ROI +/-" in text
    assert "self.publish_debug(road,white,yellow,path" in text

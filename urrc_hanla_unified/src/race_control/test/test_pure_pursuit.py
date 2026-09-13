import math

import pytest

from race_control.pure_pursuit import dynamic_lookahead, rpm_to_speed_mps, select_lookahead_point, steering_angle_deg


def test_dynamic_lookahead_is_bounded():
    assert dynamic_lookahead(0.0, 1.2, 0.7, 4.0) == 1.2
    assert dynamic_lookahead(2.0, 1.2, 0.7, 4.0) == pytest.approx(2.6)
    assert dynamic_lookahead(20.0, 1.2, 0.7, 4.0) == 4.0


def test_measured_speed_drives_configured_lookahead():
    assert dynamic_lookahead(2.98 / 3.6, 0.8, 1.0, 2.0) == pytest.approx(
        1.6277777778
    )


def test_rpm_is_converted_to_vehicle_speed():
    assert rpm_to_speed_mps(60.0, 0.15) == pytest.approx(2.0 * math.pi * 0.15)


def test_path_selection_and_steering():
    points = [[0.5, 0.0], [1.0, 0.2], [2.0, 0.4], [3.0, 0.5]]
    assert select_lookahead_point(points, 1.5) == (2.0, 0.4)
    assert steering_angle_deg((2.0, 0.0), 1.04, 27.0) == 0.0
    assert 0.0 < steering_angle_deg((1.2, 1.0), 1.04, 27.0) <= 27.0


def test_missing_path_is_safe():
    assert select_lookahead_point([], 1.2) is None
    assert steering_angle_deg(None, 1.04, 27.0) == 0.0

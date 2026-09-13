import math

import pytest

from race_control.curvature_speed_planner import (
    CurvatureStagePlanner,
    maximum_path_curvature,
    stage_for_curvature,
)


def test_straight_path_uses_cruise_stage():
    path = [(0.1*i, 0.0) for i in range(41)]
    curvature = maximum_path_curvature(path, spacing_m=0.4)
    assert curvature == pytest.approx(0.0)
    assert stage_for_curvature(curvature) == 2


def test_moderate_curve_uses_slow_stage():
    radius = 3.0
    path = [(radius*math.sin(a), radius*(1.0-math.cos(a)))
            for a in [i*0.04 for i in range(25)]]
    curvature = maximum_path_curvature(path, spacing_m=0.4)
    assert curvature == pytest.approx(1.0/radius, rel=0.08)
    assert stage_for_curvature(curvature) == 1


def test_high_curvature_stops():
    assert stage_for_curvature(0.60) == 0
    assert stage_for_curvature(0.90) == 0


def test_missing_or_too_short_path_stops():
    assert maximum_path_curvature([(0.0, 0.0), (0.1, 0.0)]) is None
    assert stage_for_curvature(None) == 0


def test_high_curvature_continues_at_stage_one_without_stop_go():
    planner = CurvatureStagePlanner()
    assert planner.update(0.7, now=10.0) == (1, "HIGH_CURVATURE_STAGE1")
    assert planner.update(0.7, now=10.99) == (1, "HIGH_CURVATURE_STAGE1")
    assert planner.update(0.7, now=11.0) == (1, "HIGH_CURVATURE_STAGE1")
    assert planner.update(0.8, now=12.0) == (1, "HIGH_CURVATURE_STAGE1")


def test_leaving_and_reentering_curve_does_not_create_entry_stop():
    planner = CurvatureStagePlanner()
    planner.update(0.7, now=1.0)
    assert planner.update(0.1, now=2.0) == (2, "CRUISE")
    assert planner.update(0.7, now=3.0) == (1, "HIGH_CURVATURE_STAGE1")

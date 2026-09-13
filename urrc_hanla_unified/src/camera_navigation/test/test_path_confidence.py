import numpy as np

from camera_navigation.path_confidence import (path_confidence,
                                                validate_candidate_confidence)


def line(y=0.0, length=4.0):
    x = np.linspace(0.5, length, 30)
    return np.column_stack((x, np.full_like(x, y)))


def test_supported_stable_path_has_high_confidence():
    road = np.ones((100, 100), np.uint8)
    score, metrics = path_confidence(
        road, line(.4), line(-.4), line(), line(.01))
    assert score > .7
    assert metrics["center_shift_m"] < .02


def test_first_frame_without_previous_path_is_supported():
    road = np.ones((100, 100), np.uint8)
    score, metrics = path_confidence(
        road, line(.4), line(-.4), line(), None)
    assert score > .7
    assert metrics["previous_path"] == 1.0
    assert metrics["center_shift"] == 1.0


def test_large_lateral_jump_reduces_confidence():
    road = np.ones((100, 100), np.uint8)
    stable, _ = path_confidence(road, line(.4), line(-.4), line(), line())
    jumped, metrics = path_confidence(
        road, line(.4), line(-.4), line(.45), line())
    assert jumped < stable
    assert metrics["lateral_delta_m"] > .4


def test_unsupported_short_path_has_low_confidence():
    road = np.zeros((100, 100), np.uint8)
    score, _ = path_confidence(road, [], [], line(length=.7), line())
    assert score < .4


def test_straight_nonuniform_points_have_good_curvature_score():
    road = np.ones((100, 100), np.uint8)
    path = np.array([[.5, 0.], [.51, 0.], [1.7, 0.], [1.71, 0.], [4., 0.]])
    score, metrics = path_confidence(
        road, line(.4), line(-.4), path, path.copy())
    assert score > .7
    assert metrics["max_curvature_per_m"] < 1e-6
    assert metrics["maximum_curvature"] == 1.0


def test_validator_caps_confidence_for_lateral_jump():
    adjusted, reasons = validate_candidate_confidence(
        .9, {"lateral_delta_m": .31, "max_curvature_per_m": .1,
             "path_span_m": 3.0})
    assert adjusted == .35
    assert reasons == ["PATH_SHIFT"]


def test_validator_reports_multiple_geometric_failures():
    adjusted, reasons = validate_candidate_confidence(
        .8, {"lateral_delta_m": .01, "max_curvature_per_m": 1.2,
             "path_span_m": .8})
    assert adjusted == .35
    assert reasons == ["CURVATURE", "PATH_TOO_SHORT"]

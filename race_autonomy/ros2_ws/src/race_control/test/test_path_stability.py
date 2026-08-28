import numpy as np

from race_control.path_stability import path_jump_metrics, path_spatial_quality


def path(offset=0.0):
    x = np.linspace(0.3, 4.0, 30)
    return np.column_stack((x, np.full_like(x, offset)))


def test_identical_path_is_fully_accurate():
    accuracy, median, maximum, jumped, valid = path_jump_metrics(path(), path())
    assert valid and accuracy == 1.0 and median == 0.0 and maximum == 0.0
    assert not jumped


def test_quarter_meter_lateral_jump_is_reported():
    accuracy, median, maximum, jumped, valid = path_jump_metrics(path(), path(.25))
    assert valid and jumped
    assert median == maximum == .25
    assert accuracy == 0.0


def test_small_shift_produces_fractional_accuracy():
    accuracy, median, _maximum, jumped, valid = path_jump_metrics(path(), path(.05))
    assert valid and not jumped and median == .05
    assert accuracy == .8


def test_nonoverlapping_or_empty_paths_are_invalid():
    assert not path_jump_metrics([], path())[-1]
    old=np.array([[.3,0.],[.4,0.]])
    new=np.array([[4.,0.],[5.,0.]])
    assert not path_jump_metrics(old,new)[-1]


def test_smooth_near_path_has_full_spatial_quality():
    quality, valid, diagnostics = path_spatial_quality(path())
    assert valid and quality == 1.0
    assert diagnostics["nearest_x_m"] == .3


def test_path_at_near_boundary_does_not_score_zero():
    x = np.linspace(1.0, 3.0, 30)
    boundary = np.column_stack((x, np.zeros_like(x)))
    quality, valid, diagnostics = path_spatial_quality(boundary)
    assert valid and quality == 1.0
    assert diagnostics["nearest_x_m"] == 1.0


def test_far_only_path_has_zero_spatial_quality():
    x=np.linspace(2.0,3.0,30)
    far=np.column_stack((x,np.zeros_like(x)))
    quality, valid, diagnostics = path_spatial_quality(far)
    assert not valid and quality == 0.0
    assert diagnostics["nearest_x_m"] == 2.0


def test_zigzag_path_has_zero_spatial_quality():
    x=np.linspace(.3,3.,30)
    y=np.where(np.arange(30)%2,0.5,-0.5)
    quality, valid, diagnostics = path_spatial_quality(np.column_stack((x,y)))
    assert not valid and quality == 0.0
    assert diagnostics["lateral_step_p90_m"] > .15

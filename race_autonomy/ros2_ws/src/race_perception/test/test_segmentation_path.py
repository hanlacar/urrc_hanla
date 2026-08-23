import numpy as np
import pytest

from race_perception.segmentation_path import (
    centerline_from_mask,
    pixels_to_ground_path,
    pixels_to_vehicle_path,
    quaternion_to_pitch_deg,
)


def test_straight_road_creates_centered_path():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[30:, 50:150] = 255
    pixels = centerline_from_mask(mask, band_count=8, minimum_pixels=10, top_ratio=0.3)
    path = pixels_to_vehicle_path(pixels, 200, 100, top_ratio=0.3)
    assert len(path) >= 6
    assert all(abs(point[1]) < 0.02 for point in path)
    assert all(path[i][0] <= path[i + 1][0] for i in range(len(path) - 1))


def test_empty_mask_has_no_path():
    assert centerline_from_mask(np.zeros((20, 30), dtype=np.uint8)) == []


def test_ground_projection_center_ray_at_45_degrees():
    path = pixels_to_ground_path(
        [(50, 50)], 100.0, 100.0, 50.0, 50.0, 1.0, 45.0,
    )
    assert path == [[1.0, 0.0]]


def test_ground_projection_lateral_right_sign():
    path = pixels_to_ground_path(
        [(60, 50)], 100.0, 100.0, 50.0, 50.0, 1.0, 45.0,
    )
    assert path[0][0] == 1.0
    assert path[0][1] > 0.0


def test_quaternion_to_pitch_deg():
    half = np.deg2rad(10.0) / 2.0
    assert quaternion_to_pitch_deg(0.0, np.sin(half), 0.0, np.cos(half)) == pytest.approx(10.0)

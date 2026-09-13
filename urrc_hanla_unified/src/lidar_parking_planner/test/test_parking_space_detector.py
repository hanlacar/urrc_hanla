import math

import numpy as np

from lidar_parking_planner.parking_space_detector import (
    DetectorConfig,
    GridMap,
    MapMetadata,
    ParkingCandidate,
    ParkingSpaceDetector,
)


def make_grid(value=0, resolution=0.05):
    metadata = MapMetadata(
        resolution=resolution,
        width=240,
        height=240,
        origin_x=-6.0,
        origin_y=-6.0,
        origin_yaw=0.0,
    )
    return GridMap(
        np.full((metadata.height, metadata.width), value, dtype=np.int16),
        metadata,
    )


def make_detector():
    return ParkingSpaceDetector(DetectorConfig(
        sample_step_m=0.5,
        max_candidates_per_type=4,
        boundary_band_m=0.0,
    ))


def test_grid_index_converts_to_world_cell_center():
    metadata = MapMetadata(
        resolution=0.5,
        width=20,
        height=20,
        origin_x=1.0,
        origin_y=2.0,
        origin_yaw=math.pi / 2.0,
    )
    grid = GridMap(np.zeros((20, 20), dtype=np.int16), metadata)
    world_x, world_y = grid.grid_to_world(1, 2)
    assert math.isclose(world_x, -0.25, abs_tol=1e-9)
    assert math.isclose(world_y, 2.75, abs_tol=1e-9)


def test_world_coordinate_converts_to_grid_index():
    grid = make_grid(resolution=0.5)
    assert grid.world_to_grid(-5.25, -4.75) == (1, 2)
    assert grid.world_to_grid(-6.01, -4.75) is None


def test_unknown_cells_are_not_treated_as_free():
    grid = make_grid(value=-1)
    detector = make_detector()
    candidate = ParkingCandidate(
        parking_type='T',
        center_x=1.0,
        center_y=-1.5,
        yaw=-math.pi / 2.0,
        width=1.2,
        length=1.8,
        confidence=1.0,
    )
    metrics = detector.evaluate_candidate(grid, candidate)
    assert metrics.free_ratio == 0.0
    assert metrics.unknown_ratio == 1.0
    assert detector.detect(grid, 0.0, 0.0, 0.0) == []


def test_empty_t_rectangle_is_detected():
    candidates = make_detector().detect(make_grid(), 0.0, 0.0, 0.0)
    assert any(candidate.parking_type == 'T' for candidate in candidates)


def test_t_rectangle_with_obstacle_is_rejected():
    grid = make_grid()
    detector = make_detector()
    candidate = ParkingCandidate(
        parking_type='T',
        center_x=1.0,
        center_y=-1.5,
        yaw=-math.pi / 2.0,
        width=1.2,
        length=1.8,
        confidence=1.0,
    )
    length, width = detector.required_clearance('T')
    columns, rows = np.meshgrid(
        np.arange(grid.width), np.arange(grid.height))
    world_x, world_y = grid.grid_to_world(columns, rows)
    dx = world_x - candidate.center_x
    dy = world_y - candidate.center_y
    cosine = math.cos(candidate.yaw)
    sine = math.sin(candidate.yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    obstacle_mask = (
        (np.abs(local_x) <= length / 2.0)
        & (np.abs(local_y) <= width / 2.0))
    grid.data[obstacle_mask] = 100

    metrics = detector.evaluate_candidate(grid, candidate)
    assert metrics.occupied_ratio > 0.0
    assert detector._candidate_if_clear(
        grid, 'T', candidate.center_x, candidate.center_y,
        candidate.yaw, candidate.length, candidate.width) is None


def test_empty_parallel_rectangle_is_detected():
    candidates = make_detector().detect(make_grid(), 0.0, 0.0, 0.0)
    assert any(
        candidate.parking_type == 'PARALLEL'
        for candidate in candidates)


def test_candidate_outside_map_is_rejected():
    detector = make_detector()
    candidate = ParkingCandidate(
        parking_type='PARALLEL',
        center_x=5.8,
        center_y=5.8,
        yaw=0.0,
        width=1.0,
        length=2.0,
        confidence=1.0,
    )
    assert not detector.evaluate_candidate(
        make_grid(), candidate).inside_map

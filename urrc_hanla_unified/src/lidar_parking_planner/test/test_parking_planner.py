import math

from lidar_parking_planner.parking_planner_node import ParkingPlanner


class Scan:
    range_min = 0.1
    range_max = 10.0
    ranges = [math.nan, math.inf, 0.05, 0.5, 3.0, 11.0]


def test_command_clamp_respects_requested_limits():
    assert ParkingPlanner._clamp(0.30, 0.0, 0.25) == 0.25
    assert ParkingPlanner._clamp(-0.30, -0.20, 0.0) == -0.20
    assert ParkingPlanner._clamp(50.0, -45.0, 45.0) == 45.0


def test_slot_point_count_uses_rectangular_roi():
    points = [(0.5, -1.5), (0.9, -1.1), (-0.5, -1.5), (2.0, -1.5)]
    assert ParkingPlanner._count_in_box(points, 0.0, 1.0, -2.0, -1.0) == 2
    assert ParkingPlanner._count_in_box(points, -1.0, 0.0, -2.0, -1.0) == 1


def test_minimum_scan_range_ignores_invalid_values():
    assert ParkingPlanner._minimum_valid_range(Scan()) == 0.5

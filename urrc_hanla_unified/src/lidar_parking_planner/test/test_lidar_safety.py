import math

import pytest

from lidar_parking_planner.lidar_safety import LidarSafetyGate


class Scan:
    def __init__(self):
        self.angle_min = -math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.10
        self.range_max = 10.0
        self.ranges = [math.inf] * 361

    def set_range(self, angle_deg, distance):
        index = round(
            (math.radians(angle_deg) - self.angle_min) / self.angle_increment)
        self.ranges[index] = distance
        return self


def confirmed_gate(distance):
    gate = LidarSafetyGate()
    scan = Scan().set_range(0.0, distance)
    gate.update_scan(scan)
    gate.update_scan(scan)
    return gate


@pytest.mark.parametrize('distance', [1.0, 0.8, 0.51])
def test_distance_above_stop_threshold_does_not_stop(distance):
    assert confirmed_gate(distance).state == LidarSafetyGate.CLEAR


@pytest.mark.parametrize('distance', [0.50, 0.49, 0.30])
def test_distance_at_or_below_stop_threshold_stops(distance):
    assert confirmed_gate(distance).state == LidarSafetyGate.OBSTACLE_STOP


def test_stop_requires_configured_number_of_consecutive_scans():
    gate = LidarSafetyGate(stop_confirm_scans=2)
    scan = Scan().set_range(0.0, 0.30)
    gate.update_scan(scan)
    assert gate.state != LidarSafetyGate.OBSTACLE_STOP
    gate.update_scan(scan)
    assert gate.state == LidarSafetyGate.OBSTACLE_STOP


def test_rear_obstacle_is_outside_front_sector():
    scan = Scan().set_range(0.0, 1.0).set_range(180.0, 0.30)
    assert confirmed_gate_from_scan(scan).state == LidarSafetyGate.CLEAR


def test_side_obstacle_is_outside_front_sector():
    scan = Scan().set_range(0.0, 1.0).set_range(45.0, 0.30)
    assert confirmed_gate_from_scan(scan).state == LidarSafetyGate.CLEAR


def confirmed_gate_from_scan(scan):
    gate = LidarSafetyGate()
    gate.update_scan(scan)
    gate.update_scan(scan)
    return gate


def test_nan_and_infinity_are_ignored():
    scan = Scan().set_range(-1.0, math.nan).set_range(0.0, math.inf)
    scan.set_range(1.0, 1.0)
    gate = confirmed_gate_from_scan(scan)
    assert gate.front_min_distance == 1.0
    assert gate.state == LidarSafetyGate.CLEAR


def test_values_below_range_min_are_ignored():
    scan = Scan().set_range(0.0, 0.05).set_range(1.0, 1.0)
    gate = confirmed_gate_from_scan(scan)
    assert gate.front_min_distance == 1.0
    assert gate.state == LidarSafetyGate.CLEAR


def test_zero_and_values_above_range_max_are_ignored():
    scan = Scan().set_range(-1.0, 0.0).set_range(0.0, 11.0)
    scan.set_range(1.0, 1.0)
    gate = confirmed_gate_from_scan(scan)
    assert gate.front_min_distance == 1.0
    assert gate.state == LidarSafetyGate.CLEAR


def test_minimum_valid_range_is_enforced_above_sensor_range_min():
    scan = Scan()
    scan.range_min = 0.01
    scan.set_range(0.0, 0.04).set_range(1.0, 1.0)
    gate = confirmed_gate_from_scan(scan)
    assert gate.front_min_distance == 1.0
    assert gate.state == LidarSafetyGate.CLEAR


def test_hysteresis_keeps_stop_between_thresholds():
    gate = confirmed_gate(0.30)
    gate.update_scan(Scan().set_range(0.0, 0.55))
    assert gate.state == LidarSafetyGate.OBSTACLE_STOP


def test_clear_requires_consecutive_scans_at_resume_distance():
    gate = confirmed_gate(0.30)
    clear_scan = Scan().set_range(0.0, 0.61)
    for _ in range(gate.clear_confirm_scans - 1):
        gate.update_scan(clear_scan)
        assert gate.state == LidarSafetyGate.OBSTACLE_STOP
    gate.update_scan(clear_scan)
    assert gate.state == LidarSafetyGate.CLEAR


def test_scan_timeout_stops_drive():
    gate = confirmed_gate(1.0)
    gate.update_timeout(0.51)
    drive, steering = gate.filter_command(0.2, 12.0)
    assert gate.state == LidarSafetyGate.LIDAR_TIMEOUT
    assert drive == 0.0
    assert steering == 12.0


def test_obstacle_clamps_nonzero_drive_without_changing_wheel():
    gate = confirmed_gate(0.30)
    drive, steering = gate.filter_command(0.2, -27.0)
    assert drive == 0.0
    assert steering == -27.0

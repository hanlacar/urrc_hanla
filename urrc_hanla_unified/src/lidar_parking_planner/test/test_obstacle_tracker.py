import math

from lidar_parking_planner.obstacle_tracker import (
    DYNAMIC_APPROACHING,
    DYNAMIC_RECEDING,
    ObstacleSafetyGate,
    ObstacleTracker,
    SafetyConfig,
    STATIC,
    Track,
    TrackerConfig,
    UNKNOWN,
)
import pytest


class Scan:
    def __init__(self):
        self.angle_min = math.radians(-90.0)
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.10
        self.range_max = 12.0
        self.ranges = [math.inf] * 181

    def obstacle(self, angle_deg, distance, point_count=5):
        first = round(
            (math.radians(angle_deg) - self.angle_min) / self.angle_increment)
        first -= point_count // 2
        for offset in range(point_count):
            self.ranges[first + offset] = distance
        return self


def tracker(**overrides):
    values = {
        'velocity_filter_alpha': 1.0,
        'classification_confirm_frames': 3,
    }
    values.update(overrides)
    return ObstacleTracker(TrackerConfig(**values))


def classify_sequence(ranges, ego_speed):
    detector = tracker()
    tracks = []
    for index, distance in enumerate(ranges):
        tracks = detector.update(
            Scan().obstacle(0.0, distance), index * 0.1,
            ego_speed_mps=ego_speed, encoder_valid=True)
    assert len(tracks) == 1
    return tracks[0]


def test_stationary_vehicle_and_wall_classify_static():
    result = classify_sequence([2.0, 2.0, 2.0, 2.0], 0.0)
    assert result.classification == STATIC
    assert result.filtered_object_speed == pytest.approx(0.0, abs=1e-6)


def test_forward_ego_motion_is_removed_from_static_wall():
    result = classify_sequence([2.0, 1.95, 1.90, 1.85], 0.5)
    assert result.raw_range_rate == pytest.approx(-0.5, abs=0.02)
    assert result.ego_radial_speed == pytest.approx(0.5, abs=0.002)
    assert result.filtered_object_speed == pytest.approx(0.0, abs=0.02)
    assert result.classification == STATIC


def test_reverse_ego_motion_is_removed_from_static_wall():
    result = classify_sequence([2.0, 2.05, 2.10, 2.15], -0.5)
    assert result.raw_range_rate == pytest.approx(0.5, abs=0.02)
    assert result.filtered_object_speed == pytest.approx(0.0, abs=0.02)
    assert result.classification == STATIC


def test_stationary_vehicle_detects_approaching_object():
    result = classify_sequence([2.0, 1.95, 1.90, 1.85], 0.0)
    assert result.classification == DYNAMIC_APPROACHING


def test_stationary_vehicle_detects_receding_object():
    result = classify_sequence([1.0, 1.05, 1.10, 1.15], 0.0)
    assert result.classification == DYNAMIC_RECEDING


def test_same_direction_world_motion_is_preserved_after_compensation():
    result = classify_sequence([2.0, 1.98, 1.96, 1.94], 0.5)
    assert result.filtered_object_speed == pytest.approx(0.3, abs=0.02)
    assert result.classification == DYNAMIC_RECEDING


def test_small_range_noise_remains_static():
    result = classify_sequence([2.0, 2.004, 1.998, 2.003], 0.0)
    assert result.classification == STATIC


def test_nan_inf_zero_and_out_of_range_values_are_ignored():
    scan = Scan()
    scan.ranges[88:93] = [math.nan, math.inf, 0.0, 0.05, 13.0]
    assert tracker().scan_to_clusters(scan, 0.0) == []


def test_single_velocity_glitch_does_not_change_confirmed_classification():
    detector = tracker(max_reasonable_object_speed_mps=2.0)
    for index in range(4):
        tracks = detector.update(
            Scan().obstacle(0.0, 2.0), index * 0.1, 0.0, True)
    assert tracks[0].classification == STATIC
    # Large enough to reject as a 3 m/s glitch, but still inside the configured
    # association gate so this exercises velocity rejection rather than a new ID.
    tracks = detector.update(Scan().obstacle(0.0, 1.7), 0.4, 0.0, True)
    assert tracks[0].classification == STATIC


def test_moving_obstacle_keeps_track_id():
    detector = tracker()
    first = detector.update(Scan().obstacle(0.0, 2.0), 0.0, 0.0, True)[0]
    second = detector.update(Scan().obstacle(1.0, 1.95), 0.1, 0.0, True)[0]
    assert second.track_id == first.track_id


def test_separated_objects_receive_different_ids():
    tracks = tracker().update(
        Scan().obstacle(-20.0, 1.0).obstacle(20.0, 1.0), 0.0, 0.0, True)
    assert len(tracks) == 2
    assert tracks[0].track_id != tracks[1].track_id


def test_track_survives_one_missing_frame():
    detector = tracker(track_timeout_sec=0.5)
    first_id = detector.update(
        Scan().obstacle(0.0, 1.0), 0.0, 0.0, True)[0].track_id
    assert detector.update(Scan(), 0.1, 0.0, True) == []
    reappeared = detector.update(
        Scan().obstacle(0.0, 1.0), 0.2, 0.0, True)[0]
    assert reappeared.track_id == first_id


def test_cluster_extent_crossing_front_sector_is_considered_front():
    clusters = tracker().scan_to_clusters(
        Scan().obstacle(32.0, 1.0, point_count=7), 0.0)
    assert len(clusters) == 1
    assert clusters[0].center_angle > math.radians(30.0)
    assert clusters[0].intersects_front_sector


def make_track(distance, classification, speed=0.0):
    return Track(
        track_id='OBS_0001', center_x=distance, center_y=0.0,
        center_range=distance, center_angle=0.0, min_range=distance,
        width=0.1, point_count=5, timestamp=0.0,
        front_min_range=distance, first_seen=0.0, last_seen=0.0,
        filtered_object_speed=speed, classification=classification)


@pytest.mark.parametrize('classification', [
    STATIC, DYNAMIC_APPROACHING, DYNAMIC_RECEDING, UNKNOWN,
])
def test_all_obstacle_types_stop_at_049_m(classification):
    gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    gate.update([make_track(0.49, classification)])
    assert gate.should_stop


def test_obstacle_at_051_m_does_not_stop_by_default():
    gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    gate.update([make_track(0.51, STATIC)])
    assert not gate.should_stop


def test_stop_hysteresis_holds_at_055_then_clears_after_three_frames():
    gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    gate.update([make_track(0.49, UNKNOWN)])
    gate.update([make_track(0.55, UNKNOWN)])
    assert gate.should_stop
    for _ in range(2):
        gate.update([make_track(0.61, UNKNOWN)])
        assert gate.should_stop
    gate.update([make_track(0.61, UNKNOWN)])
    assert not gate.should_stop


def test_scan_timeout_stops():
    gate = ObstacleSafetyGate(SafetyConfig())
    gate.update([])
    gate.update_timeout(0.51)
    assert gate.state == ObstacleSafetyGate.LIDAR_TIMEOUT
    assert gate.filter_drive(0.2) == 0.0


def test_restored_scan_with_hazard_never_reopens_gate():
    gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=2))
    gate.update([])
    gate.update_timeout(0.51)
    gate.update([make_track(0.49, UNKNOWN)])
    assert gate.state == ObstacleSafetyGate.OBSTACLE_STOP
    assert gate.should_stop


def test_encoder_timeout_forces_unknown_but_close_obstacle_still_stops():
    detector = tracker(classification_confirm_frames=1)
    detector.update(Scan().obstacle(0.0, 0.50), 0.0, 0.0, True)
    tracks = detector.update(Scan().obstacle(0.0, 0.49), 0.1, 0.0, False)
    assert tracks[0].classification == UNKNOWN
    gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    gate.update(tracks)
    assert gate.should_stop


def test_commanded_turn_suppresses_linear_only_motion_classification():
    detector = tracker(classification_confirm_frames=1)
    detector.update(Scan().obstacle(0.0, 2.0), 0.0, 0.5, True)
    tracks = detector.update(
        Scan().obstacle(0.0, 1.95), 0.1, 0.5, True,
        motion_classification_allowed=False)
    assert tracks[0].classification == UNKNOWN


def test_ttc_stop_is_optional():
    approaching = make_track(1.0, DYNAMIC_APPROACHING, speed=-1.0)
    disabled = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    disabled.update([approaching])
    assert not disabled.should_stop
    enabled = ObstacleSafetyGate(SafetyConfig(
        stop_confirm_frames=1, enable_ttc_stop=True, ttc_stop_sec=1.5))
    enabled.update([approaching])
    assert enabled.should_stop

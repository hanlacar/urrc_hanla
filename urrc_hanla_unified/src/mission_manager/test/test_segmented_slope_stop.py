import time

from mission_manager.dr_real_segmented_follower_node import (
    DrRealSegmentedFollower,
)


def follower(pitch, age=0.0):
    node = object.__new__(DrRealSegmentedFollower)
    node.pitch_deg = pitch
    node.last_pitch_time = time.monotonic() - age
    node.imu_pitch_timeout = 0.5
    node.slope_stop_threshold = 5.0
    return node


def test_slope_stop_requires_five_degrees_at_waypoint():
    assert not DrRealSegmentedFollower._slope_stop_required(follower(4.99))
    assert DrRealSegmentedFollower._slope_stop_required(follower(5.0))
    assert DrRealSegmentedFollower._slope_stop_required(follower(-5.0))


def test_slope_stop_rejects_stale_imu():
    assert not DrRealSegmentedFollower._slope_stop_required(
        follower(8.0, age=0.6))


def test_slope_stop_rejects_missing_imu():
    node = follower(8.0)
    node.last_pitch_time = None
    assert not DrRealSegmentedFollower._slope_stop_required(node)

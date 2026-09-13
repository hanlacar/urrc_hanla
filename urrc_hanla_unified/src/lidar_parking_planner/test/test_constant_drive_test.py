import inspect
import math

from lidar_parking_planner import constant_drive_test_node
from lidar_parking_planner.constant_drive_test_node import (
    DEFAULT_DRIVE_VALUE,
    DEFAULT_PUBLISH_RATE_HZ,
    TEST_DRIVE_REQUEST_TOPIC,
)
from lidar_parking_planner.lidar_safety import LidarSafetyGate
from lidar_parking_planner.obstacle_tracker import (
    DYNAMIC_APPROACHING,
    DYNAMIC_RECEDING,
    ObstacleSafetyGate,
    SafetyConfig,
    STATIC,
    Track,
    UNKNOWN,
)
from lidar_parking_planner.parking_planner_node import ParkingPlanner
from std_msgs.msg import Float32


class Scan:
    def __init__(self, distance):
        self.angle_min = -math.radians(1.0)
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.10
        self.range_max = 10.0
        self.ranges = [math.inf, distance, math.inf]


def make_track(distance, classification):
    return Track(
        track_id='OBS_TEST', center_x=distance, center_y=0.0,
        center_range=distance, center_angle=0.0, min_range=distance,
        width=0.1, point_count=5, timestamp=0.0,
        front_min_range=distance, first_seen=0.0, last_seen=0.0,
        classification=classification,
    )


def apply_gates(requested_drive, point_gate, obstacle_gate):
    drive, _steering = point_gate.filter_command(requested_drive, 0.0)
    return obstacle_gate.filter_drive(drive)


def test_test_mode_off_keeps_nominal_planner_drive():
    selected = ParkingPlanner._select_requested_drive(
        0.2, False, 2.0, math.inf, 0.5)
    assert selected == 0.2


def test_test_mode_on_selects_fresh_20_hz_request():
    selected = ParkingPlanner._select_requested_drive(
        0.0, True, 2.0, 1.0 / DEFAULT_PUBLISH_RATE_HZ, 0.5)
    assert selected == 2.0
    assert DEFAULT_DRIVE_VALUE == 2.0
    assert DEFAULT_PUBLISH_RATE_HZ == 20.0


def test_test_request_timeout_fails_closed():
    selected = ParkingPlanner._select_requested_drive(
        0.2, True, 2.0, 0.51, 0.5)
    assert selected == 0.0


def test_clear_and_static_070_m_pass_request_unchanged():
    point_gate = LidarSafetyGate(stop_confirm_scans=1)
    obstacle_gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    point_gate.update_scan(Scan(0.70))
    obstacle_gate.update([make_track(0.70, STATIC)])
    assert apply_gates(2.0, point_gate, obstacle_gate) == 2.0


def test_static_049_m_stops_request():
    point_gate = LidarSafetyGate(stop_confirm_scans=1)
    obstacle_gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    point_gate.update_scan(Scan(0.49))
    obstacle_gate.update([make_track(0.49, STATIC)])
    assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0


def test_every_tracked_type_at_049_m_stops_request():
    for classification in (
            STATIC, DYNAMIC_APPROACHING, DYNAMIC_RECEDING, UNKNOWN):
        point_gate = LidarSafetyGate(stop_confirm_scans=1)
        obstacle_gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
        point_gate.update_scan(Scan(0.49))
        obstacle_gate.update([make_track(0.49, classification)])
        assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0


def test_lidar_timeout_stops_request():
    point_gate = LidarSafetyGate(stop_confirm_scans=1)
    obstacle_gate = ObstacleSafetyGate(SafetyConfig(stop_confirm_frames=1))
    point_gate.update_scan(Scan(0.70))
    obstacle_gate.update([])
    point_gate.update_timeout(0.51)
    obstacle_gate.update_timeout(0.51)
    assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0


def test_stop_holds_at_055_and_resumes_after_three_060_m_frames():
    point_gate = LidarSafetyGate(
        stop_confirm_scans=1, clear_confirm_scans=3)
    obstacle_gate = ObstacleSafetyGate(SafetyConfig(
        stop_confirm_frames=1, clear_confirm_frames=3))

    point_gate.update_scan(Scan(0.49))
    obstacle_gate.update([make_track(0.49, UNKNOWN)])
    assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0

    point_gate.update_scan(Scan(0.55))
    obstacle_gate.update([make_track(0.55, UNKNOWN)])
    assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0

    for _ in range(2):
        point_gate.update_scan(Scan(0.60))
        obstacle_gate.update([make_track(0.60, UNKNOWN)])
        assert apply_gates(2.0, point_gate, obstacle_gate) == 0.0
    point_gate.update_scan(Scan(0.60))
    obstacle_gate.update([make_track(0.60, UNKNOWN)])
    assert apply_gates(2.0, point_gate, obstacle_gate) == 2.0


def test_constant_node_only_publishes_float32_test_request():
    source = inspect.getsource(constant_drive_test_node)
    assert TEST_DRIVE_REQUEST_TOPIC == '/parking/test_drive_request'
    assert constant_drive_test_node.Float32 is Float32
    assert source.count('self.create_publisher(') == 1
    assert '/cmd_drive' not in source
    assert '/cmd_wheel' not in source


def test_parking_planner_remains_single_final_command_publisher():
    source = inspect.getsource(ParkingPlanner)
    assert source.count('self.cmd_drive_pub = self.create_publisher(') == 1
    assert source.count('self.cmd_drive_pub.publish(') == 1

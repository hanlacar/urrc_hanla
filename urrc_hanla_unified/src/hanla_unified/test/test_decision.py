from hanla_unified.decision import (
    Candidate, DecisionInput, decide, limit_rate, requires_emergency_brake,
)
from hanla_unified.mission_decision_node import MissionDecisionNode


VALID = Candidate(0.4, 3.0, True, 1.0)
INVALID = Candidate()


def data(section, camera=VALID, dr=VALID, lidar=VALID, **kwargs):
    return DecisionInput(section, camera, dr, lidar, **kwargs)


def test_section_priorities():
    assert decide(data(1)).source == "dr"
    assert decide(data(2)).source == "dr"
    assert decide(data(3)).source == "dr"
    assert decide(data(5)).source == "lidar"
    assert decide(data(7)).source == "lidar"
    assert decide(data(9)).source == "dr"
    assert decide(data(10)).source == "lidar"


def test_camera_never_owns_drive_or_steering():
    for section in (1, 2, 3, 4, 6, 8, 9, 11):
        result = decide(data(section, traffic_go=True))
        assert result.source == "dr"


def test_intersection_signal_does_not_stop_before_the_dr_stop_line():
    assert decide(data(4, traffic_go=False)).source == "dr"
    assert decide(data(4, traffic_go=True)).source == "dr"


def test_section11_uses_the_same_signal_gate():
    # The segmented follower owns the mode-11 STOP_LINE gate as well.
    assert decide(data(11, traffic_go=False)).source == "dr"
    assert decide(data(11, traffic_go=True)).source == "dr"


def test_acceleration_lidar_stop_has_highest_priority():
    result = decide(data(9, lidar_stop=True))
    assert result.stop and result.source == "lidar_stop"


def test_lidar_safety_stop_applies_in_every_section():
    assert decide(data(2, lidar_stop=True)).stop


def test_fail_closed_without_valid_source():
    result = decide(data(2, INVALID, INVALID, INVALID))
    assert result.stop and result.speed_mps == 0.0
    assert not requires_emergency_brake(result)


def test_only_explicit_lidar_safety_stop_uses_emergency_brake():
    assert requires_emergency_brake(decide(data(9, lidar_stop=True)))


def test_command_rate_limit_smooths_source_transition():
    assert limit_rate(12.0, -18.0, 45.0, 0.05) == 9.75
    assert limit_rate(-5.0, 5.0, 45.0, 1.0) == 5.0


def test_segmented_lidar_command_is_used_when_parking_executor_is_absent():
    node = object.__new__(MissionDecisionNode)
    node.section = 7
    node.values = {"lidar_drive": 2.0, "lidar_steer": -12.0}
    node.p = lambda name: {
        "stage_1_speed_mps": 0.229,
        "stage_2_speed_mps": 0.455,
        "stage_3_speed_mps": 0.70,
        "lidar_steering_sign": 1.0,
    }[name]
    node.fresh = lambda *keys: all(key in node.values for key in keys)

    candidate = MissionDecisionNode.candidate(node, "lidar")

    assert candidate.valid
    assert candidate.speed_mps == 0.455
    assert candidate.steering_deg == -12.0


def test_section5_lidar_is_valid_only_during_active_avoidance():
    node = object.__new__(MissionDecisionNode)
    node.section = 5
    node.values = {
        "lidar_drive": 1.0,
        "lidar_steer": 8.0,
        "avoidance_active": False,
    }
    node.p = lambda name: {
        "stage_1_speed_mps": 0.229,
        "stage_2_speed_mps": 0.455,
        "stage_3_speed_mps": 0.70,
        "lidar_steering_sign": 1.0,
    }[name]
    node.fresh = lambda *keys: all(key in node.values for key in keys)

    assert not MissionDecisionNode.candidate(node, "lidar").valid
    node.values["avoidance_active"] = True
    assert MissionDecisionNode.candidate(node, "lidar").valid


    assert not requires_emergency_brake(decide(data(4, traffic_go=False)))

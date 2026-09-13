"""t870_mcu 순수 로직 테스트 (ROS 불필요).

실행:
    cd <워크스페이스>/src/t870_mcu && python3 -m pytest test/ -v
"""

import math

import pytest

from t870_mcu.protocol import (
    drive_serial_command,
    parse_drive_stage,
    parse_status,
    valid_wheel_deg,
    wheel_serial_command,
)
from t870_mcu.arbitration import (
    InputManager,
    PrioritySelector,
    SafetyManager,
    WheelGate,
    CENTER_SOURCE,
    FAILSAFE,
)

SOURCES = ["lidar", "camera", "gps", "manual"]


# ============================================================
# 프로토콜
# ============================================================

def test_drive_protocol_mapping():
    assert drive_serial_command(-1) == "6.00"
    assert drive_serial_command(0) == "1.00"
    assert drive_serial_command(1) == "2.00"
    assert drive_serial_command(2) == "3.00"
    assert drive_serial_command(3) == "4.00"


def test_drive_rejects_non_stage_values():
    assert parse_drive_stage(1.0) == 1
    assert parse_drive_stage(1.2) is None
    assert parse_drive_stage(4.0) is None
    assert parse_drive_stage(math.nan) is None
    assert parse_drive_stage(math.inf) is None


def test_wheel_validation_no_clamp():
    assert valid_wheel_deg(-27) and valid_wheel_deg(27)
    assert not valid_wheel_deg(-28) and not valid_wheel_deg(28)


def test_wheel_w_protocol():
    assert wheel_serial_command(0, 27, 440, "W") == "W0"
    assert wheel_serial_command(27, 27, 440, "W") == "W440"
    assert wheel_serial_command(-27, 27, 440, "W") == "W-440"


# ---- STATUS 회귀 테스트 -------------------------------------
# v28 은 fault 필드에 숫자가 아니라 "NONE" 문자열을 넣는다.
# 이전 파서는 여기서 예외가 나 라인 전체를 버렸고, 그 결과
# /drive, /rpm, /arduino/raw_status, /odom 이 전혀 발행되지 않았다.

REAL_LINES = [
    "STATUS,READY,NONE,251,363,0,0.00,0,0,0,440",
    "STATUS,ACTIVE,NONE,296,363,51,13.52,1624,163,163,440",
]


def test_real_vehicle_status_with_text_fault_is_parsed():
    for line in REAL_LINES:
        s = parse_status(line)
        assert s is not None, "실차 STATUS 파싱 실패: %s" % line
        assert s["fault"] == 0 and s["fault_text"] == "NONE"


def test_real_vehicle_status_fields():
    s = parse_status("STATUS,ACTIVE,NONE,296,363,51,13.52,1624,163,163,440")
    assert s["state"] == "ACTIVE"
    assert s["adc"] == 296 and s["rpm"] == 13.52
    assert s["encoder_count"] == 1624 and s["steer_ms"] == 163


def test_text_and_numeric_fault_both_detected():
    assert parse_status("STATUS,FAULT,POT_RANGE,251,363,0,0,0,0,0,440")["fault"] == 1
    assert parse_status("STATUS,READY,0,1,2,3,4.0,5,6,7,8")["fault"] == 0
    assert parse_status("STATUS,FAULT,3,1,2,3,4.0,5,6,7,8")["fault"] == 1


def test_broken_field_does_not_discard_whole_line():
    s = parse_status("STATUS,READY,NONE,xx,363,0,BAD,555,0,0,440")
    assert s is not None and s["encoder_count"] == 555
    assert s["adc"] == 0 and s["rpm"] == 0.0


def test_malformed_lines_return_none():
    for bad in ("STATUS,READY", "garbage line", "", None):
        assert parse_status(bad) is None


# ============================================================
# 구동 — 고정 우선순위
# ============================================================

@pytest.fixture
def inputs():
    return InputManager(SOURCES)


@pytest.fixture
def prio():
    return PrioritySelector(["lidar", "camera", "gps"], SOURCES)


def test_lidar_beats_camera(inputs, prio):
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    inputs.update_drive("lidar", 1.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("lidar", 1.0)


def test_camera_wins_when_lidar_silent(inputs, prio):
    """라이다는 평소 조용하다. 그때는 카메라가 운전한다."""
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("camera", 2.0)


def test_falls_through_to_gps(inputs, prio):
    now = 100.0
    inputs.update_drive("gps", 1.0, now, True, "ok")
    src, val, _ = prio.select(inputs, now, 0.5)
    assert (src, val) == ("gps", 1.0)


def test_no_source_means_stop(inputs, prio):
    src, val, tried = prio.select(inputs, 100.0, 0.5)
    assert src is None and val == 0.0
    assert len(tried) == 3


def test_stale_high_priority_yields_to_fresh_lower(inputs, prio):
    """라이다가 끊기면 카메라가 이어받는다."""
    inputs.update_drive("lidar", 1.0, 100.0, True, "ok")
    inputs.update_drive("camera", 2.0, 100.9, True, "ok")
    src, val, _ = prio.select(inputs, 100.9, 0.5)
    assert (src, val) == ("camera", 2.0)


def test_invalid_value_disqualifies_source(inputs, prio):
    """허용 안 된 값을 준 소스는 이기지 못한다."""
    now = 100.0
    inputs.update_drive("lidar", 1.5, now, False, "not_allowed")
    inputs.update_drive("camera", 2.0, now, True, "ok")
    src, _, _ = prio.select(inputs, now, 0.5)
    assert src == "camera"


def test_priority_rejects_unknown_source():
    with pytest.raises(ValueError):
        PrioritySelector(["radar"], SOURCES)


# ============================================================
# 조향 — 모드 게이트
# ============================================================

@pytest.fixture
def gate():
    return WheelGate("camera", ["T_PARK:lidar", "PARALLEL_PARK:lidar"], SOURCES)


def test_wheel_owner_by_mode(gate):
    assert gate.owner("NORMAL") == "camera"
    assert gate.owner("t_park") == "lidar"
    assert gate.owner("PARALLEL_PARK") == "lidar"
    assert gate.owner("ANYTHING_ELSE") == "camera"


def test_lidar_wheel_ignored_outside_parking(inputs, gate):
    """★ 일반 주행에서는 라이다 조향이 무시되고 카메라가 쓰인다."""
    now = 100.0
    inputs.update_wheel("lidar", 20, now, True, "ok")
    inputs.update_wheel("camera", -5, now, True, "ok")
    val, used, ok, _ = gate.resolve("NORMAL", inputs, now, 0.5, 0)
    assert (val, used, ok) == (-5, "camera", True)


def test_lidar_wheel_used_in_parking(inputs, gate):
    now = 100.0
    inputs.update_wheel("lidar", 20, now, True, "ok")
    inputs.update_wheel("camera", -5, now, True, "ok")
    val, used, ok, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (val, used, ok) == (20, "lidar", True)


def test_wheel_absent_falls_back_to_center(inputs, gate):
    """★ 조향이 없어도 구동은 별개다. 조향만 0도로 폴백."""
    now = 100.0
    val, used, ok, reason = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (val, used, ok) == (0, FAILSAFE, False)
    assert "never_received" in reason


def test_wheel_timeout_falls_back(inputs, gate):
    inputs.update_wheel("camera", 10, 100.0, True, "ok")
    val, used, ok, reason = gate.resolve("NORMAL", inputs, 100.9, 0.5, 0)
    assert (val, used, ok) == (0, FAILSAFE, False)
    assert "timeout" in reason


def test_wheel_hold_last_failsafe(inputs, gate):
    val, used, ok, _ = gate.resolve("NORMAL", inputs, 100.0, 0.5, 15)
    assert val == 15 and used == FAILSAFE and not ok


def test_center_keyword_is_by_design(inputs):
    g = WheelGate("center", [], SOURCES)
    val, used, ok, reason = g.resolve("NORMAL", inputs, 100.0, 0.5, 0)
    assert (val, used, ok) == (0, CENTER_SOURCE, True)
    assert reason == "no_wheel_source_by_design"


# ============================================================
# 급정거
# ============================================================

def test_stop_is_detected(inputs):
    inputs.update_stop("lidar", True, 100.0)
    assert inputs.stop_asserted("lidar", 100.0, 0.5)
    assert inputs.any_stop(["lidar", "camera"], 100.0, 0.5) == ["lidar"]


def test_stop_releases_when_false(inputs):
    inputs.update_stop("lidar", True, 100.0)
    inputs.update_stop("lidar", False, 100.1)
    assert not inputs.stop_asserted("lidar", 100.1, 0.5)


def test_stale_stop_is_released(inputs):
    """★ 오래된 true 를 계속 믿으면 장애물이 치워져도 영영 못 움직인다."""
    inputs.update_stop("lidar", True, 100.0)
    assert not inputs.stop_asserted("lidar", 100.9, 0.5)


def test_any_stop_lists_all_asserting_sources(inputs):
    inputs.update_stop("lidar", True, 100.0)
    inputs.update_stop("camera", True, 100.0)
    assert set(inputs.any_stop(["lidar", "camera"], 100.0, 0.5)) == {"lidar", "camera"}


# ============================================================
# 값 검증
# ============================================================

@pytest.fixture
def safety():
    return SafetyManager("allowed_values", [-1.0, 0.0, 1.0, 2.0, 3.0],
                         -1.0, 3.0, -27, 27)


def test_drive_allowed_values(safety):
    assert safety.validate_drive(2.0) == (True, "ok")
    assert not safety.validate_drive(1.5)[0]
    assert not safety.validate_drive(4.0)[0]


def test_non_finite_drive_is_rejected(safety):
    assert safety.validate_drive(math.nan) == (False, "non_finite")
    assert safety.validate_drive(math.inf) == (False, "non_finite")


def test_wheel_range_is_rejected_not_clamped(safety):
    assert safety.validate_wheel(27) == (True, "ok")
    assert safety.validate_wheel(28) == (False, "out_of_range")
    assert safety.validate_wheel(-28) == (False, "out_of_range")


# ============================================================
# 통합 시나리오 — 실제 미션 상황
# ============================================================

def test_scenario_s_course(inputs, prio, gate):
    """S코스: 카메라가 회피 주행, 라이다는 감시만."""
    now = 100.0
    inputs.update_drive("camera", 1.0, now, True, "ok")
    inputs.update_wheel("camera", 15, now, True, "ok")
    inputs.update_wheel("lidar", -20, now, True, "ok")   # 무시돼야 함

    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, _, _ = gate.resolve("NORMAL", inputs, now, 0.5, 0)
    assert (src, dval) == ("camera", 1.0)
    assert (wval, wused) == (15, "camera")
    assert not inputs.any_stop(["lidar", "camera"], now, 0.5)


def test_scenario_sudden_obstacle(inputs):
    """돌발: 라이다 급정거가 모든 것을 이긴다."""
    now = 100.0
    inputs.update_drive("camera", 3.0, now, True, "ok")
    inputs.update_stop("lidar", True, now)
    assert inputs.any_stop(["lidar", "camera"], now, 0.5) == ["lidar"]


def test_scenario_t_parking(inputs, prio, gate):
    """T자 주차: 라이다가 구동과 조향을 모두 가져간다."""
    now = 100.0
    inputs.update_drive("camera", 2.0, now, True, "ok")
    inputs.update_drive("lidar", -1.0, now, True, "ok")
    inputs.update_wheel("lidar", 27, now, True, "ok")

    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, _, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (src, dval) == ("lidar", -1.0)
    assert (wval, wused) == (27, "lidar")


def test_scenario_lidar_drive_without_wheel(inputs, prio, gate):
    """★ 라이다가 구동만 주고 조향을 안 줘도 주행은 계속된다."""
    now = 100.0
    inputs.update_drive("lidar", 2.0, now, True, "ok")
    src, dval, _ = prio.select(inputs, now, 0.5)
    wval, wused, wok, _ = gate.resolve("T_PARK", inputs, now, 0.5, 0)
    assert (src, dval) == ("lidar", 2.0)      # 구동 살아있음
    assert (wval, wok) == (0, False)          # 조향만 폴백


# ==========================================================
# 구독 진단 (0829) — 타입·QoS 불일치를 잡아내는가
#
#  이 두 가지는 ROS 가 에러를 안 낸다. 메시지만 조용히 안 온다.
#  "저쪽은 쏘는데 나는 못 받는" 상황의 대부분이 이것이라 회귀 시험을 둔다.
# ==========================================================
from t870_mcu.diagnostics import check_subscriptions


class _Rel:
    def __init__(self, name):
        self.name = name


class _Qos:
    def __init__(self, name):
        self.reliability = _Rel(name)


class _Info:
    def __init__(self, node, typ, rel="RELIABLE"):
        self.node_namespace = "/"
        self.node_name = node
        self.topic_type = typ
        self.qos_profile = _Qos(rel)


class _Log:
    def __init__(self):
        self.msgs = []

    def error(self, m):
        self.msgs.append(m)


class _Node:
    def __init__(self, table):
        self.table = table
        self._log = _Log()

    def get_logger(self):
        return self._log

    def get_publishers_info_by_topic(self, topic):
        return self.table.get(topic, [])


def test_진단_정상이면_조용하다():
    n = _Node({"/a": [_Info("x", "std_msgs/msg/Float32")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Float32", "구동")], set())
    assert n._log.msgs == []


def test_진단_발행자없으면_조용하다():
    # 팀 노드가 늦게 뜨는 것은 정상이다. 이걸 에러로 찍으면 소음이 된다.
    n = _Node({})
    check_subscriptions(n, [("/a", "std_msgs/msg/Float32", "구동")], set())
    assert n._log.msgs == []


def test_진단_타입불일치를_잡는다():
    n = _Node({"/a": [_Info("lidar", "std_msgs/msg/Float32")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Int32", "조향")], set())
    assert len(n._log.msgs) == 1
    assert "타입이 다르다" in n._log.msgs[0]


def test_진단_QoS불일치를_잡는다():
    n = _Node({"/a": [_Info("cam", "std_msgs/msg/Bool", "BEST_EFFORT")]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Bool", "급정거")], set())
    assert len(n._log.msgs) == 1
    assert "QoS" in n._log.msgs[0]


def test_진단_같은문제를_두번찍지않는다():
    n = _Node({"/a": [_Info("cam", "std_msgs/msg/Bool", "BEST_EFFORT")]})
    seen = set()
    spec = [("/a", "std_msgs/msg/Bool", "급정거")]
    check_subscriptions(n, spec, seen)
    check_subscriptions(n, spec, seen)
    check_subscriptions(n, spec, seen)
    assert len(n._log.msgs) == 1


def test_진단_reliability가_정수여도_동작한다():
    # rclpy 구현에 따라 enum 이 아니라 정수로 오는 경우가 있다
    class _IntQos:
        reliability = 2          # 2 = BEST_EFFORT

    info = _Info("cam", "std_msgs/msg/Bool")
    info.qos_profile = _IntQos()
    n = _Node({"/a": [info]})
    check_subscriptions(n, [("/a", "std_msgs/msg/Bool", "급정거")], set())
    assert len(n._log.msgs) == 1

"""명령 중재 순수 로직 (ROS 의존 없음).

v3 구조
-------
구동(drive) = 고정 우선순위      라이다 > 카메라 > GPS
조향(wheel) = 모드 게이트        주차 모드에서만 라이다, 그 외 카메라
급정거(stop) = 별도 채널          모드·우선순위 무시, 즉시 정지

구동과 조향은 완전히 독립적으로 결정된다.
조향 소스가 값을 안 줘도 구동은 절대 막히지 않는다.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


CENTER_SOURCE = "center"     # 조향 0도 고정
NO_SOURCE = "none"           # 소스 없음
FAILSAFE = "failsafe"        # 권한 소스가 값을 안 줌


# ============================================================
# 상태 컨테이너
# ============================================================

@dataclass
class ChannelState:
    """한 채널(구동/조향/정지)의 최신값과 유효성."""

    value: float = 0.0
    received_at: Optional[float] = None
    valid: bool = False
    reason: str = "never_received"

    def update(self, value: float, now: float, valid: bool, reason: str = "ok") -> None:
        self.value = value
        self.received_at = now
        self.valid = valid
        self.reason = reason

    def is_fresh(self, now: float, timeout_s: float) -> bool:
        if self.received_at is None:
            return False
        return (now - self.received_at) <= timeout_s


@dataclass
class SourceState:
    """소스 하나의 구동/조향/정지 상태. 서로 독립."""

    drive: ChannelState = field(default_factory=ChannelState)
    wheel: ChannelState = field(default_factory=ChannelState)
    stop: ChannelState = field(default_factory=ChannelState)


@dataclass
class ArbitrationStatus:
    mode: str = "IDLE"
    drive_source: str = "stop"
    wheel_source: str = "stop"
    safety_state: str = "IDLE"
    ready: bool = False


# ============================================================
# 입력 관리
# ============================================================

class InputManager:
    """소스별 최신 구동/조향/정지 값을 보관한다."""

    def __init__(self, source_names: Iterable[str]):
        names = [str(n).strip() for n in source_names if str(n).strip()]
        if not names:
            raise ValueError("at least one command source is required")
        if len(set(names)) != len(names):
            raise ValueError("duplicate command source name")
        self.states: Dict[str, SourceState] = {n: SourceState() for n in names}

    # ---- 갱신 ----

    def update_drive(self, source, value, now, valid, reason) -> None:
        self.states[source].drive.update(float(value), now, valid, reason)

    def update_wheel(self, source, value, now, valid, reason) -> None:
        self.states[source].wheel.update(float(value), now, valid, reason)

    def update_stop(self, source, value, now) -> None:
        self.states[source].stop.update(1.0 if value else 0.0, now, True, "ok")

    # ---- 조회 ----

    def drive_status(self, source, now, timeout_s) -> Tuple[bool, str, float]:
        st = self.states[source].drive
        if not st.valid:
            return False, st.reason, st.value
        if not st.is_fresh(now, timeout_s):
            return False, "timeout", st.value
        return True, "ok", st.value

    def wheel_status(self, source, now, timeout_s) -> Tuple[bool, str, int]:
        st = self.states[source].wheel
        if not st.valid:
            return False, st.reason, int(round(st.value))
        if not st.is_fresh(now, timeout_s):
            return False, "timeout", int(round(st.value))
        return True, "ok", int(round(st.value))

    def stop_asserted(self, source, now, timeout_s) -> bool:
        """정지 요청이 살아있는가.

        '최근에 true 로 온 것'만 유효하다. 오래된 true 를 계속 믿으면
        장애물이 치워져도 차가 영영 못 움직인다.
        """
        st = self.states[source].stop
        if st.value < 0.5:
            return False
        return st.is_fresh(now, timeout_s)

    def any_stop(self, sources, now, timeout_s) -> List[str]:
        """정지를 요청 중인 소스 목록."""
        return [s for s in sources if self.stop_asserted(s, now, timeout_s)]


# ============================================================
# 구동 — 고정 우선순위
# ============================================================

class PrioritySelector:
    """우선순위 목록에서 '살아있는 것 중 가장 높은' 소스를 고른다.

    priority = ["lidar", "camera", "gps"]
      라이다가 값을 주면 라이다가 이김.
      라이다가 조용하면 카메라, 카메라도 없으면 GPS.
      전부 없으면 (None, 0.0, 사유목록).
    """

    def __init__(self, priority: Iterable[str], known_sources: Iterable[str]):
        self.priority = [str(p).strip().lower() for p in priority if str(p).strip()]
        known = set(known_sources)
        unknown = [p for p in self.priority if p not in known]
        if unknown:
            raise ValueError("drive_priority 에 알 수 없는 소스: %s" % unknown)
        if not self.priority:
            raise ValueError("drive_priority 가 비어 있음")

    def select(self, inputs: "InputManager", now: float,
               timeout_s: float) -> Tuple[Optional[str], float, List[str]]:
        tried = []
        for src in self.priority:
            ok, reason, value = inputs.drive_status(src, now, timeout_s)
            if ok:
                return src, value, tried
            tried.append("%s:%s" % (src, reason))
        return None, 0.0, tried


# ============================================================
# 조향 — 모드 게이트
# ============================================================

class WheelGate:
    """모드에 따라 조향 권한을 가진 소스를 정한다.

    default_owner = "camera"
    overrides = ["T_PARK:lidar", "PARALLEL_PARK:lidar"]

    → 주차 모드에서만 라이다가 조향, 그 외에는 카메라.
      권한 없는 소스가 조향을 발행해도 무시된다.
    """

    def __init__(self, default_owner: str, override_entries: Iterable[str],
                 known_sources: Iterable[str], known_modes: Iterable[str] = ()):
        known = set(known_sources) | {CENTER_SOURCE, NO_SOURCE}
        self.default_owner = str(default_owner).strip().lower()
        if self.default_owner not in known:
            raise ValueError("wheel_owner_default 가 알 수 없는 소스: %s"
                             % self.default_owner)

        self.overrides: Dict[str, str] = {}
        for entry in override_entries:
            text = str(entry).strip()
            if not text:
                continue
            if ":" not in text:
                raise ValueError("wheel_owner_overrides 형식은 MODE:SOURCE : %s" % text)
            mode, owner = text.split(":", 1)
            owner = owner.strip().lower()
            if owner not in known:
                raise ValueError("wheel_owner_overrides 에 알 수 없는 소스: %s" % owner)
            self.overrides[mode.strip().upper()] = owner

        # ---- 모드 문자열 화이트리스트 ----
        #
        # ★ 이게 없으면 오타가 조용히 넘어간다.
        #   "T_PARK" 를 "TPARK" 로 발행하면 override 에 안 걸려 기본 소유자
        #   (카메라)가 조향을 가져간다. 에러도 경고도 없다. 주차 구간에서
        #   이게 나면 주차 자체가 실패하는데 원인을 찾기가 매우 어렵다.
        #
        # known_modes 가 비어 있으면 검증하지 않는다 (기존 동작).
        self.known_modes = set(
            str(m).strip().upper() for m in known_modes if str(m).strip())

        unknown_override = [m for m in self.overrides
                            if self.known_modes and m not in self.known_modes]
        if unknown_override:
            raise ValueError(
                "wheel_owner_overrides 의 모드가 known_modes 에 없다: %s"
                % sorted(unknown_override))

    def is_known(self, mode: str) -> bool:
        """known_modes 에 있는 모드인가. 목록이 비어 있으면 항상 True."""
        if not self.known_modes:
            return True
        return str(mode).strip().upper() in self.known_modes

    def owner(self, mode: str) -> str:
        return self.overrides.get(str(mode).strip().upper(), self.default_owner)

    def resolve(self, mode: str, inputs: "InputManager", now: float,
                timeout_s: float, failsafe_value: int) -> Tuple[int, str, bool, str]:
        """(조향각, 사용된소스, 권한소스정상여부, 사유) 반환.

        권한 소스가 값을 안 주면 failsafe 값으로 폴백하되 '폴백 중'임을
        별도로 알린다. 구동은 이와 무관하게 계속된다.
        """
        own = self.owner(mode)
        if own in (CENTER_SOURCE, NO_SOURCE):
            return 0, CENTER_SOURCE, True, "no_wheel_source_by_design"

        ok, reason, value = inputs.wheel_status(own, now, timeout_s)
        if ok:
            return int(value), own, True, "ok"
        return failsafe_value, FAILSAFE, False, "%s:%s" % (own, reason)


# ============================================================
# 값 검증
# ============================================================

class SafetyManager:
    """명령값 검증. 범위를 벗어나면 클램프하지 않고 거부한다."""

    def __init__(self, drive_validation_mode, drive_allowed_values,
                 drive_min, drive_max, wheel_min, wheel_max,
                 drive_value_tolerance=1e-6):
        mode = str(drive_validation_mode).strip().lower()
        if mode not in ("allowed_values", "range"):
            raise ValueError("drive_validation_mode must be allowed_values or range")
        if drive_min > drive_max:
            raise ValueError("drive_min must be <= drive_max")
        if wheel_min > wheel_max:
            raise ValueError("wheel_min must be <= wheel_max")

        self.drive_validation_mode = mode
        self.drive_allowed_values = [float(v) for v in drive_allowed_values]
        self.drive_min = float(drive_min)
        self.drive_max = float(drive_max)
        self.wheel_min = int(wheel_min)
        self.wheel_max = int(wheel_max)
        self.drive_value_tolerance = float(drive_value_tolerance)

        if mode == "allowed_values" and not self.drive_allowed_values:
            raise ValueError("drive_allowed_values cannot be empty")

    def validate_drive(self, value) -> Tuple[bool, str]:
        value = float(value)
        if not math.isfinite(value):
            return False, "non_finite"
        if self.drive_validation_mode == "range":
            if value < self.drive_min or value > self.drive_max:
                return False, "out_of_range"
            return True, "ok"
        for allowed in self.drive_allowed_values:
            if abs(value - allowed) <= self.drive_value_tolerance:
                return True, "ok"
        return False, "not_allowed"

    def validate_wheel(self, value) -> Tuple[bool, str]:
        if isinstance(value, bool):
            return False, "invalid_type"
        try:
            numeric = int(value)
        except (TypeError, ValueError, OverflowError):
            return False, "invalid_type"
        if numeric < self.wheel_min or numeric > self.wheel_max:
            return False, "out_of_range"
        return True, "ok"

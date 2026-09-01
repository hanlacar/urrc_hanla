"""T870 Arduino 펌웨어 v28 프로토콜 헬퍼 (순수 함수).

v2 수정 사항
------------
★ parse_status 의 fault 필드 파싱 버그 수정.
  v28 STATUS 의 fault 필드는 숫자가 아니라 "NONE" 같은 문자열이다.
      STATUS,READY,NONE,251,363,0,0.00,0,0,0,440
                   ^^^^
  기존 코드는 int(float("NONE")) 에서 ValueError 가 나 None 을 반환했고,
  그 결과 모든 STATUS 라인이 통째로 버려졌다.
  → /drive, /rpm, /arduino/raw_status, /odom 이 전혀 발행되지 않고
    펌웨어 fault 감지(E-stop 래치)도 동작하지 않았다.
  송신 경로는 독립이라 주행은 되므로 증상이 잘 드러나지 않는 유형의 버그.
"""

import math

DRIVE_MAP = {
    -1: "6.00",
    0: "1.00",
    1: "2.00",
    2: "3.00",
    3: "4.00",
}

VALID_DRIVE_VALUES = frozenset(DRIVE_MAP)

# fault 필드가 이 값이면 정상으로 본다 (대문자 비교).
# v28 은 정상일 때 "NONE" 을 출력한다.
DEFAULT_FAULT_OK_VALUES = ("NONE", "0", "", "OK")

# STATUS 필드 순서 (v28 고정. 팀 파서 호환을 위해 변경 금지)
#   0:STATUS 1:state 2:fault 3:adc 4:target_adc 5:drive_pwm
#   6:rpm 7:encoder_count 8:steer_net_ms 9:steer_target_ms 10:steer_limit_ms
STATUS_MIN_FIELDS = 8


def parse_drive_stage(value: float):
    """허용된 정확한 단계값만 반환. 아니면 None.

    1.0 같은 부동소수는 허용하되 1.2 를 1 로 반올림하지 않는다.
    NaN, Inf 는 거부.
    """
    if not math.isfinite(value):
        return None
    rounded = int(round(value))
    if abs(value - rounded) > 1e-6:
        return None
    return rounded if rounded in VALID_DRIVE_VALUES else None


def drive_serial_command(stage: int) -> str:
    return DRIVE_MAP[stage]


def valid_wheel_deg(value, max_deg=27) -> bool:
    if isinstance(value, bool):
        return False
    try:
        numeric = int(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return -int(max_deg) <= numeric <= int(max_deg)


def wheel_serial_command(deg: int, max_deg: float, steer_limit_ms: int, mode: str) -> str:
    ms_per_deg = float(steer_limit_ms) / float(max_deg)
    if str(mode).upper() == "V":
        return "V,%.3f" % (deg / float(max_deg))
    ms = int(round(deg * ms_per_deg))
    return "W%d" % ms


def _to_int(text, default=0):
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return default


def _to_int_or_none(text):
    """읽을 수 없으면 0 이 아니라 None 을 준다.

    🔴 0829: 누적 카운터에 기본값 0 을 주면 안 된다.
      제동 순간(역토크 펄스)에 시리얼 노이즈로 필드가 깨지면
      엔코더 누적값이 조용히 0 으로 발행되어 "멈추면 초기화" 로 보인다.
      실제로 그 증상이 났다. 못 읽었으면 못 읽었다고 말해야 한다.
    """
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return None


def _to_float(text, default=0.0):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def encoder_sanity(value, last, dt, max_cps):
    """엔코더 누적값을 믿어도 되는지. (믿어도_되나, 이유) 를 준다.

    ROS 없이 시험할 수 있도록 순수 함수로 둔다.

    value    이번 STATUS 에서 읽은 누적 카운트. 못 읽었으면 None
    last     마지막으로 믿었던 값. 처음이면 None
    dt       마지막 값 이후 흐른 시간 [s]
    max_cps  초당 허용 변화량. 0 이하면 점프 검사를 하지 않는다

    🔴 왜 필요한가 (0829)
      제동은 역토크 펄스라 전기 노이즈가 가장 큰 순간이다. 그때 시리얼이
      깨지면 필드가 밀리거나 문자가 섞인다. 예전에는 그걸 못 읽으면
      그냥 0 을 발행했고, 누적 카운터에 0 이 튀니 구독자 쪽에서는
      "달리다가 멈추면 엔코더가 초기화된다" 로 보였다.
    """
    if value is None:
        return False, "필드를 읽을 수 없다"

    if last is None or max_cps <= 0.0:
        return True, ""

    if dt <= 0.0:
        dt = 1e-3
    #  전진↔후진 전환에서 부호가 뒤집히므로 여유(50)를 더 준다
    limit = max_cps * dt + 50.0
    moved = abs(value - last)
    if moved > limit:
        return False, ("%d → %d 로 %.2f초 만에 %d 카운트 튀었다 (한계 %.0f)"
                       % (last, value, dt, moved, limit))
    return True, ""


def parse_status(line: str, fault_ok_values=DEFAULT_FAULT_OK_VALUES):
    """STATUS 라인을 파싱한다.

    반환 dict:
        raw            원문
        encoder_count  int 또는 **None** (필드가 깨져 못 읽은 경우)
        state          상태머신 문자열 (READY/ACTIVE/FAULT/ESTOP/...)
        fault_text     fault 필드 원문 ("NONE" 등)
        fault          0 = 정상, 1 = fault (fault_ok_values 에 없으면 1)
        adc, rpm, encoder_count, steer_ms

    ★ fault 필드는 문자열일 수도 숫자일 수도 있다. 둘 다 처리한다.
      숫자면 그 값이 0 인지로 판정하고, 문자열이면 fault_ok_values 로 판정한다.
      개별 필드가 깨져도 라인 전체를 버리지 않는다 (텔레메트리 전멸 방지).
    """
    if not isinstance(line, str) or not line.startswith("STATUS,"):
        return None
    fields = line.split(",")
    if len(fields) < STATUS_MIN_FIELDS:
        return None

    fault_raw = fields[2].strip()
    fault_upper = fault_raw.upper()

    # 숫자로 읽히면 숫자 기준, 아니면 화이트리스트 기준
    try:
        fault_code = 0 if float(fault_raw) == 0.0 else 1
    except (TypeError, ValueError):
        ok = tuple(str(v).upper() for v in fault_ok_values)
        fault_code = 0 if fault_upper in ok else 1

    result = {
        "raw": line,
        "state": fields[1].strip(),
        "fault_text": fault_raw,
        "fault": fault_code,
        "adc": _to_int(fields[3]),
        "rpm": _to_float(fields[6]),
        #  ★ 못 읽으면 None. 호출한 쪽이 "직전 값 유지" 를 하도록 한다.
        "encoder_count": _to_int_or_none(fields[7]),
        "steer_ms": _to_int(fields[8]) if len(fields) > 8 else 0,
        "steer_target_ms": _to_int(fields[9]) if len(fields) > 9 else 0,
        "steer_limit_ms": _to_int(fields[10]) if len(fields) > 10 else 0,
    }
    return result

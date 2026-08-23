import math
import re


LEGACY_STATUS_PATTERN = re.compile(
    r"\[상태\].*?A0=(-?\d+).*?편차=(-?\d+).*?누적=(-?\d+)(?:/-?\d+)?"
    r".*?RPM=([-+]?\d+(?:\.\d+)?)"
    r".*?오도=(-?\d+)"
)


def parse_v27_status(line):
    """Parse v27 STATUS telemetry and its raw/center steering ADC values."""
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    fields = str(line).strip().split(",")
    if len(fields) != 11 or fields[0] != "STATUS":
        return None
    try:
        a0 = int(fields[3])
        center_a0 = int(fields[4])
        rpm = float(fields[6])
        encoder_count = int(fields[7])
        steering_position_ms = float(fields[8])
    except ValueError:
        return None
    if not math.isfinite(rpm) or not math.isfinite(steering_position_ms):
        return None
    return encoder_count, rpm, steering_position_ms, a0, center_a0


def parse_telemetry(line):
    """Parse T,<encoder_count>,<rpm>,<steering_position_ms>."""
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    fields = str(line).strip().split(",")
    if len(fields) != 4 or fields[0] != "T":
        return None
    try:
        encoder_count = int(fields[1])
        rpm = float(fields[2])
        steering_angle = float(fields[3])
    except ValueError:
        return None
    if not math.isfinite(rpm) or not math.isfinite(steering_angle):
        return None
    if not -(2**63) <= encoder_count < 2**63:
        return None
    return encoder_count, rpm, steering_angle


def parse_legacy_status(line):
    """Parse v14's on-demand Korean S response into the T-frame contract.

    Example: [상태] A0=359 편차=-4 누적=0/440 RPM=0.0 오도=0 PWM=0
    Returns (odometry_count, rpm, accumulated_steering_ms).
    """
    v27 = parse_v27_status(line)
    if v27 is not None:
        return v27[:3]
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    match = LEGACY_STATUS_PATTERN.search(str(line).strip())
    if match is None:
        return None
    steering_position_ms = float(match.group(3))
    rpm = float(match.group(4))
    encoder_count = int(match.group(5))
    if not math.isfinite(rpm) or not math.isfinite(steering_position_ms):
        return None
    return encoder_count, rpm, steering_position_ms


def parse_legacy_steering_a0(line):
    """Return the raw A0 steering sensor reading from a Korean status line."""
    v27 = parse_v27_status(line)
    if v27 is not None:
        return v27[3]
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("utf-8")
        except UnicodeDecodeError:
            return None
    match = LEGACY_STATUS_PATTERN.search(str(line).strip())
    return None if match is None else int(match.group(1))


def steering_position_to_degrees(
    steering_position_ms, maximum_position_ms, maximum_steering_deg
):
    """Convert v14 accumulated steering time to a clamped angle estimate."""
    steering_position_ms = float(steering_position_ms)
    maximum_position_ms = float(maximum_position_ms)
    maximum_steering_deg = float(maximum_steering_deg)
    if not all(
        math.isfinite(value)
        for value in (
            steering_position_ms,
            maximum_position_ms,
            maximum_steering_deg,
        )
    ):
        raise ValueError("steering conversion values must be finite")
    if maximum_position_ms <= 0.0 or maximum_steering_deg <= 0.0:
        raise ValueError("steering conversion maxima must be positive")
    estimated = steering_position_ms / maximum_position_ms * maximum_steering_deg
    return max(-maximum_steering_deg, min(maximum_steering_deg, estimated))


def encoder_delta_to_distance_m(delta_count, counts_per_meter):
    """Convert a signed encoder-count change to signed travel distance."""
    counts_per_meter = float(counts_per_meter)
    if not math.isfinite(counts_per_meter) or counts_per_meter <= 0.0:
        raise ValueError("counts_per_meter must be positive and finite")
    return float(delta_count) / counts_per_meter


def encoder_delta_to_speed_mps(delta_count, delta_time_sec, counts_per_meter):
    """Convert a signed count change over time to signed linear speed."""
    delta_time_sec = float(delta_time_sec)
    if not math.isfinite(delta_time_sec) or delta_time_sec <= 0.0:
        raise ValueError("delta_time_sec must be positive and finite")
    return encoder_delta_to_distance_m(delta_count, counts_per_meter) / delta_time_sec


def meters_per_second_to_kilometers_per_hour(speed_mps):
    """Convert a signed speed in m/s to km/h."""
    speed_mps = float(speed_mps)
    if not math.isfinite(speed_mps):
        raise ValueError("speed_mps must be finite")
    return speed_mps * 3.6


DRIVE_STAGE_TO_FIRMWARE = {
    0: 1,
    1: 2,
    2: 3,
    3: 4,
    -1: 6,
    -2: 7,
    -3: 8,
}


def encode_drive_command(stage, maximum_abs_stage):
    """Map signed ROS stages (-3..3) to the existing firmware command."""
    if isinstance(stage, bool) or not isinstance(stage, int):
        raise ValueError("stage must be an integer")
    maximum_abs_stage = int(maximum_abs_stage)
    if not 1 <= maximum_abs_stage <= 3:
        raise ValueError("maximum_abs_stage must be between 1 and 3")
    if abs(stage) > maximum_abs_stage:
        raise ValueError("stage exceeds configured limit")
    return f"{DRIVE_STAGE_TO_FIRMWARE[stage]:.2f}\n".encode("ascii")


def encode_steering_command(steering_deg, maximum_steering_deg):
    """Map a Lookahead steering target to the firmware steering protocol.

    An exact zero target uses the firmware's logical-center command.  This
    returns the accumulated steering drive position to zero without consulting
    the unreliable A0 neutral reference.  Nonzero targets remain proportional
    V commands.
    """
    steering_deg = float(steering_deg)
    maximum_steering_deg = float(maximum_steering_deg)
    if not math.isfinite(steering_deg):
        raise ValueError("steering_deg must be finite")
    if not math.isfinite(maximum_steering_deg) or maximum_steering_deg <= 0.0:
        raise ValueError("maximum_steering_deg must be positive and finite")
    if steering_deg == 0.0:
        return b"C\n"
    ratio = max(-1.0, min(1.0, steering_deg / maximum_steering_deg))
    return f"V,{ratio:.3f}\n".encode("ascii")


def encode_commands(stage, steering_deg, maximum_abs_stage, maximum_steering_deg):
    """Encode one drive line and one proportional-steering line."""
    return encode_drive_command(stage, maximum_abs_stage) + encode_steering_command(
        steering_deg, maximum_steering_deg
    )

"""Pure protocol helpers for the T870 Arduino firmware v28."""

import math

DRIVE_MAP = {
    -1: "6.00",
    0: "1.00",
    1: "2.00",
    2: "3.00",
    3: "4.00",
}

VALID_DRIVE_VALUES = frozenset(DRIVE_MAP)


def parse_drive_stage(value: float) -> int | None:
    """Return an exact allowed stage, otherwise None.

    Floating-point values such as 1.0 are accepted, but 1.2 is not rounded to 1.
    NaN and Inf are rejected.
    """
    if not math.isfinite(value):
        return None
    rounded = int(round(value))
    if abs(value - rounded) > 1e-6:
        return None
    return rounded if rounded in VALID_DRIVE_VALUES else None


def drive_serial_command(stage: int) -> str:
    return DRIVE_MAP[stage]


def valid_wheel_deg(value: int, max_deg: int = 27) -> bool:
    return -max_deg <= int(value) <= max_deg


def wheel_serial_command(deg: int, max_deg: float, steer_limit_ms: int, mode: str) -> str:
    ms_per_deg = steer_limit_ms / max_deg
    if mode.upper() == "V":
        return f"V,{deg / max_deg:.3f}"
    ms = int(round(deg * ms_per_deg))
    return f"W{ms}"


def parse_status(line: str) -> dict | None:
    """Parse STATUS,state,fault,adc,target_adc,drive_pwm,rpm,count,steer_ms,..."""
    if not line.startswith("STATUS,"):
        return None
    fields = line.split(",")
    if len(fields) < 8:
        return None
    try:
        result = {
            "raw": line,
            "state": fields[1],
            "fault": int(float(fields[2])),
            "adc": int(float(fields[3])),
            "rpm": float(fields[6]),
            "encoder_count": int(float(fields[7])),
            "steer_ms": 0,
        }
        if len(fields) > 8:
            result["steer_ms"] = int(float(fields[8]))
        return result
    except (ValueError, IndexError):
        return None

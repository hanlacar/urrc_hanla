import math
from typing import Iterable, Tuple


class SafetyManager:
    """Validates commands and computes fail-safe behavior."""

    def __init__(
        self,
        drive_validation_mode: str,
        drive_allowed_values: Iterable[float],
        drive_min: float,
        drive_max: float,
        wheel_min: int,
        wheel_max: int,
        drive_value_tolerance: float = 1e-6,
    ):
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
            raise ValueError("drive_allowed_values cannot be empty in allowed_values mode")

    def validate_drive(self, value: float) -> Tuple[bool, str]:
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

    def validate_wheel(self, value: int) -> Tuple[bool, str]:
        # std_msgs/Int32 is integral already; bool is intentionally rejected in pure tests.
        if isinstance(value, bool):
            return False, "invalid_type"
        try:
            numeric = int(value)
        except (TypeError, ValueError, OverflowError):
            return False, "invalid_type"
        if numeric < self.wheel_min or numeric > self.wheel_max:
            return False, "out_of_range"
        return True, "ok"

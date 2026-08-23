import math

from mcu_manager.safety_manager import SafetyManager


def make_manager():
    return SafetyManager(
        drive_validation_mode="allowed_values",
        drive_allowed_values=[-1.0, 0.0, 1.0, 2.0, 3.0],
        drive_min=-1.0,
        drive_max=3.0,
        wheel_min=-27,
        wheel_max=27,
    )


def test_drive_allowed_values():
    safety = make_manager()
    assert safety.validate_drive(-1.0) == (True, "ok")
    assert safety.validate_drive(3.0) == (True, "ok")
    assert safety.validate_drive(2.5) == (False, "not_allowed")


def test_non_finite_drive_is_rejected():
    safety = make_manager()
    assert safety.validate_drive(math.nan)[0] is False
    assert safety.validate_drive(math.inf)[0] is False


def test_wheel_range_is_rejected_not_clamped():
    safety = make_manager()
    assert safety.validate_wheel(-27) == (True, "ok")
    assert safety.validate_wheel(27) == (True, "ok")
    assert safety.validate_wheel(28) == (False, "out_of_range")

from mcu_manager.input_manager import InputManager


def test_drive_and_wheel_freshness_are_independent():
    manager = InputManager(["camera"])
    manager.update_drive("camera", 2.0, now=10.0, valid=True, reason="ok")
    manager.update_wheel("camera", 5, now=9.0, valid=True, reason="ok")

    drive_ok, _, drive = manager.drive_status("camera", now=10.2, timeout_s=0.5)
    wheel_ok, wheel_reason, wheel = manager.wheel_status("camera", now=10.2, timeout_s=0.5)

    assert drive_ok is True
    assert drive == 2.0
    assert wheel_ok is False
    assert wheel_reason == "timeout"
    assert wheel == 5


def test_invalid_update_invalidates_previous_command_immediately():
    manager = InputManager(["gps"])
    manager.update_drive("gps", 2.0, now=1.0, valid=True, reason="ok")
    manager.update_drive("gps", 99.0, now=1.1, valid=False, reason="not_allowed")
    ok, reason, value = manager.drive_status("gps", now=1.2, timeout_s=0.5)
    assert ok is False
    assert reason == "not_allowed"
    assert value == 99.0

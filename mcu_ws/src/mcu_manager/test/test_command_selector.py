import pytest

from mcu_manager.command_selector import CommandSelector


def test_drive_wheel_are_selected_independently():
    selector = CommandSelector(
        ["camera", "lidar", "gps", "imu", "manual"],
        ["SLOPE:imu:camera", "T_PARK:lidar:lidar", "IDLE:none:none"],
    )
    assert selector.select("slope") == ("imu", "camera", True)
    assert selector.select("T_PARK") == ("lidar", "lidar", True)


def test_unknown_mode_fails_safe():
    selector = CommandSelector(["camera"], ["IDLE:none:none"])
    assert selector.select("DOES_NOT_EXIST") == ("none", "none", False)


def test_unknown_source_in_map_is_rejected():
    with pytest.raises(ValueError):
        CommandSelector(["camera"], ["NORMAL:gps:camera"])

import math

from mcu_bridge.protocol import (
    drive_serial_command,
    parse_drive_stage,
    parse_status,
    valid_wheel_deg,
    wheel_serial_command,
)


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
    assert valid_wheel_deg(-27)
    assert valid_wheel_deg(27)
    assert not valid_wheel_deg(-28)
    assert not valid_wheel_deg(28)


def test_wheel_w_protocol():
    assert wheel_serial_command(0, 27, 440, "W") == "W0"
    assert wheel_serial_command(27, 27, 440, "W") == "W440"
    assert wheel_serial_command(-27, 27, 440, "W") == "W-440"


def test_status_parser():
    s = parse_status("STATUS,ACTIVE,0,512,520,120,33.5,1234,-163,0,440")
    assert s is not None
    assert s["state"] == "ACTIVE"
    assert s["fault"] == 0
    assert s["adc"] == 512
    assert s["rpm"] == 33.5
    assert s["encoder_count"] == 1234
    assert s["steer_ms"] == -163

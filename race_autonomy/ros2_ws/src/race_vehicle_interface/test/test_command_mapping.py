from race_vehicle_interface.command_mapping import (
    speed_to_stage,
    steering_command,
)
from race_vehicle_interface.encoder_protocol import parse_encoder_frame
import pytest

from race_vehicle_interface.serial_protocol import (
    encode_commands,
    encode_drive_command,
    encode_emergency_brake_command,
    encode_steering_command,
    encoder_delta_to_distance_m,
    encoder_delta_to_speed_mps,
    meters_per_second_to_kilometers_per_hour,
    parse_telemetry,
    parse_legacy_status,
    parse_legacy_steering_a0,
    parse_v27_status,
    steering_position_to_degrees,
)
from race_vehicle_interface.steering_calibration import (
    straight_run_trim_deg,
    wrapped_angle_delta_deg,
)


def test_uncalibrated_stage_is_zero():
    assert speed_to_stage(2.0, 0.0, 10) == 0
    assert speed_to_stage(2.0, 5.0, 0) == 0


def test_stage_mapping_and_clamp():
    assert speed_to_stage(1.2, 5.0, 10) == 6
    assert speed_to_stage(5.0, 5.0, 10) == 10
    assert speed_to_stage(-5.0, 5.0, 10) == -10


def test_v29_measured_pwm100_speed_maps_to_ros_stage_two():
    measured_speed_mps = 0.526
    assert speed_to_stage(measured_speed_mps, 2.0 / measured_speed_mps, 3) == 2


def test_measured_encoder_distance_and_speed_calibration():
    assert encoder_delta_to_distance_m(5430, 543.0) == pytest.approx(10.0)
    assert encoder_delta_to_speed_mps(5430, 9.12, 543.0) == pytest.approx(
        10.0 / 9.12
    )


def test_speed_kph_conversion():
    assert meters_per_second_to_kilometers_per_hour(1.0) == pytest.approx(3.6)
    assert meters_per_second_to_kilometers_per_hour(-0.5) == pytest.approx(-1.8)


def test_steering_clamped_to_27_degrees():
    assert steering_command(10.0, 27.0) == 10.0
    assert steering_command(40.0, 27.0) == 27.0
    assert steering_command(-40.0, 27.0) == -27.0


def test_sign_configuration():
    assert speed_to_stage(1.0, 4.0, 10, -1.0) == -4
    assert steering_command(12.0, 27.0, -1.0) == -12.0


def test_parse_arduino_v14_telemetry():
    assert parse_telemetry(b"T,1234,56.7,-12.5\r") == (1234, 56.7, -12.5)
    assert parse_telemetry(bytearray(b"T,9,1.5,2\r")) == (9, 1.5, 2.0)


def test_parse_on_demand_korean_v14_status():
    line = "[상태] A0=359 편차=-4 누적=-132/440 RPM=12.5 오도=77 PWM=50"
    assert parse_legacy_status(line) == (77, 12.5, -132.0)
    assert parse_legacy_status(line.encode("utf-8")) == (77, 12.5, -132.0)
    assert parse_legacy_steering_a0(line) == 359
    assert parse_legacy_steering_a0(line.encode("utf-8")) == 359


def test_parse_v27_status_and_steering_adc():
    line = "STATUS,READY,NONE,268,363,0,0.00,77,300,300,440"
    assert parse_v27_status(line) == (77, 0.0, 300.0, 268, 363)
    assert parse_legacy_status(line) == (77, 0.0, 300.0)
    assert parse_legacy_steering_a0(line) == 268


def test_reject_bad_v27_status():
    assert parse_v27_status("STATUS,READY,NONE,bad,363,0,0,0,0,0,440") is None
    assert parse_v27_status("STATUS,READY") is None


def test_reject_unrelated_legacy_line():
    assert parse_legacy_status(">>> 정지") is None
    assert parse_legacy_steering_a0(">>> 정지") is None


def test_ignore_boot_and_status_lines():
    assert parse_telemetry("=== v14 ===") is None
    assert parse_telemetry("[상태] RPM=0.0 PWM=0") is None


def test_reject_bad_or_nonfinite_telemetry():
    assert parse_telemetry("T,abc,0.0,0") is None
    assert parse_telemetry("T,1,nan,0") is None
    assert parse_telemetry("T,1,0") is None


def test_encode_existing_firmware_drive_commands():
    assert encode_drive_command(0, 3) == b"1.00\n"
    assert encode_drive_command(1, 3) == b"2.00\n"
    assert encode_drive_command(2, 3) == b"3.00\n"
    assert encode_drive_command(3, 3) == b"4.00\n"
    assert encode_drive_command(-1, 3) == b"6.00\n"
    assert encode_drive_command(-2, 3) == b"7.00\n"
    assert encode_drive_command(-3, 3) == b"8.00\n"


def test_encode_v29_emergency_brake_command():
    assert encode_emergency_brake_command() == b"B\n"


def test_encode_proportional_steering_command():
    assert encode_steering_command(0.0, 27.0) == b"C\n"
    assert encode_steering_command(13.5, 27.0) == b"V,0.500\n"
    assert encode_steering_command(-27.0, 27.0) == b"V,-1.000\n"
    assert encode_steering_command(40.0, 27.0) == b"V,1.000\n"
    assert encode_commands(1, 13.5, 3, 27.0) == b"2.00\nV,0.500\n"


def test_zero_lookahead_steering_uses_logical_center_not_a0():
    assert encode_commands(0, 0.0, 3, 27.0) == b"1.00\nC\n"


def test_encode_command_rejects_unsafe_configuration_and_types():
    with pytest.raises(ValueError):
        encode_drive_command(1.0, 3)
    with pytest.raises(ValueError):
        encode_steering_command(float("nan"), 27.0)
    with pytest.raises(ValueError):
        encode_drive_command(1, 0)
    with pytest.raises(ValueError):
        encode_drive_command(2, 1)


def test_v14_steering_position_to_estimated_degrees():
    assert steering_position_to_degrees(0, 440, 27) == 0.0
    assert steering_position_to_degrees(220, 440, 27) == 13.5
    assert steering_position_to_degrees(-440, 440, 27) == -27.0
    assert steering_position_to_degrees(999, 440, 27) == 27.0


def test_parse_signed_quadrature_encoder_frame():
    assert parse_encoder_frame(b"E,-12,-2,-1,5,0,1\r") == (
        -12,
        -2,
        -1,
        5,
        0,
        True,
    )
    assert parse_encoder_frame("E,10,1,1,3,2,0") == (
        10,
        1,
        1,
        3,
        2,
        False,
    )


def test_reject_bad_quadrature_encoder_frame():
    assert parse_encoder_frame("E,1,1,2,0,0,1") is None
    assert parse_encoder_frame("E,1,1,1,-1,0,1") is None
    assert parse_encoder_frame("E,1,1,1,0,0,2") is None
    assert parse_encoder_frame("T,1,1,1,0,0,1") is None


def test_wrapped_yaw_delta_crosses_boundary():
    assert wrapped_angle_delta_deg(-179.0, 179.0) == pytest.approx(2.0)
    assert wrapped_angle_delta_deg(179.0, -179.0) == pytest.approx(-2.0)


def test_straight_run_trim_opposes_left_yaw_in_vehicle_command_convention():
    trim = straight_run_trim_deg(5.0, 1.0, 0.73, 1.0, 5.0)
    assert trim > 0.0
    assert trim == pytest.approx(3.645, abs=0.001)


def test_straight_run_trim_is_bounded():
    assert straight_run_trim_deg(20.0, 0.2, 0.73, 1.0, 5.0) == 5.0
    with pytest.raises(ValueError):
        straight_run_trim_deg(1.0, 0.0, 0.73)

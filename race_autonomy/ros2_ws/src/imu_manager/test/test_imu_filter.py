import math
from pathlib import Path

import numpy as np
import yaml

from imu_manager.imu_filter import (
    CALIBRATING, CALIBRATION_FAILED, READY, FilterConfig, ImuFilter,
    OPTICAL_TO_CAMERA_CANDIDATE,
    SlopeStateConfig, SlopeStateDetector, StartupCalibrationConfig,
    StartupCalibrator,
    apply_sensor_axis_transform, axis_matrix_from_row_major,
    calibration_gate_ready, compute_level_alignment, euler_rotation_matrix, level_mounting_rpy,
    normalize_angle_deg, quaternion_from_rpy, transform_sensor_to_base,
    validate_axis_matrix, validate_level_samples, validate_rotation_matrix,
    vehicle_pitch_from_accel,
)


def make_filter(**kwargs):
    return ImuFilter(FilterConfig(**kwargs), np.eye(3))


def test_stationary_gyro_bias():
    f = make_filter(bias_sample_count=3)
    for index in range(3):
        f.update_accel([0, 0, 9.80665], 1.0 + index * 0.01)
        f.update_gyro([0.01, -0.02, 0.005], 1.0 + index * 0.01)
    assert f.bias_ready
    assert np.allclose(f.gyro_bias, [0.01, -0.02, 0.005])


def test_constant_yaw_rate_integration():
    f = make_filter(bias_sample_count=1)
    f.bias_ready = True
    f.update_accel([0, 0, 9.80665], 1.0)
    f.update_gyro([0, 0, 0], 1.0)
    f.update_gyro([0, 0, math.pi / 2], 1.1)
    assert math.isclose(f.relative_yaw_deg, 9.0, abs_tol=1e-6)


def test_timestamp_regression_and_duplicate():
    f = make_filter()
    assert f.update_accel([0, 0, 9.80665], 2.0)
    assert not f.update_accel([0, 0, 9.80665], 2.0)
    assert not f.update_accel([0, 0, 9.80665], 1.9)


def test_large_dt_is_not_integrated_and_next_sample_recovers():
    f = make_filter(max_dt_sec=0.1, bias_sample_count=1)
    f.bias_ready = True
    f.update_gyro([0, 0, 0], 1.0)
    assert not f.update_gyro([0, 0, 1], 2.0)
    assert f.last_attitude_stamp is None
    assert f.update_gyro([0, 0, 1], 2.05)
    assert f.last_attitude_stamp == 2.05
    assert f.relative_yaw_deg > 0.0


def test_stale_data():
    f = make_filter()
    f.last_gyro_stamp = f.last_accel_stamp = 1.0
    assert f.is_fresh(1.1, 0.2)
    assert not f.is_fresh(1.3, 0.2)


def test_mode_transition_resets_reference():
    f = make_filter(); f.yaw_total_deg = 42.0
    f.handle_mode_transition("NORMAL", "PARALLEL_PARK")
    assert f.relative_yaw_deg == 0.0


def test_reset_reference_for_each_parking_stage():
    f = make_filter(); f.yaw_total_deg = 10.0; f.reset_reference()
    f.yaw_total_deg = 25.0
    assert f.relative_yaw_deg == 15.0
    f.reset_reference()
    assert f.relative_yaw_deg == 0.0


def test_mounting_rotation():
    f = ImuFilter(FilterConfig(), np.eye(3), (0, 0, 90))
    assert np.allclose(f.transform_sensor_to_vehicle_frame([1, 0, 0]), [0, 1, 0], atol=1e-8)


def test_nan_and_inf_rejected():
    f = make_filter()
    assert not f.update_gyro([math.nan, 0, 0], 1.0)
    assert not f.update_accel([0, math.inf, 0], 1.0)


def test_missing_axis_transform_is_invalid():
    f = ImuFilter(FilterConfig(), None)
    assert not f.transform_valid
    assert f.transform_sensor_to_vehicle_frame([1, 2, 3]) is None


def test_optical_to_camera_candidate_matrix():
    assert np.allclose(OPTICAL_TO_CAMERA_CANDIDATE @ [1, 2, 3], [3, -1, -2])


def test_accel_and_gyro_use_identical_transform():
    f = ImuFilter(FilterConfig(), OPTICAL_TO_CAMERA_CANDIDATE, (2, -3, 5))
    vector = [0.2, -0.4, 0.8]
    expected = f.mounting_rotation @ f.axis_matrix @ vector
    assert np.allclose(f.transform_sensor_to_vehicle_frame(vector), expected)


def test_level_gravity_mounting_correction():
    camera_gravity = np.array([-3.8, -0.7, -8.9])
    rpy = level_mounting_rpy(camera_gravity, yaw_deg=30.0)
    corrected = euler_rotation_matrix(*rpy) @ camera_gravity
    assert np.allclose(corrected[:2], [0, 0], atol=1e-8)
    assert corrected[2] > 0


def test_pitch_roll_and_yaw_signs():
    pitch_vector = np.array([-1.0, 0.0, 9.8])
    roll_vector = np.array([0.0, 1.0, 9.8])
    pitch = math.degrees(math.atan2(-pitch_vector[0], math.hypot(pitch_vector[1], pitch_vector[2])))
    roll = math.degrees(math.atan2(roll_vector[1], roll_vector[2]))
    assert pitch > 0 and roll > 0
    f = ImuFilter(FilterConfig(), OPTICAL_TO_CAMERA_CANDIDATE)
    assert f.transform_sensor_to_vehicle_frame([0, -1, 0])[2] > 0


def test_bad_axis_matrices_rejected():
    assert not validate_axis_matrix(np.diag([1, 1, 2]))
    assert not validate_axis_matrix([[math.nan, 0, 0], [0, 1, 0], [0, 0, 1]])


def test_quaternion_is_normalized():
    assert math.isclose(np.linalg.norm(quaternion_from_rpy(20, -10, 35)), 1.0, abs_tol=1e-12)


def test_calibration_motion_and_gravity_safety():
    stationary_accel = np.tile([0.0, 0.0, 9.81], (10, 1))
    moving_gyro = np.tile([0.0, 0.0, 0.2], (10, 1))
    assert any("motion" in error for error in validate_level_samples(stationary_accel, moving_gyro))
    bad_gravity = np.tile([0.0, 0.0, 5.0], (10, 1))
    quiet_gyro = np.zeros((10, 3))
    assert any("gravity" in error for error in validate_level_samples(bad_gravity, quiet_gyro))


def test_nonfinite_mounting_rpy_is_invalid():
    assert not ImuFilter(FilterConfig(), np.eye(3), (0, math.nan, 0)).transform_valid


def test_measured_d456_level_regression_selects_small_correction():
    gravity = np.array([-0.1555687, -0.8670082, 9.6505595])
    result = compute_level_alignment(gravity)
    assert math.isclose(abs(result.correction_roll_deg), 5.1, abs_tol=0.1)
    assert math.isclose(abs(result.correction_pitch_deg), 0.92, abs_tol=0.03)
    assert abs(result.correction_roll_deg) < 170.0
    assert np.allclose(result.gravity_after[:2], [0.0, 0.0], atol=1e-8)
    assert result.gravity_after[2] > 0.0
    assert math.isclose(np.linalg.norm(result.gravity_after), np.linalg.norm(gravity), rel_tol=1e-12)
    quaternion = quaternion_from_rpy(
        result.correction_roll_deg, result.correction_pitch_deg, 0.0)
    assert math.isclose(np.linalg.norm(quaternion), 1.0, abs_tol=1e-12)
    assert result.physical_sanity_passed


def test_already_positive_z_requires_zero_correction():
    result = compute_level_alignment([0.0, 0.0, 9.81])
    assert result.correction_roll_deg == 0.0
    assert result.correction_pitch_deg == 0.0
    assert result.correction_rotation_magnitude_deg == 0.0


def test_negative_z_requires_explicit_inverted_mount_and_plausibility_override():
    rejected = compute_level_alignment([0.0, 0.0, -9.81])
    assert not rejected.physical_sanity_passed
    assert any("inverted" in error for error in rejected.errors)
    accepted = compute_level_alignment(
        [0.0, 0.0, -9.81],
        allow_inverted_mount=True,
        plausible_mount_angle_limit_deg=180.0,
    )
    assert accepted.gravity_after[2] > 0.0
    assert accepted.physical_sanity_passed


def test_minimum_euler_branch_beats_180_degree_equivalent():
    result = compute_level_alignment([-0.1555687, -0.8670082, 9.6505595])
    assert result.selected_euler_branch == "atan2_branch_0_minimum_rotation"
    assert result.correction_rotation_magnitude_deg < 10.0


def test_plausible_mount_limit_rejects_large_candidate():
    gravity = euler_rotation_matrix(-60.0, 0.0, 0.0).T @ [0.0, 0.0, 9.81]
    result = compute_level_alignment(gravity, plausible_mount_angle_limit_deg=45.0)
    assert not result.physical_sanity_passed
    assert any("plausible" in error for error in result.errors)


def test_level_alignment_rejects_nan_and_bad_norm():
    for gravity in ([math.nan, 0.0, 9.81], [0.0, 0.0, 5.0]):
        try:
            compute_level_alignment(gravity)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid gravity must be rejected")


def test_measured_attitude_and_active_correction_are_distinct_inverse_meanings():
    result = compute_level_alignment([-0.1555687, -0.8670082, 9.6505595])
    assert result.measured_roll_deg * result.correction_roll_deg < 0.0
    assert result.measured_pitch_deg * result.correction_pitch_deg < 0.0
    rotation = euler_rotation_matrix(
        result.correction_roll_deg, result.correction_pitch_deg, 0.0)
    assert np.allclose(rotation @ result.gravity_before, result.gravity_after)


def test_angle_normalization_half_open_interval():
    assert normalize_angle_deg(180.0) == -180.0
    assert normalize_angle_deg(540.0) == -180.0
    assert normalize_angle_deg(-181.0) == 179.0


def test_reported_matrix_matches_measured_raw_gravity_regression():
    raw = np.array([0.15771296095017087, -0.07676137844988014, -9.68876305697794])
    matrix = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    expected = np.array([-9.68876305697794, -0.15771296095017087, 0.07676137844988014])
    transformed = apply_sensor_axis_transform(matrix, raw)
    assert np.array_equal(np.sign(transformed), np.sign(expected))
    assert np.allclose(transformed, expected, atol=1e-15)
    assert math.isclose(np.linalg.norm(transformed), np.linalg.norm(raw), rel_tol=1e-15)
    assert not np.allclose(matrix.T @ raw, expected)


def test_manager_filter_and_calibration_common_path_are_identical():
    raw = np.array([0.15771296095017087, -0.07676137844988014, -9.68876305697794])
    matrix = OPTICAL_TO_CAMERA_CANDIDATE
    rpy = (2.0, -3.0, 4.0)
    common = transform_sensor_to_base(raw, matrix, rpy)
    filtered = ImuFilter(FilterConfig(), matrix, rpy).transform_sensor_to_vehicle_frame(raw)
    calibration_axis_value = apply_sensor_axis_transform(matrix, raw)
    assert np.array_equal(calibration_axis_value, common.axis_transformed_vector)
    assert np.allclose(filtered, common.base_link_vector, atol=1e-15)


def test_yaml_row_major_round_trip_preserves_matrix_without_transpose():
    matrix = OPTICAL_TO_CAMERA_CANDIDATE
    serialized = yaml.safe_dump({"sensor_axis_matrix": matrix.reshape(-1, order="C").tolist()})
    values = yaml.safe_load(serialized)["sensor_axis_matrix"]
    restored = axis_matrix_from_row_major(values)
    assert np.array_equal(restored, matrix)
    assert not np.array_equal(restored, matrix.T)


def test_signed_permutation_validation_records_reflection_policy():
    proper = validate_rotation_matrix(OPTICAL_TO_CAMERA_CANDIDATE)
    assert proper.valid and proper.signed_permutation
    assert proper.orthogonality_error == 0.0
    reflection_matrix = np.diag([-1.0, 1.0, 1.0])
    allowed = validate_rotation_matrix(reflection_matrix, allow_reflection=True)
    rejected = validate_rotation_matrix(reflection_matrix, allow_reflection=False)
    assert allowed.valid and allowed.reflection and allowed.determinant == -1.0
    assert not rejected.valid and rejected.reflection


def test_rotation_matrix_rejects_orthogonal_but_not_signed_permutation():
    rotation_45 = euler_rotation_matrix(0.0, 0.0, 45.0)
    result = validate_rotation_matrix(rotation_45)
    assert not result.valid
    assert not result.signed_permutation


def feed_startup(calibrator, gravity=(0.0, 0.0, 9.80665), bias=(0.01, -0.02, 0.005),
                 start=1.0, duration=3.0, step=0.01):
    count = int(round(duration / step)) + 1
    for index in range(count):
        stamp = start + index * step
        calibrator.add_accel(gravity, stamp, now_monotonic=stamp - start)
        calibrator.add_gyro(bias, stamp, now_monotonic=stamp - start)


def test_startup_calibration_succeeds_after_continuous_three_seconds():
    c = StartupCalibrator(StartupCalibrationConfig(), started_monotonic=0.0)
    feed_startup(c)
    result = c.try_complete(3.0, mounting_yaw_deg=0.0)
    assert c.state == READY
    assert result is not None and result.duration_sec >= 3.0
    assert np.allclose(result.gyro_bias_axis, [0.01, -0.02, 0.005])
    assert abs(result.mounting_roll_deg) < 1e-12
    assert abs(result.mounting_pitch_deg) < 1e-12


def test_startup_motion_discards_window_and_restarts_duration():
    c = StartupCalibrator(StartupCalibrationConfig(), started_monotonic=0.0)
    feed_startup(c, duration=2.0)
    assert not c.add_gyro([0.1, 0.0, 0.0], 3.1, now_monotonic=2.1)
    assert c.restart_count == 1
    feed_startup(c, start=3.2, duration=2.9)
    assert c.try_complete(5.0) is None
    feed_startup(c, start=6.11, duration=0.2)
    # The helper's second segment remains continuous in sensor time.
    assert c.try_complete(5.3) is not None


def test_startup_timeout_bad_norm_nonfinite_and_timestamp_regression():
    timeout = StartupCalibrator(StartupCalibrationConfig(timeout_sec=1.0), 0.0)
    assert timeout.check_timeout(1.01) == CALIBRATION_FAILED
    c = StartupCalibrator(StartupCalibrationConfig(), 0.0)
    assert not c.add_accel([0.0, 0.0, 5.0], 1.0, 0.0)
    assert not c.add_accel([math.nan, 0.0, 9.8], 1.1, 0.1)
    assert c.add_accel([0.0, 0.0, 9.8], 2.0, 0.2)
    assert not c.add_accel([0.0, 0.0, 9.8], 1.9, 0.3)
    assert c.state == CALIBRATING


def startup_result_for_mounting(mounting_rpy):
    gravity_base = np.array([0.0, 0.0, 9.80665])
    gravity_axis = euler_rotation_matrix(*mounting_rpy).T @ gravity_base
    c = StartupCalibrator(StartupCalibrationConfig(), 0.0)
    feed_startup(c, gravity=gravity_axis)
    return c.try_complete(3.0, mounting_yaw_deg=mounting_rpy[2])


def test_startup_pitch_roll_offsets_are_removed_in_memory():
    for mounting in ((0.0, -35.0, 0.0), (0.0, 20.0, 0.0),
                     (12.0, -8.0, 0.0)):
        result = startup_result_for_mounting(mounting)
        assert result is not None
        f = ImuFilter(FilterConfig(), np.eye(3))
        assert f.apply_startup_calibration(
            (result.mounting_roll_deg, result.mounting_pitch_deg, result.mounting_yaw_deg),
            result.gyro_bias_axis)
        raw_level = euler_rotation_matrix(*mounting).T @ [0.0, 0.0, 9.80665]
        corrected = f.transform_sensor_to_vehicle_frame(raw_level)
        assert math.isclose(vehicle_pitch_from_accel(corrected), 0.0, abs_tol=1e-8)
        assert math.isclose(math.degrees(math.atan2(corrected[1], corrected[2])),
                            0.0, abs_tol=1e-8)


def test_startup_correction_preserves_real_positive_and_negative_slope():
    mounting = (0.0, -35.0, 0.0)
    result = startup_result_for_mounting(mounting)
    f = ImuFilter(FilterConfig(), np.eye(3))
    f.apply_startup_calibration(
        (result.mounting_roll_deg, result.mounting_pitch_deg, 0.0), result.gyro_bias_axis)
    for road_pitch in (25.0, -25.0):
        base = np.array([-9.80665 * math.sin(math.radians(road_pitch)), 0.0,
                         9.80665 * math.cos(math.radians(road_pitch))])
        raw = euler_rotation_matrix(*mounting).T @ base
        assert math.isclose(vehicle_pitch_from_accel(f.transform_sensor_to_vehicle_frame(raw)),
                            road_pitch, abs_tol=1e-8)


def detector():
    return SlopeStateDetector(SlopeStateConfig())


def test_slope_threshold_boundaries_and_sign():
    expected = ((24.99, False), (25.0, True), (25.01, True),
                (-24.99, False), (-25.0, True), (-30.0, True))
    for pitch, state in expected:
        assert detector().update(pitch, True) is state


def test_slope_changes_immediately_without_confirmation_hysteresis_or_rate_limit():
    d = detector()
    assert d.update(25.0, True)
    assert not d.update(24.99, True)
    assert d.update(25.0, True)
    assert d.update(-30.0, True)


def test_slope_invalid_nonfinite_and_internal_rounding_boundary():
    d = detector()
    assert not d.update(30.0, False)
    for pitch in (math.nan, math.inf, -math.inf):
        assert not d.update(pitch, True)
    assert not d.update(24.996, True)
    assert round(24.996, 2) == 25.0
    assert d.update(25.000, True)


def test_slope_uses_pitch_only():
    # Roll/Yaw cannot be passed to this detector and therefore cannot affect it.
    assert not detector().update(0.0, True)


def test_yaw_reset_does_not_change_startup_mounting_calibration():
    f = ImuFilter(FilterConfig(), np.eye(3))
    f.apply_startup_calibration((5.0, -10.0, 0.0), [0.01, 0.02, 0.03])
    mounting_before = f.mounting_rpy_deg.copy()
    f.yaw_total_deg = 42.0
    f.reset_reference()
    assert f.relative_yaw_deg == 0.0
    assert np.array_equal(f.mounting_rpy_deg, mounting_before)


def test_yaw_reset_does_not_change_slope_threshold_state():
    f = ImuFilter(FilterConfig(), np.eye(3))
    d = detector()
    assert d.update(25.0, True)
    f.reset_reference()
    assert d.state


def test_slope_implementation_has_no_timer_latch_or_drive_publishers():
    package = Path(__file__).resolve().parents[1]
    implementation = "\n".join(
        (package / relative).read_text(encoding="utf-8")
        for relative in (
            "imu_manager/imu_filter.py",
            "imu_manager/imu_manager_node.py",
            "config/imu_manager.yaml",
        )
    )
    forbidden = (
        "slope_confirm_duration_sec", "slope_pitch_rate_limit_deg_s",
        "slope_release_deg", "confirm_started_at", "/slope/stop",
        "RAMP_CONFIRMED", "STOPPING", "stop_duration", "drive_pub",
        "wheel_pub",
    )
    assert all(token not in implementation for token in forbidden)


def test_runtime_ready_is_not_blocked_by_legacy_static_calibration_flag():
    assert calibration_gate_ready(True, READY, False)
    assert not calibration_gate_ready(True, CALIBRATING, True)
    assert not calibration_gate_ready(True, CALIBRATION_FAILED, True)
    assert calibration_gate_ready(False, CALIBRATING, True)
    assert not calibration_gate_ready(False, READY, False)

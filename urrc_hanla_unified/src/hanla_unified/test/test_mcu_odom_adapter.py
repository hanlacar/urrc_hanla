from pathlib import Path

from builtin_interfaces.msg import Time
from hanla_unified.mcu_odom_adapter import (
    BicycleOdometry,
    make_odometry,
    make_transform,
)
from launch import LaunchContext
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
import pytest


def integrator(*, signed=False, max_delta=1000):
    return BicycleOdometry(
        wheelbase_m=0.73,
        counts_per_meter=100.0,
        max_encoder_delta_counts=max_delta,
        encoder_counts_are_signed=signed,
    )


def baseline(state, count=100, stamp=1.0):
    result = state.update_encoder(count, stamp)
    assert result.accepted and result.baseline_only
    return result


def test_first_encoder_sample_only_sets_baseline():
    state = integrator()
    baseline(state, 500)
    assert (state.x, state.y, state.yaw) == (0.0, 0.0, 0.0)


def test_straight_forward_increases_x_and_reports_positive_speed():
    state = integrator()
    state.set_applied_drive(1.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    assert state.x == pytest.approx(0.4)
    assert state.y == pytest.approx(0.0)
    assert state.yaw == pytest.approx(0.0)
    assert result.linear_x == pytest.approx(0.4)


def test_straight_reverse_decreases_x_and_reports_negative_speed():
    state = integrator()
    state.set_applied_drive(-1.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    assert state.x == pytest.approx(-0.4)
    assert state.y == pytest.approx(0.0)
    assert state.yaw == pytest.approx(0.0)
    assert result.ds < 0.0
    assert result.linear_x < 0.0


def test_forward_left_has_positive_yaw():
    state = integrator()
    state.set_applied_drive(1.0)
    state.set_steering_deg(10.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    assert state.yaw > 0.0
    assert result.dtheta > 0.0


def test_forward_right_has_negative_yaw():
    state = integrator()
    state.set_applied_drive(1.0)
    state.set_steering_deg(-10.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    assert state.yaw < 0.0
    assert result.dtheta < 0.0


def test_reverse_steering_uses_bicycle_sign_convention():
    state = integrator()
    state.set_applied_drive(-1.0)
    state.set_steering_deg(10.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    assert result.ds < 0.0
    assert state.yaw < 0.0


def test_invalid_steering_feedback_cannot_change_odom_yaw():
    state = BicycleOdometry(
        wheelbase_m=0.73,
        counts_per_meter=797.0,
        max_encoder_delta_counts=1000,
        steering_feedback_required=True,
    )
    state.set_applied_drive(1.0)
    state.set_steering_deg(-25.5)
    baseline(state, count=0)
    result = state.update_encoder(100, 2.0)
    assert result.ds == pytest.approx(100.0 / 797.0)
    assert result.dtheta == 0.0
    assert state.yaw == 0.0
    odom = make_odometry(
        Time(sec=2), state, result, "odom", "base_link")
    assert odom.pose.covariance[35] == pytest.approx(1.0e3)


def test_validity_gate_enables_measured_steering_for_odom():
    state = BicycleOdometry(
        wheelbase_m=0.73,
        counts_per_meter=797.0,
        max_encoder_delta_counts=1000,
        steering_feedback_required=True,
    )
    state.set_applied_drive(1.0)
    state.set_steering_deg(10.0)
    state.set_steering_valid(True)
    baseline(state, count=0)
    result = state.update_encoder(100, 2.0)
    assert result.dtheta > 0.0
    assert state.yaw > 0.0


def test_monotonic_encoder_reset_is_rebaselined_without_pose_jump():
    state = integrator()
    state.set_applied_drive(1.0)
    baseline(state, 500)
    state.update_encoder(520, 2.0)
    pose_before = (state.x, state.y, state.yaw)
    result = state.update_encoder(0, 3.0)
    assert not result.accepted
    assert result.baseline_only
    assert (state.x, state.y, state.yaw) == pose_before
    state.update_encoder(10, 4.0)
    assert state.x > pose_before[0]


def test_large_encoder_delta_is_rebaselined_without_pose_jump():
    state = integrator(max_delta=50)
    state.set_applied_drive(1.0)
    baseline(state)
    result = state.update_encoder(1000, 2.0)
    assert not result.accepted
    assert (state.x, state.y, state.yaw) == (0.0, 0.0, 0.0)
    state.update_encoder(1010, 3.0)
    assert state.x == pytest.approx(0.1)


def test_zero_delta_keeps_pose_and_zero_twist():
    state = integrator()
    state.set_applied_drive(1.0)
    baseline(state)
    result = state.update_encoder(100, 2.0)
    assert (state.x, state.y, state.yaw) == (0.0, 0.0, 0.0)
    assert result.linear_x == 0.0
    assert result.angular_z == 0.0


def test_signed_encoder_mode_does_not_apply_drive_sign_twice():
    state = integrator(signed=True)
    state.set_applied_drive(-1.0)
    baseline(state)
    result = state.update_encoder(60, 2.0)
    assert result.ds == pytest.approx(-0.4)
    assert state.x == pytest.approx(-0.4)


def test_odom_frames_and_tf_pose_match_exactly():
    state = integrator()
    state.set_applied_drive(1.0)
    state.set_steering_deg(10.0)
    baseline(state)
    result = state.update_encoder(140, 2.0)
    stamp = Time(sec=2)
    odom = make_odometry(stamp, state, result, "odom", "base_link")
    transform = make_transform(stamp, state, "odom", "base_link")

    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_link"
    assert transform.transform.translation.x == odom.pose.pose.position.x
    assert transform.transform.translation.y == odom.pose.pose.position.y
    assert transform.transform.translation.z == odom.pose.pose.position.z
    assert transform.transform.rotation == odom.pose.pose.orientation
    assert odom.twist.twist.angular.z == pytest.approx(
        result.dtheta / 1.0)


def test_system_launch_prevents_fake_and_real_odom_duplicates():
    source = (
        Path(__file__).parents[1] / "launch" / "system.launch.py"
    ).read_text()
    assert '"enable_mcu_odom_adapter", default_value="true"' in source
    assert "AndSubstitution(" in source
    assert 'NotSubstitution(LaunchConfiguration("bench_fake_odom"))' in source
    assert 'executable="mcu_odom_adapter"' in source
    assert '"mcu_odom_counts_per_meter", default_value="797.0"' in source
    assert '"steering_feedback_required": True' in source

    condition = IfCondition(AndSubstitution(
        LaunchConfiguration("enable_mcu_odom_adapter"),
        NotSubstitution(LaunchConfiguration("bench_fake_odom"))))
    context = LaunchContext()
    context.launch_configurations["enable_mcu_odom_adapter"] = "true"
    context.launch_configurations["bench_fake_odom"] = "false"
    assert condition.evaluate(context)
    context.launch_configurations["bench_fake_odom"] = "true"
    assert not condition.evaluate(context)
    context.launch_configurations["enable_mcu_odom_adapter"] = "false"
    context.launch_configurations["bench_fake_odom"] = "false"
    assert not condition.evaluate(context)


def test_invalid_parameters_are_rejected():
    with pytest.raises(ValueError):
        BicycleOdometry(
            wheelbase_m=0.0, counts_per_meter=100.0,
            max_encoder_delta_counts=100)
    with pytest.raises(ValueError):
        BicycleOdometry(
            wheelbase_m=0.73, counts_per_meter=0.0,
            max_encoder_delta_counts=100)
    with pytest.raises(ValueError):
        BicycleOdometry(
            wheelbase_m=0.73, counts_per_meter=100.0,
            max_encoder_delta_counts=0)

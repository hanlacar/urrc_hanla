import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import TransformStamped
from lidar_ws_plus_bringup import mcu_simple_compat as compat
from lidar_ws_plus_bringup.command_mux import (
    choose_output, select_source, SourceState, valid_drive)
from lidar_ws_plus_bringup.mcu_simple_compat import (
    make_bench_odometry, make_bench_static_transform, McuSimpleCompat,
    translate_drive, translate_speed_mps, translate_wheel)
from rclpy.time import Time
from tf2_ros import Buffer


def fresh_source(now=10.0):
    return SourceState(
        drive=1.0, wheel=-12, stop=False,
        drive_time=now, wheel_time=now, stop_time=now,
        drive_valid=True, wheel_valid=True)


def test_mode_contract_selects_only_configured_source():
    assert select_source('T_PARK', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'parking'
    assert select_source('parallel_park', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'parking'
    assert select_source('5', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'avoidance'
    assert select_source('NORMAL', ['T_PARK', 'PARALLEL_PARK'], ['5']) is None


def test_drive_contract_is_exact_discrete_mcu_set():
    for value in (-1.0, 0.0, 1.0, 2.0, 3.0):
        assert valid_drive(value)
    for value in (-2.0, 0.5, 4.0, math.nan, math.inf):
        assert not valid_drive(value)


def test_fresh_active_command_passes_without_sign_change():
    assert choose_output(fresh_source(), 10.1, 0.5) == (
        1.0, -12, False, 'ACTIVE')


def test_stop_overrides_drive_but_preserves_safe_steering_command():
    source = fresh_source()
    source.stop = True
    assert choose_output(source, 10.1, 0.5) == (
        0.0, -12, True, 'SOURCE_STOP')


def test_watchdog_is_fail_closed():
    assert choose_output(fresh_source(), 10.6, 0.5) == (
        0.0, 0, True, 'ACTIVE_SOURCE_TIMEOUT_OR_INVALID')


def test_forced_stop_has_highest_priority():
    assert choose_output(fresh_source(), 10.1, 0.5, True) == (
        0.0, 0, True, 'GLOBAL_STOP')


def test_non_lidar_mode_is_zero_and_non_latching():
    assert choose_output(None, 10.0, 0.5) == (
        0.0, 0, False, 'NON_LIDAR_MODE')


def test_simple_bridge_converts_legacy_right_positive_to_mcu_left_positive():
    # Legacy -12 means left; SIMPLE +12 means left.
    assert translate_wheel(-12, sign_multiplier=-1, limit_deg=22) == 12
    assert translate_wheel(12, sign_multiplier=-1, limit_deg=22) == -12


def test_simple_bridge_rejects_over_limit_wheel_instead_of_clamping():
    assert translate_wheel(-23, sign_multiplier=-1, limit_deg=22) is None


def test_simple_bridge_bench_drive_is_limited_to_level_one():
    assert translate_drive(1.0, max_forward_level=1) == 1.0
    assert translate_drive(2.0, max_forward_level=1) is None
    assert translate_drive(float('nan'), max_forward_level=1) is None


def test_integrated_speed_is_converted_to_calibrated_simple_levels():
    scale = 4.3956043956
    assert translate_speed_mps(0.0, scale, 3) == 0.0
    assert translate_speed_mps(0.229, scale, 3) == 1.0
    assert translate_speed_mps(0.455, scale, 3) == 2.0
    assert translate_speed_mps(0.70, scale, 3) == 3.0
    assert translate_speed_mps(float('nan'), scale, 3) is None
    assert translate_speed_mps(0.70, scale, 1) is None


def test_float_wheel_input_fails_closed_when_non_finite():
    assert translate_wheel(float('nan')) is None
    assert translate_wheel(float('inf')) is None


def test_real_vehicle_launch_keeps_simple_bridge_and_fake_odom_opt_in():
    source = (Path(__file__).parents[1] / 'launch' /
              'real_vehicle.launch.py').read_text()
    assert "'enable_mcu_simple_compat', default_value='false'" in source
    assert "'bench_fake_odom', default_value='false'" in source
    assert "'front_laser_yaw', default_value='3.14159265359'" in source
    assert "'left_curb_inner_y_m', default_value='1.095'" in source
    assert "'right_curb_inner_y_m', default_value='-1.095'" in source
    assert "'replan_trigger_distance_m', default_value='2.0'" in source
    assert "'auto_start_avoidance': ParameterValue(" in source
    assert "'auto_start': ParameterValue(" not in source


def test_real_vehicle_debug_visualization_is_opt_in_and_uses_one_rviz():
    package = Path(__file__).parents[1]
    launch = (package / 'launch' / 'real_vehicle.launch.py').read_text()
    rviz = (package / 'rviz' / 'real_avoidance_debug.rviz').read_text()
    setup = (package / 'setup.py').read_text()
    assert "'debug_visualization', default_value='false'" in launch
    assert "'launch_debug_rviz', default_value='false'" in launch
    assert "'publish_rejected_points', default_value='false'" in launch
    assert 'UnlessCondition' in launch
    assert 'real_avoidance_debug.rviz' in launch
    assert "'rviz/*.rviz'" in setup
    for topic in (
            '/front/scan', '/avoidance/debug/roi',
            '/avoidance/debug/roi_points',
            '/avoidance/debug/rejected_points',
            '/avoidance/debug/obstacles',
            '/avoidance/debug/selected_obstacle',
            '/avoidance/debug/collision',
            '/avoidance/route/reference_path',
            '/avoidance/candidate_paths', '/avoidance/selected_path'):
        assert topic in rviz
    assert 'Fixed Frame: base_link' in rviz


def test_bench_fake_odom_owns_one_static_transform_and_periodic_odom(
        monkeypatch):
    broadcaster = Mock()
    constructor = Mock(return_value=broadcaster)
    monkeypatch.setattr(compat, 'StaticTransformBroadcaster', constructor)
    odom_publisher = Mock()
    stamp = Time(nanoseconds=1_000_000_000).to_msg()
    fake = SimpleNamespace(
        p={'bench_fake_odom': True},
        create_publisher=Mock(return_value=odom_publisher),
        get_clock=Mock(return_value=SimpleNamespace(
            now=Mock(return_value=SimpleNamespace(to_msg=Mock(
                return_value=stamp))))))
    McuSimpleCompat._configure_bench_fake_odom(fake)
    assert fake.pub_odom is odom_publisher
    assert fake.static_tf_br is broadcaster
    constructor.assert_called_once_with(fake)
    broadcaster.sendTransform.assert_called_once()
    transform = broadcaster.sendTransform.call_args.args[0]
    assert (transform.header.frame_id, transform.child_frame_id) == (
        'odom', 'base_link')
    assert transform.transform.translation.x == 0.0
    assert transform.transform.rotation.w == 1.0

    McuSimpleCompat._publish_bench_odometry(fake)
    odom_publisher.publish.assert_called_once()
    odom = odom_publisher.publish.call_args.args[0]
    assert (odom.header.frame_id, odom.child_frame_id) == (
        'odom', 'base_link')
    assert odom.pose.pose.position.x == 0.0
    assert odom.pose.pose.orientation.w == 1.0
    assert odom.twist.twist.linear.x == 0.0
    assert odom.twist.twist.angular.z == 0.0
    broadcaster.sendTransform.assert_called_once()


def test_production_mode_creates_neither_fake_odom_nor_fake_tf(monkeypatch):
    constructor = Mock()
    monkeypatch.setattr(compat, 'StaticTransformBroadcaster', constructor)
    fake = SimpleNamespace(
        p={'bench_fake_odom': False}, create_publisher=Mock())
    McuSimpleCompat._configure_bench_fake_odom(fake)
    assert fake.pub_odom is None
    assert fake.static_tf_br is None
    fake.create_publisher.assert_not_called()
    constructor.assert_not_called()


def test_fake_odom_has_no_dynamic_transform_broadcaster_or_timer_tf():
    assert not hasattr(compat, 'TransformBroadcaster')
    timer_source = __import__('inspect').getsource(McuSimpleCompat.timer_cb)
    assert 'sendTransform' not in timer_source


def test_bench_static_tf_is_valid_at_arbitrary_scan_timestamps():
    buffer = Buffer()
    buffer.set_transform_static(
        make_bench_static_transform(Time(nanoseconds=1).to_msg()), 'bench')
    front = TransformStamped()
    front.header.frame_id = 'base_link'
    front.child_frame_id = 'front_laser'
    front.transform.translation.x = 0.73
    front.transform.translation.z = 0.105
    front.transform.rotation.z = 1.0
    buffer.set_transform_static(front, 'bench')
    for nanoseconds in (10_000_000, 1_000_000_000, 123_456_789_000):
        result = buffer.lookup_transform(
            'odom', 'front_laser', Time(nanoseconds=nanoseconds))
        assert result.header.frame_id == 'odom'
        assert result.child_frame_id == 'front_laser'


def test_bench_message_builders_keep_fixed_zero_pose_and_twist():
    first_stamp = Time(nanoseconds=10).to_msg()
    later_stamp = Time(nanoseconds=999_999_999).to_msg()
    transform = make_bench_static_transform(first_stamp)
    first = make_bench_odometry(first_stamp)
    later = make_bench_odometry(later_stamp)
    assert transform.header.stamp == first_stamp
    assert first.header.stamp == first_stamp
    assert later.header.stamp == later_stamp
    assert first.pose.pose == later.pose.pose
    assert first.twist.twist == later.twist.twist

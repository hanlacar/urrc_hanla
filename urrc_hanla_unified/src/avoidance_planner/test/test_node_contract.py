from collections import deque
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from visualization_msgs.msg import Marker

from avoidance_planner.collision_evaluator import ReplanDebounce
from avoidance_planner.coordinator_node import AvoidanceCoordinator, PLANNER_STATES


def collision_item(track_id, distance, required=True):
    track = SimpleNamespace(track_id=track_id)
    risk = SimpleNamespace(
        required=required, collision_path_distance=distance)
    return distance, track, risk


def collision_confirmation_fake():
    fake = SimpleNamespace(
        scan_count=0, last_collision_evaluation_scan_count=-1,
        debounce=ReplanDebounce(3),
        p={'replan_trigger_distance_m': 2.0})
    fake._replan_trigger_distance = lambda: float(
        fake.p['replan_trigger_distance_m'])
    return fake


def consume_frame(fake, scan_count, risks):
    fake.scan_count = scan_count
    return AvoidanceCoordinator._consume_collision_frame(fake, risks)


def test_same_selected_track_confirms_on_three_fresh_front_frames():
    fake = collision_confirmation_fake()
    risk = (collision_item(4, 0.40),)
    first = consume_frame(fake, 1, risk)
    assert first[0][1].track_id == 4
    assert first[1:] == (False, True)
    assert fake.debounce.count == 1
    second = consume_frame(fake, 2, risk)
    assert second[1:] == (False, True)
    assert fake.debounce.count == 2
    third = consume_frame(fake, 3, risk)
    assert third[1:] == (True, True)
    assert fake.debounce.count == 3


def test_multiple_collision_tracks_update_only_selected_track_once():
    fake = collision_confirmation_fake()
    fake.debounce.update = Mock(wraps=fake.debounce.update)
    distances = {1: 0.10, 2: 0.20, 3: 0.30, 4: 0.40,
                 5: 0.65, 6: 0.90, 7: 0.15}
    risks = tuple(collision_item(
        track_id, distances[track_id], track_id in (4, 5, 6))
        for track_id in range(1, 8))
    selected, confirmed, consumed = consume_frame(fake, 1, risks)
    assert selected[1].track_id == 4
    assert not confirmed and consumed
    fake.debounce.update.assert_called_once_with(True, 4)


def test_later_iteration_track_still_confirms_across_frames():
    fake = collision_confirmation_fake()
    # Track 4 is intentionally last; collision-path distance keeps the
    # existing priority independent of tracker iteration order.
    risks = (collision_item(5, 0.65), collision_item(6, 0.90),
             collision_item(4, 0.40))
    counts = []
    confirmations = []
    for scan_count in (1, 2, 3):
        selected, confirmed, _consumed = consume_frame(
            fake, scan_count, risks)
        assert selected[1].track_id == 4
        counts.append(fake.debounce.count)
        confirmations.append(confirmed)
    assert counts == [1, 2, 3]
    assert confirmations == [False, False, True]


def test_duplicate_planner_ticks_do_not_consume_one_scan_twice():
    fake = collision_confirmation_fake()
    fake.debounce.update = Mock(wraps=fake.debounce.update)
    risk = (collision_item(4, 0.40),)
    for _ in range(3):
        selected, confirmed, consumed = consume_frame(fake, 1, risk)
        assert selected[1].track_id == 4
        assert not confirmed
    assert fake.debounce.count == 1
    assert fake.debounce.update.call_count == 1
    assert consumed is False


def test_selected_track_change_restarts_confirmation():
    fake = collision_confirmation_fake()
    consume_frame(fake, 1, (collision_item(4, 0.40),))
    consume_frame(fake, 2, (collision_item(4, 0.40),))
    assert fake.debounce.count == 2
    selected, confirmed, consumed = consume_frame(
        fake, 3, (collision_item(6, 0.30),))
    assert selected[1].track_id == 6
    assert not confirmed and consumed
    assert fake.debounce.track_id == 6
    assert fake.debounce.count == 1


def test_no_trigger_resets_debounce_once_per_fresh_front_frame():
    fake = collision_confirmation_fake()
    consume_frame(fake, 1, (collision_item(4, 0.40),))
    fake.debounce.update = Mock(wraps=fake.debounce.update)
    outside_trigger = (collision_item(4, 2.10),)
    selected, confirmed, consumed = consume_frame(fake, 2, outside_trigger)
    assert selected is None
    assert not confirmed and consumed
    assert fake.debounce.count == 0
    fake.debounce.update.assert_called_once_with(False)
    consume_frame(fake, 2, outside_trigger)
    assert fake.debounce.update.call_count == 1


def test_rear_scan_does_not_advance_front_collision_frame_count():
    rear = SimpleNamespace(header=SimpleNamespace(frame_id='rear_laser'))
    fake = SimpleNamespace(
        pending_scans=deque(((rear, False, time.perf_counter()),)),
        p={'tf_lookup_timeout_sec': 5.0, 'rear_lidar_required': True},
        tf_ready_frames=set(), front_tf_ready=True, rear_tf_ready=False,
        tf_ready=False, scan_drops=0, superseded_scan_drops=0,
        scan_count=9, _lookup_transform=Mock(return_value=(object(), '', '')),
        _process_scan=Mock(), _record_tf_drop=Mock())
    AvoidanceCoordinator._drain_scan_queue(fake)
    fake._process_scan.assert_not_called()
    assert fake.scan_count == 9
    assert fake.rear_tf_ready


def test_planner_status_exposes_collision_frame_diagnostics():
    source = __import__('inspect').getsource(
        AvoidanceCoordinator._publish_status)
    assert 'scan_count' in source
    assert 'last_collision_evaluation_scan_count' in source
    assert 'collision_confirmation_count' in source
    assert 'collision_confirmation_track_id' in source
    assert 'selected_track_id' in source


def test_required_planner_states_are_present():
    required = {
        'FOLLOWING_CSV', 'OBSTACLE_CANDIDATE', 'OBSTACLE_CONFIRMED',
        'REPLAN_REQUIRED', 'STOPPING', 'STOPPED_FOR_PLANNING',
        'DETECTING_BOUNDARIES', 'BUILDING_CORRIDOR',
        'GENERATING_CANDIDATES', 'VALIDATING_CANDIDATES', 'PATH_READY',
        'PATH_INFEASIBLE', 'DYNAMIC_OBSTACLE_STOP', 'TIME_RESET_STOP', 'ERROR'}
    assert set(PLANNER_STATES) == required


def test_planner_latches_replan_but_does_not_compete_for_mcu_outputs():
    fake = SimpleNamespace(
        debounce=SimpleNamespace(latched=True), state='PATH_READY',
        replan_pub=Mock())
    AvoidanceCoordinator._publish_stop_contract(fake)
    assert fake.replan_pub.publish.call_args.args[0].data is True


def test_planner_has_no_vehicle_command_publisher_contract():
    source = __import__('inspect').getsource(AvoidanceCoordinator.__init__)
    assert '/lidar_drive' not in source
    assert '/lidar_wheel' not in source


def test_fixed_environment_filters_apparent_dynamic_surface_tracks():
    source = __import__('inspect').getsource(AvoidanceCoordinator._process_scan)
    assert "if self.p['fixed_environment_mode']" in source
    assert 'if track.state == DYNAMIC_OBSTACLE' in source
    assert 'self.fixed_matched_track_ids.add(track.track_id)' in source
    assert 'track.state = STATIC_OBSTACLE' in source


def test_dynamic_to_static_transition_cannot_return_to_latched_csv_deadlock():
    source = __import__('inspect').getsource(AvoidanceCoordinator._tick)
    stationary_block = source.split(
        "self.selected_track.state == STATIC_OBSTACLE):", 1)[1].split(
            "if self.state in ('PATH_READY'", 1)[0]
    assert "self._set_state('REPLAN_REQUIRED'" in stationary_block
    assert "self._set_state('STOPPING'" in stationary_block
    assert "self._set_state('FOLLOWING_CSV'" not in stationary_block


def test_confirmed_path_intersection_is_latched_until_track_is_passed():
    source = __import__('inspect').getsource(AvoidanceCoordinator._tick)
    assert 'self.path_relevant_track_ids.add(track.track_id)' in source
    assert "'LATCHED_COLLISION_RISK'" in source


def test_rejoin_extension_gives_rate_limited_follower_settling_distance():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config' /
                             'avoidance_planner.yaml').read_text())
    assert config['avoidance_coordinator']['ros__parameters'][
        'rejoin_straight_extension_m'] >= 2.0


def test_tf_drop_diagnostics_separate_startup_and_runtime():
    fake = SimpleNamespace(
        scan_drops=0, tf_failures=0, last_tf_error='', tf_ready=False,
        startup_tf_drop_count=0, runtime_tf_drop_count=0,
        future_extrapolation_count=0, past_extrapolation_count=0,
        missing_frame_count=0)
    AvoidanceCoordinator._record_tf_drop(fake, 'future extrapolation', 'future')
    assert fake.startup_tf_drop_count == 1
    assert fake.runtime_tf_drop_count == 0
    assert fake.future_extrapolation_count == 1
    fake.tf_ready = True
    AvoidanceCoordinator._record_tf_drop(fake, 'past extrapolation', 'past')
    assert fake.runtime_tf_drop_count == 1
    assert fake.past_extrapolation_count == 1
    assert fake.scan_drops == fake.tf_failures == 2


def test_max_steering_deg_twenty_two_is_accepted():
    fake = SimpleNamespace(p={'wheelbase_m': 0.73, 'max_steering_deg': 22.0,
                              'planning_roi_x_min_m': -7.0,
                              'planning_roi_x_max_m': -0.1,
                              'planning_roi_half_width_m': 1.3,
                              'left_curb_inner_y_m': 1.095,
                              'right_curb_inner_y_m': -1.095})
    AvoidanceCoordinator._validate_parameters(fake)


def test_max_steering_deg_above_twenty_two_is_rejected():
    fake = SimpleNamespace(p={'wheelbase_m': 0.73, 'max_steering_deg': 22.1,
                              'planning_roi_x_min_m': -7.0,
                              'planning_roi_x_max_m': -0.1,
                              'planning_roi_half_width_m': 1.3,
                              'left_curb_inner_y_m': 1.095,
                              'right_curb_inner_y_m': -1.095})
    try:
        AvoidanceCoordinator._validate_parameters(fake)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_watchdog_runs_before_expensive_scan_queue_drain():
    calls = []
    fake = SimpleNamespace(
        last_tick_time=None, debounce=SimpleNamespace(count=0), active=False,
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=Mock()))),
        _publish_stop_contract=Mock(side_effect=lambda: calls.append('stop')),
        _watchdog=Mock(side_effect=lambda: calls.append('watchdog')),
        _drain_scan_queue=Mock(side_effect=lambda: calls.append('drain')),
        _update_cpu_usage=Mock(side_effect=lambda: calls.append('cpu')),
        _mode_allowed=Mock(return_value=False), _publish_status=Mock())
    AvoidanceCoordinator._tick(fake)
    assert calls[:4] == ['stop', 'watchdog', 'drain', 'cpu']


def test_successful_slow_scan_processing_cannot_timeout_itself():
    from rclpy.time import Time
    now = Time(nanoseconds=2_000_000_000)
    fake = SimpleNamespace(
        active=True, state='FOLLOWING_CSV', odom_receive_time=now,
        reference_receive_time=now,
        last_front_scan_time=Time(nanoseconds=1_000_000_000),
        last_front_scan_processed_time=now,
        scan_drops=0, debounce=SimpleNamespace(update=Mock()),
        selected_track=object(), operational_state='MONITORING',
        p={'odom_timeout_sec': 0.5, 'reference_path_timeout_sec': 0.0,
           'scan_timeout_sec': 0.3},
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=now))),
        _set_state=Mock())
    AvoidanceCoordinator._watchdog(fake)
    fake._set_state.assert_not_called()
    assert fake.scan_drops == 0


def test_latest_transformable_scan_supersedes_stale_past_scan_without_error():
    old = SimpleNamespace(header=SimpleNamespace(frame_id='front_laser'))
    fresh = SimpleNamespace(header=SimpleNamespace(frame_id='front_laser'))
    transform = object()
    fake = SimpleNamespace(
        pending_scans=deque(((old, True, time.perf_counter()-10.0),
                             (fresh, True, time.perf_counter()))),
        p={'tf_lookup_timeout_sec': 5.0, 'rear_lidar_required': False},
        tf_ready_frames=set(), front_tf_ready=False, rear_tf_ready=False,
        tf_ready=False, scan_drops=0, superseded_scan_drops=0,
        _lookup_transform=Mock(side_effect=lambda scan:
            (transform, '', '') if scan is fresh else
            (None, 'past', 'extrapolation into the past')),
        _process_scan=Mock(), _record_tf_drop=Mock())
    AvoidanceCoordinator._drain_scan_queue(fake)
    fake._process_scan.assert_called_once_with(fresh, transform)
    fake._record_tf_drop.assert_not_called()
    assert fake.tf_ready
    assert fake.superseded_scan_drops == 1
    assert not fake.pending_scans


def test_continuous_transformable_front_batches_keep_advancing_scan_count():
    fake = SimpleNamespace(
        pending_scans=deque(),
        p={'tf_lookup_timeout_sec': 5.0, 'rear_lidar_required': False},
        tf_ready_frames=set(), front_tf_ready=False, rear_tf_ready=False,
        tf_ready=False, scan_drops=0, superseded_scan_drops=0,
        scan_count=0, _lookup_transform=Mock(
            return_value=(object(), '', '')), _record_tf_drop=Mock())

    def process(_scan, _transform):
        fake.scan_count += 1

    fake._process_scan = Mock(side_effect=process)
    observed = []
    for batch in range(3):
        for index in range(3):
            scan = SimpleNamespace(header=SimpleNamespace(
                frame_id='front_laser', stamp=(batch, index)))
            fake.pending_scans.append((scan, True, time.perf_counter()))
        AvoidanceCoordinator._drain_scan_queue(fake)
        observed.append(fake.scan_count)
    assert observed == [1, 2, 3]
    assert fake._process_scan.call_count == 3
    assert fake.superseded_scan_drops == 6
    assert fake.tf_ready


def test_production_and_lifted_bench_boundary_profiles_are_separate():
    config_dir = Path(__file__).parents[1] / 'config'
    production = yaml.safe_load(
        (config_dir / 'avoidance_planner.yaml').read_text())
    bench = yaml.safe_load(
        (config_dir / 'avoidance_planner_bench.yaml').read_text())
    production_p = production['avoidance_coordinator']['ros__parameters']
    bench_p = bench['avoidance_coordinator']['ros__parameters']
    assert production_p['left_curb_inner_y_m'] == 1.095
    assert production_p['right_curb_inner_y_m'] == -1.095
    assert bench_p['left_curb_inner_y_m'] == 1.50
    assert bench_p['right_curb_inner_y_m'] == -1.50
    assert production_p['replan_trigger_distance_m'] == 2.0
    assert production_p['planning_roi_x_min_m'] == -7.0
    assert production_p['planning_roi_x_max_m'] == -0.1
    assert production_p['planning_roi_half_width_m'] == 1.3
    assert production_p['debug_visualization'] is False


def test_replan_trigger_accessor_reads_runtime_ros_parameter():
    fake = SimpleNamespace(get_parameter=Mock(return_value=SimpleNamespace(
        value=3.5)))
    assert AvoidanceCoordinator._replan_trigger_distance(fake) == 3.5
    fake.get_parameter.assert_called_once_with('replan_trigger_distance_m')


def test_path_infeasible_reason_uses_runtime_steering_parameter():
    source = __import__('inspect').getsource(
        AvoidanceCoordinator._advance_planning)
    assert "float(self.p['max_steering_deg'])" in source
    assert '<=25 deg' not in source


def test_completed_track_is_marked_passed_and_excluded_from_status():
    fake = SimpleNamespace(
        selected_track=SimpleNamespace(track_id=7), avoidance_started=True,
        debounce=SimpleNamespace(), replan_pub=Mock(),
        tracker=SimpleNamespace(reset_epoch=Mock()),
        tracks=(), walls=(), unknown=(), wall_hits=0,
        last_track_decisions=[],
        p={'confirmation_frames': 3, 'fixed_environment_mode': False},
        passed_track_ids=set(), passed_obstacle_s=[],
        get_logger=Mock(return_value=Mock()), _set_state=Mock())
    AvoidanceCoordinator._control_source(fake, SimpleNamespace(data='GPS'))
    assert fake.passed_track_ids == {7}
    fake.tracker.reset_epoch.assert_called_once_with()


def test_new_lidar_face_of_passed_physical_obstacle_is_suppressed():
    fake = SimpleNamespace(passed_obstacle_s=[8.0])
    assert AvoidanceCoordinator._is_passed_obstacle_face(fake, 8.9)
    assert not AvoidanceCoordinator._is_passed_obstacle_face(fake, 10.0)


def test_raw_route_boundary_points_are_not_tracked_by_chord_centroid():
    from avoidance_planner.geometry import Pose2
    from avoidance_planner.perception import Detection, ScanPoint
    route = tuple(Pose2(index*0.1, 0.0, 0.0) for index in range(101))
    points = tuple(ScanPoint(index, 3.0+index*0.01, 1.50, 3.0)
                   for index in range(80))
    # A curved chord centroid can project well inside the road even though
    # every source return lies on the known curb band.
    detection = Detection(3.2, 0.55, 3.0, 3.4, 0.5, 1.5,
                          len(points), 0.40, 0.40, points=points)
    fake = SimpleNamespace(
        route=route, route_nearest_index=0,
        p={'left_curb_inner_y_m': 1.50,
           'right_curb_inner_y_m': -1.50,
           'fixed_environment_mode': False})
    walls, kept = AvoidanceCoordinator._classify_curved_boundaries(
        fake, (), (detection,))
    assert len(walls) == 1
    assert kept == ()


def test_wide_obstacle_face_is_not_absorbed_into_raw_curb_band():
    from avoidance_planner.geometry import Pose2
    from avoidance_planner.perception import Detection, ScanPoint
    route = tuple(Pose2(index*0.1, 0.0, 0.0) for index in range(101))
    points = tuple(ScanPoint(index, 3.0+index*0.01, -1.50, 3.0)
                   for index in range(80))
    detection = Detection(3.4, -1.50, 3.0, 3.8, -1.55, -1.45,
                          len(points), 0.78, 0.10, points=points)
    fake = SimpleNamespace(
        route=route, route_nearest_index=0,
        p={'left_curb_inner_y_m': 1.50,
           'right_curb_inner_y_m': -1.50,
           'fixed_environment_mode': False})
    walls, kept = AvoidanceCoordinator._classify_curved_boundaries(
        fake, (), (detection,))
    assert walls == ()
    assert kept == (detection,)


def test_obstacle_wholly_beyond_goal_is_not_clamped_to_last_csv_pose():
    from avoidance_planner.geometry import Pose2
    fake = SimpleNamespace(
        route=(Pose2(0.0, 0.0, 0.0), Pose2(10.0, 0.0, 0.0)),
        p={'vehicle_length_m': 1.30, 'minimum_obstacle_depth_m': 0.65,
           'obstacle_safety_longitudinal_m': 0.15})
    assert AvoidanceCoordinator._is_beyond_route_goal(
        fake, SimpleNamespace(x=11.20, y=0.0))
    assert not AvoidanceCoordinator._is_beyond_route_goal(
        fake, SimpleNamespace(x=11.00, y=0.0))


def test_obstacle_statuses_label_passed_track_and_order_by_x():
    track_a = SimpleNamespace(track_id=1, x=2.0, state='STATIC_OBSTACLE')
    track_b = SimpleNamespace(track_id=2, x=8.0, state='STATIC_OBSTACLE')
    fake = SimpleNamespace(
        tracks=(track_a, track_b), selected_track=track_b,
        avoidance_started=True, passed_track_ids={1})
    from avoidance_planner.perception import STATIC_OBSTACLE  # noqa: F401
    statuses = AvoidanceCoordinator._obstacle_statuses(fake)
    by_id = {entry['track_id']: entry for entry in statuses}
    assert by_id[1]['status'] == 'PASSED' and by_id[1]['label'] == 'obstacle_1'
    assert by_id[2]['status'] == 'AVOIDING' and by_id[2]['label'] == 'obstacle_2'


def _debug_marker(namespace, marker_id, marker_type, stamp=None):
    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.header.stamp = stamp if stamp is not None else marker.header.stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def test_front_laser_points_and_roi_use_exact_mount_transform():
    from geometry_msgs.msg import TransformStamped
    from avoidance_planner.perception import ScanPoint

    transform = TransformStamped()
    transform.transform.translation.x = 0.730
    transform.transform.translation.z = 0.105
    transform.transform.rotation.z = 1.0  # yaw pi
    transform.transform.rotation.w = 0.0
    fake = SimpleNamespace(_yaw=AvoidanceCoordinator._yaw)
    transformed = AvoidanceCoordinator._transform_points(
        fake, (ScanPoint(0, -3.0, 0.40, 3.0),), transform)
    assert transformed[0].x == pytest.approx(3.730)
    assert transformed[0].y == pytest.approx(-0.40)

    fake = SimpleNamespace(
        debug_visualization=True,
        p={'planning_roi_x_min_m': -7.0,
           'planning_roi_x_max_m': -0.1,
           'planning_roi_half_width_m': 1.3,
           'publish_rejected_points': True},
        debug_roi_pub=Mock(), debug_roi_points_pub=Mock(),
        debug_rejected_points_pub=Mock(), _marker=_debug_marker,
        _yaw=AvoidanceCoordinator._yaw)
    fake._transform_points = lambda points, tf: (
        AvoidanceCoordinator._transform_points(fake, points, tf))
    AvoidanceCoordinator._publish_debug_scan(
        fake, transform.header.stamp, transform, transformed, ())
    roi = fake.debug_roi_pub.publish.call_args.args[0]
    assert min(point.x for point in roi.points) == pytest.approx(0.830)
    assert max(point.x for point in roi.points) == pytest.approx(7.730)
    assert min(point.y for point in roi.points) == pytest.approx(-1.30)
    assert max(point.y for point in roi.points) == pytest.approx(1.30)
    assert roi.header.frame_id == 'odom'


def test_selected_track_marker_uses_track_id_and_clears_when_missing():
    track = SimpleNamespace(
        track_id=6, state='STATIC_OBSTACLE', hits=5,
        x=3.0, y=0.7, min_x=2.9, max_x=3.1,
        min_y=0.5, max_y=0.9)
    state_token = object()
    fake = SimpleNamespace(
        debug_visualization=True, debug_perception_valid=True,
        tracks=(track,), selected_track=track,
        last_track_decisions=[{
            'track_id': 6, 'lidar_surface_distance_m': 2.31,
            's': 3.05}], debug_obstacles_pub=Mock(),
        debug_selected_obstacle_pub=Mock(), _marker=_debug_marker,
        _clear_array=AvoidanceCoordinator._clear_array,
        state=state_token)
    AvoidanceCoordinator._publish_debug_obstacles(fake)
    selected = fake.debug_selected_obstacle_pub.publish.call_args.args[0]
    assert selected.markers[-1].id == 6
    labels = fake.debug_obstacles_pub.publish.call_args.args[0].markers
    assert any('track=6' in marker.text for marker in labels)
    assert fake.state is state_token

    fake.tracks = ()
    AvoidanceCoordinator._publish_debug_obstacles(fake)
    cleared = fake.debug_selected_obstacle_pub.publish.call_args.args[0]
    assert len(cleared.markers) == 1
    assert cleared.markers[0].action == Marker.DELETEALL


def test_stale_scan_or_tf_failure_clears_debug_obstacle_state():
    stale_track = SimpleNamespace(
        track_id=4, state='STATIC_OBSTACLE', hits=5,
        x=2.0, y=0.5, min_x=1.9, max_x=2.1,
        min_y=0.3, max_y=0.7)
    fake = SimpleNamespace(
        debug_visualization=True, debug_roi_pub=Mock(),
        debug_roi_points_pub=Mock(), debug_rejected_points_pub=Mock(),
        debug_obstacles_pub=Mock(), debug_selected_obstacle_pub=Mock(),
        debug_collision_pub=Mock(), last_collision_debug=object(),
        tracks=(stale_track,), selected_track=stale_track,
        last_track_decisions=[], _marker=_debug_marker,
        _clear_array=AvoidanceCoordinator._clear_array)
    AvoidanceCoordinator._clear_debug_perception(fake)
    assert fake.last_collision_debug is None
    assert not fake.debug_perception_valid
    for publisher in (
            fake.debug_roi_pub, fake.debug_roi_points_pub,
            fake.debug_rejected_points_pub):
        assert publisher.publish.call_args.args[0].action == Marker.DELETE
    for publisher in (
            fake.debug_obstacles_pub, fake.debug_selected_obstacle_pub,
            fake.debug_collision_pub):
        assert publisher.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    AvoidanceCoordinator._publish_debug_obstacles(fake)
    assert len(fake.debug_obstacles_pub.publish.call_args.args[0].markers) == 1


def test_debug_visualization_disabled_has_no_side_effects():
    fake = SimpleNamespace(
        debug_visualization=False, debug_obstacles_pub=Mock(),
        debug_selected_obstacle_pub=Mock(), state='FOLLOWING_CSV',
        selected_track=None, tracks=())
    AvoidanceCoordinator._publish_debug_obstacles(fake)
    fake.debug_obstacles_pub.publish.assert_not_called()
    fake.debug_selected_obstacle_pub.publish.assert_not_called()
    assert fake.state == 'FOLLOWING_CSV'


def test_stale_processed_scan_clears_debug_even_if_scans_keep_arriving():
    from rclpy.time import Time

    now = Time(nanoseconds=2_000_000_000)
    fake = SimpleNamespace(
        active=False, state='PATH_READY', odom_receive_time=now,
        reference_receive_time=now, last_front_scan_time=now,
        last_front_scan_processed_time=Time(nanoseconds=1_000_000_000),
        selected_track=object(), scan_drops=0,
        debounce=SimpleNamespace(update=Mock()),
        p={'odom_timeout_sec': 0.5, 'reference_path_timeout_sec': 0.0,
           'scan_timeout_sec': 0.3}, debug_visualization=True,
        get_clock=Mock(return_value=SimpleNamespace(now=Mock(return_value=now))),
        _clear_debug_perception=Mock(), _set_state=Mock())
    AvoidanceCoordinator._watchdog(fake)
    fake._clear_debug_perception.assert_called_once_with()
    fake._set_state.assert_not_called()


def test_collision_debug_marks_point_projection_and_runtime_trigger():
    from avoidance_planner.geometry import Pose2

    track = SimpleNamespace(track_id=6, x=3.2, y=0.7)
    risk = SimpleNamespace(
        collision_path_index=3, nearest_path_index=0,
        collision_path_distance=3.0)
    fake = SimpleNamespace(
        debug_visualization=True, debug_perception_valid=True,
        last_collision_debug=(track, risk),
        route=tuple(Pose2(float(index), 0.0, 0.0) for index in range(6)),
        debug_collision_pub=Mock(), _marker=_debug_marker,
        _clear_array=AvoidanceCoordinator._clear_array,
        _replan_trigger_distance=Mock(return_value=3.5))
    AvoidanceCoordinator._publish_debug_collision(fake)
    markers = fake.debug_collision_pub.publish.call_args.args[0].markers
    point = next(marker for marker in markers
                 if marker.ns == 'debug_collision_point')
    trigger = next(marker for marker in markers
                   if marker.ns == 'debug_replan_trigger')
    label = next(marker for marker in markers
                 if marker.ns == 'debug_collision_text')
    assert point.pose.position.x == 3.0
    assert [item.x for item in trigger.points] == [
        0.0, 1.0, 2.0, 3.0, 3.5]
    assert 'track=6' in label.text
    assert 'trigger=3.50m' in label.text


def test_candidate_markers_are_debug_only_and_include_decision_fields():
    source = __import__('inspect').getsource(AvoidanceCoordinator._publish_plan)
    assert 'if not self.debug_visualization' in source
    assert "'candidate_labels'" in source
    assert 'candidate.reason' in source
    assert 'candidate.max_steering_rad' in source

from dataclasses import replace

import pytest

from race_control.route_waypoints import Waypoint
from race_control.fused_route import RouteGeometry, FusedProgress, WaypointSchedule
from race_control.course_mission import CourseMission, MissionInput


def route():
    return RouteGeometry([Waypoint(i, 2 if i < 10 else 8 if i < 20 else 9,
        '', '', float(i), 0., 37., 127., i in (3, 7, 15, 18)) for i in range(30)])


def anchor(f):
    for t in (0., .1, .2):
        f.predict(0., t)
        assert f.correct(0., 0., .01, t)


def test_gps_odom_fusion_gates_and_bounded_outage():
    f = FusedProgress(route())
    anchor(f)
    assert f.validity(.2)[0]
    f.predict(.5, .3)
    assert f.s == pytest.approx(.5)
    assert not f.correct(20., 0., .01, .3)
    assert f.s == pytest.approx(.5)
    assert f.correct(.6, 0., .01, .4)
    assert .5 < f.s < .6
    f.predict(.5, 1.)
    assert f.validity(1.)[0]
    f.predict(0., 4.)
    assert not f.validity(4.)[0]


@pytest.mark.parametrize('x,y,var', [(0, 5, .01), (0, 0, 4), (float('nan'), 0, .01)])
def test_bad_gps_does_not_refresh_anchor(x, y, var):
    f = FusedProgress(route())
    anchor(f)
    assert not f.correct(x, y, var, .25)
    assert f.last_gps == .2


def test_odom_reset_and_stale_input_inhibit():
    f = FusedProgress(route())
    anchor(f)
    assert not f.validity(1.)[0]
    assert not f.predict(2., 1.)
    f.predict(0., 1.1)
    assert not f.validity(1.1)[0]


def test_drift_distance_limit_and_rollback():
    f = FusedProgress(route())
    anchor(f)
    for i in range(3):
        f.predict(.8, .3+i*.1)
    assert not f.validity(.5)[0]
    f = FusedProgress(route())
    anchor(f)
    f.predict(.5, .3)
    f.predict(-.2, .4)
    assert f.s == pytest.approx(.3)


def test_first_ramp_ignored_second_offset_and_each_intersection():
    s = WaypointSchedule(route(), ramp_first_index=3, ramp_stop_index=7,
                         acceleration_start_index=21, acceleration_end_index=27)
    state = s.snapshot(3.)
    assert state['stop_index'] == 7  # never the first line
    assert state['stop_remaining_m'] == 3.
    assert s.snapshot(6.)['stop_remaining_m'] == 0.
    assert not s.release(3)
    assert s.release(7)
    assert s.snapshot(13.)['stop_index'] == 15
    assert s.snapshot(13.)['stop_remaining_m'] == 0.
    assert s.release(15)
    assert s.snapshot(16.)['stop_index'] == 18
    assert s.release(18)
    assert not s.snapshot(20.)['acceleration_active']
    assert s.snapshot(21.)['acceleration_active']
    assert not s.snapshot(27.)['acceleration_active']


def test_missing_waypoint_configuration_is_not_guessed():
    s = WaypointSchedule(route())
    assert not s.snapshot(1.)['ramp_configured']
    assert not s.snapshot(21.)['acceleration_configured']
    with pytest.raises(ValueError):
        WaypointSchedule(route(), ramp_first_index=15, ramp_stop_index=7)
    with pytest.raises(ValueError):
        WaypointSchedule(route(), acceleration_start_index=21)


def test_front_offset_and_skipping_unserved_stop():
    s = WaypointSchedule(route(), ramp_first_index=3, ramp_stop_index=7, front_offset_m=.5)
    assert s.snapshot(5.5)['stop_remaining_m'] == 0.
    assert s.snapshot(12.)['section'] == 2  # cannot jump past unserved ramp


def data(**kwargs):
    values = dict(section=8, waypoint_valid=True, waypoint_event='session:15',
                  waypoint_stop_section=8, waypoint_remaining_m=0.,
                  speed_valid=True, speed_mps=0., camera_path_valid=True,
                  speed_plan_valid=True, planned_drive_stage=2)
    values.update(kwargs)
    return MissionInput(**values)


def test_signal_window_only_after_actual_stop_and_rearm_second_line():
    m = CourseMission()
    assert m.update_waypoint(data(speed_mps=1.)).stage == 0
    assert not m.signal_window
    assert m.update_waypoint(data()).stage == 0
    assert m.signal_window.startswith('session:15/vote/')
    assert m.update_waypoint(data(traffic_green=True)).stage == 0  # left required
    assert m.update_waypoint(data(traffic_left=True, confirmed_signal_window=m.signal_window)).stage == 1
    assert m.released_event == 'session:15'
    assert m.update_waypoint(data(waypoint_event='session:18')).stage == 0
    assert m.signal_window.startswith('session:18/vote/')


def test_ramp_hold_starts_when_stopped_and_camera_line_is_ignored():
    m = CourseMission()
    d = data(section=2, waypoint_stop_section=2, waypoint_event='session:7',
             odom_distance_valid=True, imu_valid=True)
    assert m.update_waypoint(replace(d, speed_mps=.5, now=0.)).status == 'WAYPOINT:BRAKING'
    assert m.update_waypoint(replace(d, now=2.)).status == 'RAMP:WAYPOINT_STOP_HOLD'
    assert not m.released_event
    assert m.update_waypoint(replace(d, now=4.9)).stage == 0
    m.update_waypoint(replace(d, now=5.))
    assert m.released_event == 'session:7'
    assert m.update_waypoint(replace(d, now=6., stop_detected=True,
                             stop_distance_valid=True, stop_distance_m=0.)).stage > 0


def test_acceleration_uses_waypoints_not_signs_and_respects_speed_plan():
    m = CourseMission()
    d = data(section=9, waypoint_event='', waypoint_stop_section=-1)
    assert m.update_waypoint(replace(d, traffic20_detected=True)).stage == 2
    assert m.update_waypoint(replace(d, acceleration_active=True)).stage == 3
    assert m.update_waypoint(replace(d, acceleration_active=True, planned_drive_stage=0)).stage == 0
    assert m.update_waypoint(replace(d, acceleration_active=True, speed_plan_valid=False)).stage == 0


def test_target_position_error_and_missing_speed_inhibit():
    m = CourseMission()
    assert m.update_waypoint(data(waypoint_remaining_m=-.5)).status == 'WAYPOINT:STOP_POSITION_ERROR'
    assert m.update_waypoint(data(speed_valid=False)).stage == 0
    assert not m.signal_window


def test_invalid_progress_resets_stop_hold():
    m = CourseMission()
    d = data(section=2, waypoint_stop_section=2)
    m.update_waypoint(d)
    m.update_waypoint(replace(d, now=2., waypoint_valid=False))
    assert m.update_waypoint(replace(d, now=10.)).stage == 0
    assert not m.released_event


def test_signal_vote_must_restart_after_motion_or_input_loss():
    m = CourseMission()
    d = data()
    m.update_waypoint(d)
    old_window = m.signal_window
    m.update_waypoint(replace(d, speed_mps=1.))
    assert m.update_waypoint(replace(d, traffic_left=True,
                             confirmed_signal_window=old_window)).stage == 0
    assert m.signal_window != old_window

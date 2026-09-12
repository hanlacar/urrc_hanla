from race_control.course_mission import CourseMission, MissionInput


def step(logic, t, distance, pitch=4.5, **kwargs):
    data = dict(section=2, now=t, odom_distance_m=distance,
                odom_distance_valid=True, imu_valid=True, pitch_deg=pitch)
    data.update(kwargs)
    return logic.update(MissionInput(**data))


def test_distance_then_continuous_pitch_and_latch():
    m = CourseMission()
    step(m, 0, 10)
    step(m, 2, 10.49)
    assert m.ramp_pitch_candidate_time is None
    step(m, 3, 10.5)
    step(m, 3.99, 10.5)
    assert not m.ramp_crossing
    step(m, 4, 10.5)
    assert m.ramp_crossing
    step(m, 5, 10.6, pitch=4.3)
    assert m.ramp_crossing


def test_invalid_imu_restart_timer():
    for invalid in [dict(imu_valid=False), dict(pitch=float('nan'))]:
        m = CourseMission()
        step(m, 0, 0)
        step(m, 1, .5)
        step(m, 1.5, .5, **invalid)
        step(m, 2, .5)
        step(m, 2.99, .5)
        assert not m.ramp_crossing
        step(m, 3, .5)
        assert m.ramp_crossing


def test_odom_loss_and_section_reentry_rebase():
    m = CourseMission()
    step(m, 0, 0)
    step(m, 1, .5)
    step(m, 1.5, .5, odom_distance_valid=False)
    step(m, 2, 2)
    step(m, 4, 2.49)
    assert not m.ramp_crossing
    step(m, 5, 2.5)
    step(m, 6, 2.5)
    assert m.ramp_crossing
    step(m, 7, 3, section=3)
    step(m, 8, 4)
    assert not m.ramp_crossing
    assert m.ramp_entry_odom_m == 4


def test_camera_stop_line_does_not_stop_ramp():
    m = CourseMission(ramp_delay_sec=0)
    step(m, 0, 0)
    step(m, 1, .5)
    step(m, 2, .5)
    for t, distance, visible in [(3, 1, True), (4, 2, False),
                                  (5, 3, False), (6, 4, True)]:
        result = step(m, t, distance, camera_path_valid=True,
                      stop_detected=visible, stop_distance_valid=True,
                      stop_distance_m=.1)
        assert result.stage > 0
        assert not m.ramp_second_line_stopped


def test_dr_stop_before_confirmation_is_latched_and_not_retriggered():
    m = CourseMission()
    result = step(m, 0, 0, ramp_dr_stop_reached=True,
                  imu_valid=False, odom_distance_valid=False)
    assert result.stage == 0
    assert 'STOP_LINE_STOP' in result.status
    assert not m.ramp_crossing
    result = step(m, 2.99, 0, camera_path_valid=True)
    assert result.stage == 0
    result = step(m, 3, 0, camera_path_valid=True)
    assert m.ramp_second_line_completed
    assert not m.ramp_crossing
    step(m, 4, .5)
    step(m, 5, .5)
    assert m.ramp_crossing
    result = step(m, 6, .6, ramp_dr_stop_reached=True, camera_path_valid=True)
    assert result.stage > 0
    assert m.ramp_second_line_stop_start == 0
    step(m, 7, 1, section=3)
    step(m, 8, 1)
    assert not m.ramp_second_line_stopped

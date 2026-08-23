from race_control.course_mission import (
    CAMERA, CourseMission as _CourseMission, MissionInput, section_after_ramp_detection,
    traffic20_drive_stage,
)
CourseMission = _CourseMission


def data(section, now=0.0, **kwargs):
    values = dict(section=section, now=now, camera_path_valid=True, camera_steering_deg=4.0)
    values.update(kwargs)
    return MissionInput(**values)


def test_valid_five_degree_pitch_transitions_start_to_ramp_only():
    assert section_after_ramp_detection(1, True, 4.99) == 1
    assert section_after_ramp_detection(1, False, 5.0) == 1
    assert section_after_ramp_detection(1, True, 5.0) == 2
    assert section_after_ramp_detection(1, True, 8.0) == 2
    assert section_after_ramp_detection(3, True, 8.0) == 3


def test_non_ramp_camera_sections_are_limited_to_stage_one():
    logic = CourseMission()
    for section in (1, 3, 5, 12):
        out = logic.update(data(section))
        assert (out.stage, out.steering_deg, out.control_mode) == (1, 4.0, CAMERA)


def test_section_five_uses_pure_pursuit_camera_steering():
    logic = CourseMission()
    output = logic.update(data(5, camera_steering_deg=-17.4,
                               planned_drive_stage=2))
    assert (output.stage, output.steering_deg, output.control_mode) == (
        1, -17.4, CAMERA)
    assert output.status == "S_CURVE:PURE_PURSUIT"


def test_ramp_stops_aligns_then_drives_straight_and_reacquires_path():
    logic = CourseMission()
    stopped = logic.update(data(2, 10.0, imu_valid=True, pitch_deg=10.0))
    assert (stopped.stage, stopped.steering_deg) == (0, 0.0)
    straight = logic.update(data(2, 10.5, imu_valid=True, pitch_deg=11.0,
                                 camera_path_valid=False))
    assert (straight.stage, straight.steering_deg) == (2, 0.0)
    confirming = logic.update(data(2, 11.0, imu_valid=True, pitch_deg=2.5))
    assert (confirming.stage, confirming.steering_deg) == (1, 0.0)
    resumed = logic.update(data(2, 12.0, imu_valid=True, pitch_deg=2.5))
    assert (resumed.stage, resumed.steering_deg) == (1, 4.0)


def test_ramp_stops_and_centers_wheels_at_exactly_five_degrees():
    logic = CourseMission(ramp_pitch_deg=5.0)
    output = logic.update(data(2, 1.0, imu_valid=True, pitch_deg=5.0,
                               camera_path_valid=True, camera_steering_deg=18.0))
    assert output.stage == 0
    assert output.steering_deg == 0.0
    assert output.status == "RAMP:ALIGN_WHEELS"


def test_ramp_pitch_ten_must_hold_one_second_before_stage_one():
    logic = CourseMission(ramp_slow_pitch_deg=10.0, ramp_slow_hold_sec=1.0)
    logic.update(data(2, 0.0, imu_valid=True, pitch_deg=5.0))
    before = logic.update(data(2, 0.5, imu_valid=True, pitch_deg=10.0))
    assert (before.stage, before.steering_deg) == (2, 0.0)
    still_before = logic.update(data(2, 1.49, imu_valid=True, pitch_deg=10.0))
    assert still_before.stage == 2
    slowed = logic.update(data(2, 1.5, imu_valid=True, pitch_deg=10.0))
    assert (slowed.stage, slowed.steering_deg) == (1, 0.0)
    assert slowed.status == "RAMP:PITCH_10_STRAIGHT_LATCHED"


def test_ramp_slow_straight_latches_until_level_then_reacquires_path():
    logic = CourseMission(ramp_slow_pitch_deg=10.0, ramp_slow_hold_sec=1.0)
    logic.update(data(2, 0.0, imu_valid=True, pitch_deg=5.0))
    logic.update(data(2, 0.5, imu_valid=True, pitch_deg=10.0))
    latched = logic.update(data(2, 1.5, imu_valid=True, pitch_deg=10.0,
                                camera_steering_deg=20.0))
    assert (latched.stage, latched.steering_deg) == (1, 0.0)
    descending = logic.update(data(2, 2.0, imu_valid=True, pitch_deg=6.0,
                                   camera_steering_deg=20.0))
    assert (descending.stage, descending.steering_deg) == (1, 0.0)
    confirming = logic.update(data(2, 2.5, imu_valid=True, pitch_deg=3.0,
                                   camera_path_valid=True, camera_steering_deg=4.0))
    assert (confirming.stage, confirming.steering_deg) == (1, 0.0)
    assert confirming.status == "RAMP:LEVEL_CONFIRM_1SEC"
    level = logic.update(data(2, 3.5, imu_valid=True, pitch_deg=3.0,
                              camera_path_valid=True, camera_steering_deg=4.0))
    assert (level.stage, level.steering_deg) == (1, 4.0)
    assert level.status == "RAMP:PATH_FOLLOW"
    assert logic.section_request == 3


def test_ramp_pitch_ten_hold_resets_when_pitch_drops():
    logic = CourseMission(ramp_slow_pitch_deg=10.0, ramp_slow_hold_sec=1.0)
    logic.update(data(2, 0.0, imu_valid=True, pitch_deg=5.0))
    logic.update(data(2, 0.5, imu_valid=True, pitch_deg=10.0))
    logic.update(data(2, 1.0, imu_valid=True, pitch_deg=9.9))
    assert logic.update(data(2, 1.5, imu_valid=True, pitch_deg=10.0)).stage == 2


def test_ramp_pitch_ten_holds_stage_two_for_three_seconds_without_stop_line():
    logic = CourseMission(ramp_slow_hold_sec=3.0)
    logic.update(data(2, 10.0, imu_valid=True, pitch_deg=10.0))
    held = logic.update(data(2, 12.99, imu_valid=True, pitch_deg=11.0,
                             stop_detected=False, camera_path_valid=False))
    assert (held.stage, held.steering_deg) == (2, 0.0)
    assert held.status == "RAMP:PITCH_10_STAGE_2_HOLD"
    slowed = logic.update(data(2, 13.0, imu_valid=True, pitch_deg=11.0))
    assert (slowed.stage, slowed.steering_deg) == (1, 0.0)


def test_yellow_line_one_meter_stops_then_uses_valid_replanned_path():
    logic = CourseMission()
    stopped = logic.update(data(1, 1.0, yellow_ahead_valid=True, yellow_ahead_m=1.0))
    assert stopped.stage == 0
    assert logic.update(data(1, 1.49, yellow_ahead_valid=True, yellow_ahead_m=0.9)).stage == 0
    resumed = logic.update(data(1, 1.5, yellow_ahead_valid=True, yellow_ahead_m=0.9))
    assert (resumed.stage, resumed.steering_deg) == (1, 4.0)


def test_intersection_stops_half_meter_and_releases_on_green():
    logic = CourseMission(minimum_stop_sec=0.5)
    stopped = logic.update(data(4, 1.0, stop_distance_valid=True, stop_distance_m=0.5))
    assert stopped.stage == 0
    assert logic.update(data(4, 1.4, traffic_green=True)).stage == 0
    assert logic.update(data(4, 1.5, traffic_green=True, gps_direction=-1)).stage == 1


def test_parking_sections_keep_camera_candidate_for_mcu_priority():
    logic = CourseMission()
    out = logic.update(data(7, camera_steering_deg=-8.0))
    assert (out.stage, out.steering_deg, out.control_mode) == (1, -8.0, CAMERA)
    assert out.status == "T_COURSE:CAMERA_STANDBY"
    out = logic.update(data(10, camera_steering_deg=6.0))
    assert (out.stage, out.steering_deg, out.control_mode) == (1, 6.0, CAMERA)
    assert out.status == "PARALLEL_PARK:CAMERA_STANDBY"


def test_traffic20_section_latches_cruise_stage_two_to_three():
    logic = CourseMission()
    assert logic.update(data(9, traffic20_detected=False,
                             planned_drive_stage=2)).stage == 2
    detected = logic.update(data(9, now=1.0, traffic20_detected=True,
                                 planned_drive_stage=2))
    assert detected.stage == 2
    assert detected.status.startswith("TRAFFIC20_CONFIRMING_")
    assert logic.update(data(9, now=2.99, traffic20_detected=True,
                             planned_drive_stage=2)).stage == 2
    confirmed = logic.update(data(9, now=3.0, traffic20_detected=True,
                                  planned_drive_stage=2))
    assert confirmed.stage == 3
    assert confirmed.status == "TRAFFIC20_CONFIRMED:STAGE_3"
    assert logic.update(data(9, traffic20_detected=False,
                             planned_drive_stage=2)).stage == 3


def test_traffic20_confirmation_resets_if_detection_breaks_before_two_seconds():
    logic = CourseMission()
    logic.update(data(9, now=1.0, traffic20_detected=True,
                      planned_drive_stage=2))
    logic.update(data(9, now=2.0, traffic20_detected=False,
                      planned_drive_stage=2))
    assert logic.update(data(9, now=3.0, traffic20_detected=True,
                             planned_drive_stage=2)).stage == 2


def test_traffic20_upgrade_preserves_curvature_slow_and_stop():
    assert traffic20_drive_stage(0, True) == 0
    assert traffic20_drive_stage(1, True) == 1
    assert traffic20_drive_stage(2, True) == 3


def test_curvature_planner_can_slow_or_stop_camera_path_following():
    logic = CourseMission()
    slowed = logic.update(data(1, planned_drive_stage=1))
    assert (slowed.stage, slowed.steering_deg) == (1, 4.0)
    stopped = logic.update(data(1, planned_drive_stage=0))
    assert (stopped.stage, stopped.steering_deg) == (0, 4.0)
    assert stopped.status == "CURVATURE_STOP:START"


def test_invalid_curvature_plan_safe_stops():
    output = CourseMission().update(data(1, speed_plan_valid=False))
    assert output.stage == 0
    assert output.status == "SAFE_STOP:SPEED_PLAN_INVALID"


def test_finish_stop_latches_and_invalid_path_safe_stops():
    logic = CourseMission()
    assert logic.update(data(13, stop_distance_valid=True, stop_distance_m=0.49)).stage == 0
    assert logic.update(data(13)).stage == 0
    assert logic.update(data(1, camera_path_valid=False)).stage == 0

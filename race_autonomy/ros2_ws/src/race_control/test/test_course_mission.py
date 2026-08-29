import pytest

from race_control.course_mission import (
    CAMERA,
    CourseMission as _CourseMission,
    MissionInput,
    camera_emergency_stop,
    section_after_ramp_detection,
    traffic20_drive_stage,
)
CourseMission = _CourseMission


def data(section, now=0.0, **kwargs):
    values = dict(section=section, now=now, camera_path_valid=True, camera_steering_deg=4.0)
    values.update(kwargs)
    return MissionInput(**values)


def test_camera_hard_stop_is_only_for_failure_after_motion():
    status = "SAFE_STOP:INPUT_STREAM_LOST"
    assert not camera_emergency_stop(status, control_was_active=False)
    assert camera_emergency_stop(status, control_was_active=True)


def test_ordinary_stage_zero_reasons_do_not_assert_camera_hard_stop():
    for status in (
        "INTERSECTION_RED_STOP_AT_2M",
        "FINISH_RED_STOP_AT_1M",
        "RAMP:ALIGN_WHEELS",
        "CURVATURE_STOP:START",
        "SAFE_STOP:CAMERA_PATH_INVALID",
        "SAFE_STOP:SPEED_PLAN_INVALID",
    ):
        assert not camera_emergency_stop(status, control_was_active=True)


def test_valid_five_degree_pitch_transitions_start_to_ramp_only():
    assert section_after_ramp_detection(1, True, 4.99) == 1
    assert section_after_ramp_detection(1, False, 5.0) == 1
    assert section_after_ramp_detection(1, True, 5.0) == 2
    assert section_after_ramp_detection(1, True, 8.0) == 2
    assert section_after_ramp_detection(3, True, 8.0) == 3


def test_non_ramp_camera_sections_are_limited_to_stage_one():
    logic = CourseMission()
    for section in (1, 3, 5):
        out = logic.update(data(section))
        assert (out.stage, out.steering_deg, out.control_mode) == (1, 4.0, CAMERA)


def test_missing_gps_section_and_direction_still_starts_with_camera_path():
    logic = CourseMission()
    output = logic.update(MissionInput(
        camera_path_valid=True, camera_steering_deg=-3.0,
        speed_plan_valid=True, planned_drive_stage=1))
    assert (output.stage, output.steering_deg, output.control_mode) == (
        1, -3.0, CAMERA)
    assert output.status == "START"


def test_section_five_uses_pure_pursuit_camera_steering():
    logic = CourseMission()
    output = logic.update(data(5, camera_steering_deg=-17.4,
                               planned_drive_stage=2))
    assert (output.stage, output.steering_deg, output.control_mode) == (
        1, -17.4, CAMERA)
    assert output.status == "S_CURVE:OBSTACLE_YELLOW_CORRIDOR_PATH_FOLLOW"


def test_start_ignores_stop_lines_and_follows_path():
    output=CourseMission().update(data(
        1, imu_valid=True, pitch_deg=4.9, stop_detected=True,
        stop_distance_valid=True, stop_distance_m=0.5))
    assert (output.stage,output.steering_deg,output.status)==(1,4.0,"START")


def test_start_ignores_all_traffic_signs_and_lights():
    output=CourseMission().update(data(
        1,stop_detected=True,stop_distance_valid=True,stop_distance_m=0.5,
        traffic_red=True,traffic_yellow=True,traffic_green=True,
        traffic20_detected=True,camera_steering_deg=-9.0))
    assert (output.stage,output.steering_deg,output.status)==(1,-9.0,"START")


def test_ramp_requires_stable_pitch_then_aligns_and_tracks_path():
    logic=CourseMission(ramp_pitch_confirm_sec=0.3,ramp_delay_sec=0.5,
                        ramp_slow_hold_sec=3.0)
    waiting=logic.update(data(2,0.0,imu_valid=True,pitch_deg=5.0))
    assert (waiting.stage,waiting.steering_deg)==(1,4.0)
    aligning=logic.update(data(2,0.3,imu_valid=True,pitch_deg=5.1))
    assert (aligning.stage,aligning.steering_deg)==(0,0.0)
    stage_two=logic.update(data(2,0.8,imu_valid=True,pitch_deg=6.0,
                                camera_steering_deg=7.0))
    assert (stage_two.stage,stage_two.steering_deg)==(2,7.0)
    assert stage_two.status=="RAMP:SLOPE_STAGE_2_PATH_HOLD_3SEC"
    stage_one=logic.update(data(2,3.8,imu_valid=True,pitch_deg=6.0,
                                camera_steering_deg=-4.0))
    assert (stage_one.stage,stage_one.steering_deg)==(1,-4.0)
    assert stage_one.status=="RAMP:SLOPE_STAGE_1_PATH_FOLLOW"


def test_ramp_second_line_stops_one_second_then_runs_stage_one_three_seconds():
    logic=CourseMission(ramp_pitch_confirm_sec=0.0,stop_line_rearm_sec=0.5)
    first=logic.update(data(2,0.0,imu_valid=True,pitch_deg=6.0,
                            stop_detected=True,stop_distance_valid=True,
                            stop_distance_m=1.0))
    assert first.status=="RAMP:ALIGN_WHEELS"
    logic.update(data(2,0.1,imu_valid=True,pitch_deg=6.0))
    logic.update(data(2,0.6,imu_valid=True,pitch_deg=6.0))
    approaching=logic.update(data(
        2,0.7,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.1))
    assert approaching.stage==2
    stopped=logic.update(data(
        2,0.8,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0,
        camera_steering_deg=-6.0))
    assert (stopped.stage,stopped.steering_deg)==(0,0.0)
    assert stopped.status=="RAMP:SECOND_STOP_LINE_STOP_0.0SEC"
    assert logic.update(data(
        2,1.79,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0)).stage==0
    moving=logic.update(data(
        2,1.8,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0,
        camera_steering_deg=-6.0))
    assert (moving.stage,moving.steering_deg)==(1,-6.0)
    assert moving.status=="RAMP:SECOND_STOP_LINE_STAGE_1_0.0SEC"
    still_moving=logic.update(data(
        2,4.79,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0,speed_valid=True,
        speed_mps=0.2))
    assert still_moving.stage==1
    completed=logic.update(data(
        2,4.8,imu_valid=True,pitch_deg=6.0,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0,speed_valid=True,
        speed_mps=0.2,camera_steering_deg=9.0))
    assert (completed.stage,completed.steering_deg)==(2,9.0)
    assert completed.status=="RAMP:SECOND_STOP_LINE_STAGE_2_PATH_FOLLOW"
    assert logic.ramp_second_line_completed


def test_s_curve_never_stops_merely_for_yellow_boundary():
    output=CourseMission().update(data(
        5,yellow_ahead_valid=True,yellow_ahead_m=0.5,
        camera_steering_deg=-8.0))
    assert (output.stage,output.steering_deg)==(1,-8.0)


def intersection_data(now, **kwargs):
    values = dict(stop_detected=True, stop_distance_valid=True,
                  stop_distance_m=0.5, speed_valid=True, speed_mps=0.0)
    values.update(kwargs)
    return data(4, now, **values)


@pytest.mark.parametrize("section",(4,6,8))
def test_first_two_intersections_ignore_missing_line_or_signal(section):
    logic=CourseMission()
    missing_signal=logic.update(data(
        section,stop_detected=True,stop_distance_valid=True,
        stop_distance_m=1.0))
    assert missing_signal.stage==1
    missing_line=logic.update(data(section,traffic_red=True))
    assert missing_line.stage==1


@pytest.mark.parametrize("section",(4,6,8))
@pytest.mark.parametrize("signal",("traffic_red","traffic_yellow"))
def test_first_two_intersections_stop_for_red_or_yellow_at_two_meters(section,signal):
    logic=CourseMission()
    kwargs={signal:True,"stop_detected":True,"stop_distance_valid":True,
            "stop_distance_m":2.0}
    output=logic.update(data(section,**kwargs))
    assert output.stage==0
    assert output.status.endswith("STOP_AT_2M")


@pytest.mark.parametrize("section",(4,6,8))
def test_first_two_intersections_green_goes_and_far_red_approaches(section):
    logic=CourseMission()
    green=logic.update(data(
        section,traffic_green=True,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.0))
    assert (green.stage,green.status)==(1,"INTERSECTION_GO")
    far_red=logic.update(data(
        section,traffic_red=True,stop_detected=True,
        stop_distance_valid=True,stop_distance_m=2.1))
    assert (far_red.stage,far_red.status)==(
        1,"INTERSECTION_APPROACH_STOP_LINE")


def test_parking_sections_keep_camera_candidate_for_mcu_priority():
    logic = CourseMission()
    out = logic.update(data(7, camera_steering_deg=-8.0))
    assert (out.stage, out.steering_deg, out.control_mode) == (1, -8.0, CAMERA)
    assert out.status == "T_COURSE:PATH_FOLLOW_PUBLISHING"
    out = logic.update(data(10, camera_steering_deg=6.0))
    assert (out.stage, out.steering_deg, out.control_mode) == (1, 6.0, CAMERA)
    assert out.status == "PARALLEL_PARK:PATH_FOLLOW_PUBLISHING"


def test_two_traffic20_events_toggle_stage_two_to_three_and_back():
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
    assert confirmed.status == "TRAFFIC20_COUNT_1:STAGE_3"
    logic.update(data(9,now=4.0,traffic20_detected=False,
                      planned_drive_stage=2))
    logic.update(data(9,now=4.5,traffic20_detected=False,
                      planned_drive_stage=2))
    assert logic.update(data(9,now=5.0,traffic20_detected=True,
                             planned_drive_stage=2)).stage==3
    assert logic.update(data(9,now=6.99,traffic20_detected=True,
                             planned_drive_stage=2)).stage==3
    second=logic.update(data(9,now=7.0,traffic20_detected=True,
                             planned_drive_stage=2))
    assert (second.stage,second.status)==(2,"TRAFFIC20_COUNT_2:STAGE_2")


def test_traffic20_confirmation_resets_if_detection_breaks_before_two_seconds():
    logic = CourseMission()
    logic.update(data(9, now=1.0, traffic20_detected=True,
                      planned_drive_stage=2))
    logic.update(data(9, now=2.0, traffic20_detected=False,
                      planned_drive_stage=2))
    logic.update(data(9, now=2.5, traffic20_detected=False,
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


def test_finish_red_stops_at_one_meter_and_green_goes():
    logic = CourseMission()
    red=logic.update(data(
        11,stop_detected=True,stop_distance_valid=True,stop_distance_m=1.0,
        final_signal_red=True))
    assert (red.stage,red.status)==(0,"FINISH_RED_STOP_AT_1M")
    green=logic.update(data(
        11,stop_detected=True,stop_distance_valid=True,stop_distance_m=0.5,
        final_signal_green=True))
    assert (green.stage,green.status)==(1,"FINISH_GREEN_GO")
    unknown=logic.update(data(11,stop_detected=False))
    assert unknown.stage==1
    assert logic.update(data(1, camera_path_valid=False)).stage == 0

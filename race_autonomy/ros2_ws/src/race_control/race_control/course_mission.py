"""ROS-independent state logic for the 11-section driving course."""

from dataclasses import dataclass, replace
import math
from .ramp_filter import RampPitchFilter


LEFT, STRAIGHT, RIGHT = -1, 0, 1
CAMERA = 1
INTERSECTIONS = {4, 6, 8}

EMERGENCY_SAFE_STOP_PREFIXES = (
    "SAFE_STOP:INPUT_STREAM_LOST",
)


def camera_emergency_stop(status, control_was_active):
    """Separate a true run-time control failure from an ordinary stage-0 stop.

    Before the vehicle has moved, missing perception is a normal disarmed wait.
    A path or speed-plan rejection already commands drive stage zero at 20 Hz,
    so it must not repeatedly pulse the MCU emergency brake. Only an explicit
    loss of the guarded input stream after motion uses the independent hard-
    stop channel. Traffic lights, stop lines and curvature stops are also
    ordinary drive-stage zero commands.
    """
    return bool(control_was_active) and str(status).startswith(
        EMERGENCY_SAFE_STOP_PREFIXES)


def section_after_ramp_detection(section, imu_valid, pitch_deg,
                                 ramp_pitch_deg=5.0):
    """Latch the start section into the ramp section at the pitch threshold."""
    if int(section) == 1 and bool(imu_valid) and float(pitch_deg) >= float(ramp_pitch_deg):
        return 2
    return int(section)


def traffic20_drive_stage(planned_stage, sign_seen):
    """Upgrade only a curvature-safe cruise stage 2 to stage 3."""
    stage = max(0, min(2, int(planned_stage)))
    return 3 if bool(sign_seen) and stage == 2 else stage


@dataclass
class MissionInput:
    section: int = 1
    waypoint_valid: bool = False
    waypoint_event: str = ""
    waypoint_stop_section: int = -1
    waypoint_remaining_m: float = float("inf")
    acceleration_active: bool = False
    confirmed_signal_window: str = ""
    now: float = 0.0
    pitch_deg: float = 0.0
    imu_valid: bool = False
    # DR adapter supplies a latched arrival event for the required ramp stop.
    ramp_dr_stop_reached: bool = False
    stop_detected: bool = False
    stop_distance_m: float = float("inf")
    stop_distance_valid: bool = False
    traffic_green: bool = False
    traffic_left: bool = False
    traffic_red: bool = False
    traffic_yellow: bool = False
    traffic20_detected: bool = False
    final_signal_green: bool = False
    final_signal_red: bool = False
    gps_direction: int = STRAIGHT
    camera_path_valid: bool = False
    camera_steering_deg: float = 0.0
    speed_plan_valid: bool = True
    planned_drive_stage: int = 2
    yellow_ahead_m: float = float("inf")
    yellow_ahead_valid: bool = False
    speed_mps: float = 0.0
    speed_valid: bool = False
    input_guard_alive: bool = False
    odom_distance_m: float = 0.0
    odom_distance_valid: bool = False


@dataclass
class MissionOutput:
    stage: int
    steering_deg: float
    control_mode: int
    turn_direction: int
    status: str


class CourseMission:
    def __init__(self, ramp_pitch_deg=4.5, ramp_delay_sec=0.5,
                 stop_distance_m=2.0, minimum_stop_sec=2.0,
                 ramp_level_pitch_deg=3.0, ramp_slow_pitch_deg=5.0,
                 ramp_slow_hold_sec=3.0, green_confirm_sec=2.0,
                 actual_stop_speed_mps=0.05,
                 ramp_pitch_confirm_sec=1.0,
                 stop_line_rearm_sec=0.5,
                 ramp_post_stop_drive_sec=0.0,
                 ramp_second_line_stop_sec=3.0,
                 traffic20_rearm_sec=0.5,
                 traffic20_rearm_distance_m=2.0,
                 ramp_stop_line_min_separation_m=1.5,
                 intersection_stop_wait_sec=1.0,
                 ramp_entry_distance_m=0.5, ramp_filter_tau_sec=0.25,
                 ramp_roughness_tau_sec=0.3, ramp_roughness_limit_deg=1.0,
                 ramp_outlier_limit_deg=45.0):
        self.ramp_filter = RampPitchFilter(
            ramp_filter_tau_sec, ramp_roughness_tau_sec,
            ramp_roughness_limit_deg, ramp_outlier_limit_deg)
        self.ramp_entry_distance_m = float(ramp_entry_distance_m)
        self.ramp_pitch_deg = float(ramp_pitch_deg)
        self.ramp_delay_sec = float(ramp_delay_sec)
        self.stop_distance_m = float(stop_distance_m)
        self.minimum_stop_sec = float(minimum_stop_sec)
        self.ramp_level_pitch_deg = float(ramp_level_pitch_deg)
        self.ramp_slow_pitch_deg = float(ramp_slow_pitch_deg)
        self.ramp_slow_hold_sec = float(ramp_slow_hold_sec)
        self.green_confirm_sec = float(green_confirm_sec)
        self.actual_stop_speed_mps = float(actual_stop_speed_mps)
        self.ramp_pitch_confirm_sec = float(ramp_pitch_confirm_sec)
        self.stop_line_rearm_sec = float(stop_line_rearm_sec)
        self.ramp_post_stop_drive_sec = float(ramp_post_stop_drive_sec)
        self.ramp_second_line_stop_sec = float(ramp_second_line_stop_sec)
        self.traffic20_rearm_sec = float(traffic20_rearm_sec)
        self.traffic20_rearm_distance_m=float(traffic20_rearm_distance_m)
        self.ramp_stop_line_min_separation_m = float(
            ramp_stop_line_min_separation_m)
        self.intersection_stop_wait_sec = float(intersection_stop_wait_sec)
        self.section = None
        self.waypoint_event = ""
        self.released_event = ""
        self.signal_window = ""
        self.waypoint_stop_started = None
        self.waypoint_braking = False
        self.signal_attempt = 0
        self.waypoint_stop_tolerance_m = 0.25
        self.waypoint_deceleration_mps2 = 0.5
        self.waypoint_command_latency_sec = 0.15
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.ramp_pitch_candidate_time = None
        self.ramp_entry_odom_m = None
        self.ramp_stop_line_count = 0
        self.ramp_stop_line_visible = False
        self.ramp_stop_line_lost_time = None
        self.ramp_first_stop_line_odom_m = None
        self.ramp_second_line_stopped = False
        self.ramp_second_line_stop_start = None
        self.ramp_second_line_go_start = None
        self.ramp_second_line_completed = False
        self.section_request = None
        self.yellow_stop_time = None
        self.yellow_handled = False
        self.intersection_stop_time = None
        self.intersection_released = False
        self.green_confirm_start_time = None
        self.final_stopped = False
        self.traffic20_start_time = None
        self.traffic20_confirmed = False
        self.traffic20_count = 0
        self.traffic20_active = False
        self.traffic20_absent_start = None
        self.traffic20_first_odom_m = None

    def enter_section(self, section):
        if section == self.section:
            return False
        self.section = section
        self.waypoint_event = ""
        self.released_event = ""
        self.signal_window = ""
        self.waypoint_stop_started = None
        self.waypoint_braking = False
        self.ramp_filter.reset()
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.ramp_pitch_candidate_time = None
        self.ramp_entry_odom_m = None
        self.ramp_stop_line_count = 0
        self.ramp_stop_line_visible = False
        self.ramp_stop_line_lost_time = None
        self.ramp_first_stop_line_odom_m = None
        self.ramp_second_line_stopped = False
        self.ramp_second_line_stop_start = None
        self.ramp_second_line_go_start = None
        self.ramp_second_line_completed = False
        self.section_request = None
        self.yellow_stop_time = None
        self.yellow_handled = False
        self.intersection_stop_time = None
        self.intersection_released = False
        self.green_confirm_start_time = None
        self.traffic20_start_time = None
        if section == 9:
            self.traffic20_confirmed = False
            self.traffic20_count = 0
            self.traffic20_active = False
            self.traffic20_absent_start = None
            self.traffic20_first_odom_m = None
        if section != 11:
            self.final_stopped = False
        return True

    @staticmethod
    def stopped(status, mode=CAMERA, direction=STRAIGHT):
        return MissionOutput(0, 0.0, mode, direction, status)

    def camera_output(self, data, steering_deg, status, direction, maximum_stage=1):
        if not data.speed_plan_valid:
            return self.stopped("SAFE_STOP:SPEED_PLAN_INVALID", CAMERA, direction)
        stage = max(0, min(int(maximum_stage), int(data.planned_drive_stage)))
        if stage == 0:
            return MissionOutput(0, float(steering_deg), CAMERA, direction,
                                 f"CURVATURE_STOP:{status}")
        return MissionOutput(stage, float(steering_deg), CAMERA, direction, status)

    def update_waypoint(self, data):
        """Waypoint stop targets replace painted lines and traffic20 detections."""
        self.enter_section(int(data.section))
        previous_window = self.signal_window
        self.signal_window = ""
        if not data.waypoint_valid:
            # A gap must never consume a stop hold or leave a signal armed.
            self.waypoint_stop_started = None
            return self.stopped("SAFE_STOP:WAYPOINT_STATE_INVALID")
        event = data.waypoint_event
        if event != self.waypoint_event:
            self.waypoint_event = event
            self.waypoint_stop_started = None
            self.waypoint_braking = False
            previous_window = ""
        if (event and event != self.released_event
                and data.waypoint_stop_section == 2):
            # URRC_MODE2_WAYPOINT_PASS_THROUGH
            # Mode 2 ramp waypoint is informational only.
            # Do not brake, stop or hold because of this waypoint.
            self.released_event = event
            self.waypoint_stop_started = None
            self.waypoint_braking = False
            self.ramp_second_line_completed = True

        if event and event != self.released_event:
            remaining = data.waypoint_remaining_m
            if not math.isfinite(remaining):
                return self.stopped("SAFE_STOP:WAYPOINT_DISTANCE_INVALID")
            if remaining <= 3.0:
                if not data.speed_valid or not math.isfinite(data.speed_mps):
                    self.waypoint_stop_started = None
                    return self.stopped("WAYPOINT:WAIT_SPEED_FEEDBACK")
                speed = abs(data.speed_mps)
                braking_distance = (speed*speed/(2*self.waypoint_deceleration_mps2)
                                    + speed*self.waypoint_command_latency_sec)
                self.waypoint_braking |= remaining <= max(0.05, braking_distance)
                if self.waypoint_braking:
                    if speed > self.actual_stop_speed_mps:
                        self.waypoint_stop_started = None
                        return self.stopped("WAYPOINT:BRAKING")
                    if abs(remaining) > self.waypoint_stop_tolerance_m:
                        self.waypoint_stop_started = None
                        return self.stopped("WAYPOINT:STOP_POSITION_ERROR")
                    if data.waypoint_stop_section == 2:
                        # Mode 2 waypoint stop/hold disabled.
                        self.ramp_second_line_completed = True
                    else:
                        # The perception node counts only frames in this stop's
                        # window; no pre-stop or previous-intersection vote is reused.
                        if not previous_window:
                            self.signal_attempt += 1
                        self.signal_window = previous_window or f"{event}/vote/{self.signal_attempt}"
                        permitted = (data.final_signal_green if data.waypoint_stop_section == 11
                                     else data.traffic_left if data.waypoint_stop_section == 8
                                     else data.traffic_green)
                        if not permitted or data.confirmed_signal_window != self.signal_window:
                            return self.stopped("WAYPOINT:WAIT_SIGNAL_7_FRAMES")
                    self.released_event = event
                    self.signal_window = ""
                elif not data.camera_path_valid:
                    return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID")
                else:
                    return self.camera_output(data, data.camera_steering_deg,
                                              "WAYPOINT:APPROACH", data.gps_direction, 1)
        if not data.camera_path_valid:
            return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID")
        if not data.speed_plan_valid:
            return self.stopped("SAFE_STOP:SPEED_PLAN_INVALID")
        if data.section == 9:
            safe_stage = max(0, min(2, data.planned_drive_stage))
            stage = 3 if data.acceleration_active and safe_stage == 2 else safe_stage
            return MissionOutput(stage, data.camera_steering_deg, CAMERA,
                                 data.gps_direction, "ACCEL:WAYPOINT_ACTIVE" if
                                 data.acceleration_active else "ACCEL:WAYPOINT_CRUISE")
        if data.section in (4, 6, 8, 11):
            return self.camera_output(data, data.camera_steering_deg,
                                      "INTERSECTION:WAYPOINT_PATH_FOLLOW", data.gps_direction)
        result = self.update(replace(data, ramp_dr_stop_reached=False))
        # Incline-specific stage requests still obey the path speed planner.
        return replace(result, stage=min(result.stage, max(0, int(data.planned_drive_stage))))

    def update(self, data):
        self.enter_section(int(data.section))
        section = self.section
        direction = data.gps_direction if data.gps_direction in (LEFT, STRAIGHT, RIGHT) else STRAIGHT

        if section == 2:
            pitch_stable = self.ramp_filter.update(
                data.pitch_deg, data.now, data.imu_valid)
            # A DR arrival is a stop request, independent of incline confirmation.
            # Keep it latched even if the arrival message is a short pulse.
            if data.ramp_dr_stop_reached:
                self.ramp_second_line_stopped = True
            if (self.ramp_second_line_stopped and
                    not self.ramp_second_line_completed):
                self.ramp_pitch_candidate_time = None
                if self.ramp_second_line_stop_start is None:
                    self.ramp_second_line_stop_start=data.now
                stop_elapsed=data.now-self.ramp_second_line_stop_start
                if stop_elapsed < self.ramp_second_line_stop_sec:
                    return self.stopped(
                        f"RAMP:SECOND_STOP_LINE_STOP_{stop_elapsed:.1f}SEC",
                        CAMERA,direction)
                if self.ramp_second_line_go_start is None:
                    self.ramp_second_line_go_start=data.now
                elapsed=data.now-self.ramp_second_line_go_start
                if elapsed < self.ramp_post_stop_drive_sec:
                    return self.stopped(
                        f"RAMP:SECOND_STOP_LINE_HOLD_{elapsed:.1f}SEC",
                        CAMERA,direction)
                self.ramp_second_line_completed=True
            if not self.ramp_crossing:
                odom_valid = (data.odom_distance_valid and
                              math.isfinite(data.odom_distance_m))
                if not odom_valid:
                    self.ramp_pitch_candidate_time = None
                    self.ramp_entry_odom_m = None
                    return self.stopped("RAMP:ENTRY_ODOM_INVALID", CAMERA, direction)
                if (self.ramp_entry_odom_m is None or
                        data.odom_distance_m < self.ramp_entry_odom_m):
                    self.ramp_entry_odom_m = data.odom_distance_m
                    self.ramp_pitch_candidate_time = None
            if not self.ramp_filter.valid:
                self.ramp_pitch_candidate_time = None
                return self.stopped("RAMP:IMU_INVALID", CAMERA, direction)
            if not self.ramp_crossing:
                distance_ready = (data.odom_distance_m-self.ramp_entry_odom_m >=
                                  self.ramp_entry_distance_m)
                # Start the continuous incline timer only after the entry distance.
                if (distance_ready and pitch_stable and
                        self.ramp_filter.pitch >= self.ramp_pitch_deg):
                    if self.ramp_pitch_candidate_time is None:
                        self.ramp_pitch_candidate_time = data.now
                    if (data.now-self.ramp_pitch_candidate_time >=
                            self.ramp_pitch_confirm_sec):
                        self.ramp_trigger_time = data.now
                        self.ramp_crossing = True
                else:
                    self.ramp_pitch_candidate_time = None

            if self.ramp_second_line_completed and self.ramp_crossing:
                if not data.camera_path_valid:
                    return self.stopped(
                        "RAMP:SECOND_STOP_LINE_STAGE_2_PATH_INVALID",
                        CAMERA,direction)
                return MissionOutput(
                    2,float(data.camera_steering_deg),CAMERA,direction,
                    "RAMP:SECOND_STOP_LINE_STAGE_2_PATH_FOLLOW")
            if self.ramp_crossing:
                elapsed=data.now-self.ramp_trigger_time
                if elapsed < self.ramp_delay_sec:
                    return self.stopped("RAMP:ALIGN_WHEELS", CAMERA, direction)
                if not data.camera_path_valid:
                    return self.stopped("RAMP:SLOPE_PATH_INVALID", CAMERA,
                                        direction)
                if elapsed < self.ramp_delay_sec+self.ramp_slow_hold_sec:
                    return MissionOutput(
                        2, float(data.camera_steering_deg), CAMERA, direction,
                        "RAMP:SLOPE_STAGE_2_PATH_HOLD_3SEC")
                return MissionOutput(
                    1, float(data.camera_steering_deg), CAMERA, direction,
                    "RAMP:SLOPE_STAGE_1_PATH_FOLLOW")
            if not data.camera_path_valid:
                return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)
            return self.camera_output(
                data, data.camera_steering_deg, "RAMP:WAIT_STABLE_PITCH_PATH_FOLLOW", direction)

        if section in INTERSECTIONS:
            if not data.camera_path_valid:
                return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)
            line_visible=data.stop_detected and data.stop_distance_valid
            if self.intersection_released:
                return self.camera_output(
                    data, data.camera_steering_deg,
                    ("INTERSECTION_LEFT_GO" if section == 8 else
                     "INTERSECTION_GREEN_GO"), direction)
            if self.intersection_stop_time is None and not line_visible:
                return self.camera_output(
                    data, data.camera_steering_deg,
                    "INTERSECTION_SEARCHING_FOR_STOP_LINE", direction)
            if (self.intersection_stop_time is None and
                    data.stop_distance_m > self.stop_distance_m):
                return self.camera_output(
                    data, data.camera_steering_deg,
                    "INTERSECTION_APPROACH_STOP_LINE", direction)
            if self.intersection_stop_time is None:
                self.intersection_stop_time = data.now
            wait_elapsed = max(0.0, data.now-self.intersection_stop_time)
            if wait_elapsed < self.intersection_stop_wait_sec:
                return self.stopped(
                    f"INTERSECTION_STOP_WAIT_{wait_elapsed:.1f}SEC",
                    CAMERA, direction)
            permitted = (data.traffic_left if section == 8 else
                         data.traffic_green)
            if permitted:
                self.intersection_released = True
                return self.camera_output(
                    data, data.camera_steering_deg,
                    ("INTERSECTION_LEFT_GO" if section == 8 else
                     "INTERSECTION_GREEN_GO"), direction)
            signal = ("NOT_LEFT" if section == 8 else
                      "RED" if data.traffic_red else
                      "YELLOW" if data.traffic_yellow else "NOT_GREEN")
            return self.stopped(
                f"INTERSECTION_{signal}_STOP_AT_2M", CAMERA, direction)

        if not data.camera_path_valid:
            return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)

        if section == 9:
            if data.traffic20_detected:
                self.traffic20_absent_start = None
                if not self.traffic20_active:
                    if not data.odom_distance_valid:
                        return self.stopped("SAFE_STOP:TRAFFIC20_ODOM_INVALID",CAMERA,direction)
                    if self.traffic20_count==0:
                        self.traffic20_count=1
                        self.traffic20_first_odom_m=float(data.odom_distance_m)
                        self.traffic20_active=True
                    elif (float(data.odom_distance_m)-
                          float(self.traffic20_first_odom_m) >=
                          self.traffic20_rearm_distance_m):
                        self.traffic20_count=2
                        self.traffic20_active=True
                    self.traffic20_start_time = data.now
                    self.traffic20_confirmed = self.traffic20_active
            elif self.traffic20_active:
                if self.traffic20_absent_start is None:
                    self.traffic20_absent_start = data.now
                elif (data.now-self.traffic20_absent_start >=
                      self.traffic20_rearm_sec):
                    self.traffic20_active = False
                    self.traffic20_confirmed = False
                    self.traffic20_start_time = None
                    self.traffic20_absent_start = None
            if not data.speed_plan_valid:
                return self.stopped("SAFE_STOP:SPEED_PLAN_INVALID", CAMERA, direction)
            safe_stage=max(0,min(2,int(data.planned_drive_stage)))
            if safe_stage < 2:
                stage=safe_stage
            else:
                stage=3 if self.traffic20_count == 1 else 2
            if self.traffic20_count:
                status=(f"TRAFFIC20_COUNT_{self.traffic20_count}:"
                        f"STAGE_{stage}")
            elif self.traffic20_start_time is not None:
                elapsed = max(0.0, data.now-self.traffic20_start_time)
                status = f"TRAFFIC20_CONFIRMING_{elapsed:.1f}SEC:STAGE_{stage}"
            else:
                status = f"TRAFFIC20_WAIT:STAGE_{stage}"
            return MissionOutput(
                stage, float(data.camera_steering_deg), CAMERA, direction,
                status)

        if section in (7, 10):
            return self.camera_output(
                data, data.camera_steering_deg,
                "T_COURSE:PATH_FOLLOW_PUBLISHING" if section == 7
                else "PARALLEL_PARK:PATH_FOLLOW_PUBLISHING",
                direction)

        if section == 11:
            line_visible=(data.stop_detected and data.stop_distance_valid)
            if self.intersection_released:
                return self.camera_output(
                    data,data.camera_steering_deg,"FINISH_GREEN_GO",direction)
            if self.intersection_stop_time is None and not line_visible:
                return self.camera_output(
                    data,data.camera_steering_deg,
                    "FINISH_INTERSECTION_SEARCHING_FOR_STOP_LINE",direction)
            if (self.intersection_stop_time is None and
                    data.stop_distance_m > self.stop_distance_m):
                return self.camera_output(
                    data,data.camera_steering_deg,
                    "FINISH_INTERSECTION_APPROACH_STOP_LINE",direction)
            if self.intersection_stop_time is None:
                self.intersection_stop_time = data.now
            wait_elapsed = max(0.0, data.now-self.intersection_stop_time)
            if wait_elapsed < self.intersection_stop_wait_sec:
                return self.stopped(
                    f"FINISH_STOP_WAIT_{wait_elapsed:.1f}SEC",CAMERA,direction)
            if data.final_signal_green:
                self.intersection_released = True
                return self.camera_output(
                    data,data.camera_steering_deg,"FINISH_GREEN_GO",direction)
            signal="RED" if data.final_signal_red else "NOT_GREEN"
            return self.stopped(
                f"FINISH_{signal}_STOP_AT_2M",CAMERA,direction)

        if section == 5:
            return self.camera_output(
                data, data.camera_steering_deg,
                "S_CURVE:OBSTACLE_YELLOW_CORRIDOR_PATH_FOLLOW", direction)

        names = {1: "START", 3: "CURVE_CENTERING"}
        return self.camera_output(
            data, data.camera_steering_deg,
            names.get(section, "CAMERA_CENTERING"), direction)

"""ROS-independent state logic for the 11-section driving course."""

from dataclasses import dataclass


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
                                 ramp_pitch_deg=15.0):
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
    now: float = 0.0
    pitch_deg: float = 0.0
    imu_valid: bool = False
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
    def __init__(self, ramp_pitch_deg=15.0, ramp_delay_sec=0.5,
                 stop_distance_m=2.0, minimum_stop_sec=2.0,
                 ramp_level_pitch_deg=3.0, ramp_slow_pitch_deg=5.0,
                 ramp_slow_hold_sec=3.0, green_confirm_sec=2.0,
                 actual_stop_speed_mps=0.05,
                 ramp_pitch_confirm_sec=0.5,
                 stop_line_rearm_sec=0.5,
                 ramp_post_stop_drive_sec=0.0,
                 ramp_second_line_stop_sec=3.0,
                 traffic20_rearm_sec=0.5,
                 traffic20_rearm_distance_m=2.0,
                 ramp_stop_line_min_separation_m=1.5):
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
        self.section = None
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.ramp_pitch_candidate_time = None
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
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.ramp_pitch_candidate_time = None
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

    def update(self, data):
        self.enter_section(int(data.section))
        section = self.section
        direction = data.gps_direction if data.gps_direction in (LEFT, STRAIGHT, RIGHT) else STRAIGHT

        if section == 2:
            if not data.imu_valid:
                return self.stopped("RAMP:IMU_INVALID", CAMERA, direction)
            trigger = data.pitch_deg >= self.ramp_pitch_deg
            if trigger:
                if self.ramp_pitch_candidate_time is None:
                    self.ramp_pitch_candidate_time = data.now
                stable_pitch = (data.now-self.ramp_pitch_candidate_time >=
                                self.ramp_pitch_confirm_sec)
            else:
                self.ramp_pitch_candidate_time = None
                stable_pitch = False
            if stable_pitch and self.ramp_trigger_time is None:
                self.ramp_trigger_time = data.now
                self.ramp_crossing = True

            line_visible = bool(data.stop_detected and
                                data.stop_distance_valid)
            if self.ramp_crossing:
                if line_visible and not self.ramp_stop_line_visible:
                    if not data.odom_distance_valid:
                        return self.stopped(
                            "RAMP:STOP_LINE_ODOM_INVALID", CAMERA, direction)
                    if self.ramp_stop_line_count == 0:
                        self.ramp_stop_line_count = 1
                        self.ramp_first_stop_line_odom_m = float(
                            data.odom_distance_m)
                    elif (self.ramp_stop_line_count == 1 and
                          float(data.odom_distance_m) -
                          float(self.ramp_first_stop_line_odom_m) >=
                          self.ramp_stop_line_min_separation_m):
                        self.ramp_stop_line_count = 2
                    self.ramp_stop_line_visible = True
                    self.ramp_stop_line_lost_time = None
                elif not line_visible and self.ramp_stop_line_visible:
                    if self.ramp_stop_line_lost_time is None:
                        self.ramp_stop_line_lost_time = data.now
                    elif (data.now-self.ramp_stop_line_lost_time >=
                          self.stop_line_rearm_sec):
                        self.ramp_stop_line_visible = False
                        self.ramp_stop_line_lost_time = None
                if (self.ramp_stop_line_count >= 2 and line_visible and
                        data.stop_distance_m <= self.stop_distance_m):
                    self.ramp_second_line_stopped = True
                if (self.ramp_second_line_stopped and
                        not self.ramp_second_line_completed):
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
                if self.ramp_second_line_completed:
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
            if not line_visible:
                return self.camera_output(
                    data, data.camera_steering_deg,
                    "INTERSECTION_SEARCHING_FOR_STOP_LINE", direction)
            if data.stop_distance_m > self.stop_distance_m:
                return self.camera_output(
                    data, data.camera_steering_deg,
                    "INTERSECTION_APPROACH_STOP_LINE", direction)
            permitted = (data.traffic_left if section == 8 else
                         data.traffic_green)
            if permitted:
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
            if not line_visible:
                return self.camera_output(
                    data,data.camera_steering_deg,
                    "FINISH_INTERSECTION_SEARCHING_FOR_STOP_LINE",direction)
            if data.stop_distance_m > self.stop_distance_m:
                return self.camera_output(
                    data,data.camera_steering_deg,
                    "FINISH_INTERSECTION_APPROACH_STOP_LINE",direction)
            if data.final_signal_green:
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

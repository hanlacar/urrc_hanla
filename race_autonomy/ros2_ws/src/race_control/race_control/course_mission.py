"""ROS-independent state logic for the 13-section driving course."""

from dataclasses import dataclass


LEFT, STRAIGHT, RIGHT = -1, 0, 1
CAMERA = 1
INTERSECTIONS = {4, 6, 8, 11}


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
    now: float = 0.0
    pitch_deg: float = 0.0
    imu_valid: bool = False
    stop_detected: bool = False
    stop_distance_m: float = float("inf")
    stop_distance_valid: bool = False
    traffic_green: bool = False
    traffic20_detected: bool = False
    gps_direction: int = STRAIGHT
    camera_path_valid: bool = False
    camera_steering_deg: float = 0.0
    speed_plan_valid: bool = True
    planned_drive_stage: int = 2
    yellow_ahead_m: float = float("inf")
    yellow_ahead_valid: bool = False


@dataclass
class MissionOutput:
    stage: int
    steering_deg: float
    control_mode: int
    turn_direction: int
    status: str


class CourseMission:
    def __init__(self, ramp_pitch_deg=5.0, ramp_delay_sec=0.5,
                 stop_distance_m=0.5, minimum_stop_sec=0.5,
                 ramp_level_pitch_deg=3.0, ramp_slow_pitch_deg=10.0,
                 ramp_slow_hold_sec=3.0):
        self.ramp_pitch_deg = float(ramp_pitch_deg)
        self.ramp_delay_sec = float(ramp_delay_sec)
        self.stop_distance_m = float(stop_distance_m)
        self.minimum_stop_sec = float(minimum_stop_sec)
        self.ramp_level_pitch_deg = float(ramp_level_pitch_deg)
        self.ramp_slow_pitch_deg = float(ramp_slow_pitch_deg)
        self.ramp_slow_hold_sec = float(ramp_slow_hold_sec)
        self.section = None
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.section_request = None
        self.yellow_stop_time = None
        self.yellow_handled = False
        self.intersection_stop_time = None
        self.intersection_released = False
        self.final_stopped = False
        self.traffic20_start_time = None
        self.traffic20_confirmed = False
        self.yellow_stop_time = None
        self.yellow_handled = False

    def enter_section(self, section):
        if section == self.section:
            return False
        self.section = section
        self.ramp_trigger_time = None
        self.ramp_crossing = False
        self.ramp_slow_start_time = None
        self.ramp_slow_latched = False
        self.ramp_level_start_time = None
        self.intersection_stop_time = None
        self.intersection_released = False
        self.traffic20_start_time = None
        if section == 9:
            self.traffic20_confirmed = False
        if section != 13:
            self.final_stopped = False
        return

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

        if not data.yellow_ahead_valid or data.yellow_ahead_m >= 1.2:
            self.yellow_handled = False
        if (data.yellow_ahead_valid and data.yellow_ahead_m <= 1.0 and
                not self.yellow_handled and self.yellow_stop_time is None):
            self.yellow_stop_time = data.now
            self.yellow_handled = True
        if self.yellow_stop_time is not None:
            if data.now-self.yellow_stop_time < 0.5 or not data.camera_path_valid:
                return self.stopped("YELLOW_LINE:REPLAN_STOP", CAMERA, direction)
            self.yellow_stop_time = None

        if section == 2:
            if not data.imu_valid:
                return self.stopped("RAMP:IMU_INVALID", CAMERA, direction)
            trigger = data.pitch_deg >= self.ramp_pitch_deg
            if trigger and self.ramp_trigger_time is None:
                self.ramp_trigger_time = data.now
                self.ramp_crossing = True
                if data.pitch_deg >= self.ramp_slow_pitch_deg:
                    self.ramp_slow_start_time = data.now
            if self.ramp_crossing:
                if abs(data.pitch_deg) <= self.ramp_level_pitch_deg:
                    if self.ramp_level_start_time is None:
                        self.ramp_level_start_time = data.now
                    if data.now-self.ramp_level_start_time < 1.0:
                        return MissionOutput(1, 0.0, CAMERA, direction,
                                             "RAMP:LEVEL_CONFIRM_1SEC")
                    self.ramp_crossing = False
                    self.ramp_trigger_time = None
                    self.ramp_slow_start_time = None
                    self.ramp_slow_latched = False
                    self.ramp_level_start_time = None
                    self.section_request = 3
                elif data.now - self.ramp_trigger_time < self.ramp_delay_sec:
                    self.ramp_level_start_time = None
                    return self.stopped("RAMP:ALIGN_WHEELS", CAMERA, direction)
                else:
                    self.ramp_level_start_time = None
                    if data.pitch_deg >= self.ramp_slow_pitch_deg:
                        if self.ramp_slow_start_time is None:
                            self.ramp_slow_start_time = data.now
                    else:
                        self.ramp_slow_start_time = None
                    if (self.ramp_slow_start_time is not None and
                            data.now-self.ramp_slow_start_time >= self.ramp_slow_hold_sec):
                        self.ramp_slow_latched = True
                    if self.ramp_slow_latched:
                        return MissionOutput(1, 0.0, CAMERA, direction,
                                             "RAMP:PITCH_10_STRAIGHT_LATCHED")
                    if self.ramp_slow_start_time is not None:
                        return MissionOutput(2, 0.0, CAMERA, direction,
                                             "RAMP:PITCH_10_STAGE_2_HOLD")
                    return MissionOutput(2, 0.0, CAMERA, direction,
                                         "RAMP:STRAIGHT_STAGE_2")
            if not data.camera_path_valid:
                return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)
            return self.camera_output(
                data, data.camera_steering_deg, "RAMP:PATH_FOLLOW", direction)

        if section in INTERSECTIONS:
            if not data.camera_path_valid:
                return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)
            at_line = (data.stop_distance_valid and
                       data.stop_distance_m <= self.stop_distance_m)
            if at_line and self.intersection_stop_time is None:
                self.intersection_stop_time = data.now
            if self.intersection_stop_time is not None and not self.intersection_released:
                stopped_long_enough = data.now - self.intersection_stop_time >= self.minimum_stop_sec
                if stopped_long_enough and data.traffic_green:
                    self.intersection_released = True
                else:
                    return self.stopped("INTERSECTION_WAIT_GREEN", CAMERA, direction)
            return self.camera_output(
                data, data.camera_steering_deg, "INTERSECTION_GO", direction)

        if not data.camera_path_valid:
            return self.stopped("SAFE_STOP:CAMERA_PATH_INVALID", CAMERA, direction)

        if section == 9:
            if data.traffic20_detected:
                if self.traffic20_start_time is None:
                    self.traffic20_start_time = data.now
                if data.now-self.traffic20_start_time >= 2.0:
                    self.traffic20_confirmed = True
            elif not self.traffic20_confirmed:
                self.traffic20_start_time = None
            if not data.speed_plan_valid:
                return self.stopped("SAFE_STOP:SPEED_PLAN_INVALID", CAMERA, direction)
            stage = traffic20_drive_stage(
                data.planned_drive_stage, self.traffic20_confirmed)
            if self.traffic20_confirmed:
                status = f"TRAFFIC20_CONFIRMED:STAGE_{stage}"
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
                "T_COURSE:CAMERA_STANDBY" if section == 7
                else "PARALLEL_PARK:CAMERA_STANDBY",
                direction)

        if section == 13:
            at_line = (data.stop_distance_valid and
                       data.stop_distance_m <= self.stop_distance_m)
            self.final_stopped = self.final_stopped or at_line
            if self.final_stopped:
                return self.stopped("FINISH_STOPPED", CAMERA, direction)
            return self.camera_output(
                data, data.camera_steering_deg, "FINISH_APPROACH", direction)

        if section == 5:
            return self.camera_output(
                data, data.camera_steering_deg,
                "S_CURVE:PURE_PURSUIT", direction)

        names = {1: "START", 3: "CURVE_CENTERING", 12: "SHARP_CURVE"}
        return self.camera_output(
            data, data.camera_steering_deg,
            names.get(section, "CAMERA_CENTERING"), direction)

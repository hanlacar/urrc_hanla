GPS_CONTROL = "GPS_CONTROL"
WAIT_FIRST_GPS = "WAIT_FIRST_GPS"
WAIT_2SEC = "GPS_TO_CAMERA_WAIT_2SEC"
WAIT_CAMERA = "GPS_TO_CAMERA_WAIT_CAMERA"
CAMERA_CONTROL = "CAMERA_CONTROL"


class GpsCameraTransition:
    """Latch a safe GPS-to-camera handoff inside one intersection mode."""

    def __init__(self, wait_sec=2.0, intersection_mode="INTERSECTION"):
        self.wait_sec = float(wait_sec)
        self.intersection_mode = str(intersection_mode).strip().upper()
        self.state = GPS_CONTROL
        self.loss_time = None
        self.gps_seen = False

    def reset(self):
        self.state = GPS_CONTROL
        self.loss_time = None
        self.gps_seen = False

    def update(self, mode, now, gps_ok, camera_ok):
        if str(mode).strip().upper() != self.intersection_mode:
            self.reset()
            return None, "INACTIVE"

        if self.state == CAMERA_CONTROL:
            if camera_ok:
                return "camera", CAMERA_CONTROL
            return "none", WAIT_CAMERA

        if not self.gps_seen:
            if not gps_ok:
                return "none", WAIT_FIRST_GPS
            self.gps_seen = True
            self.state = GPS_CONTROL
            return "gps", GPS_CONTROL

        if self.loss_time is None and gps_ok:
            return "gps", GPS_CONTROL

        if self.loss_time is None:
            self.loss_time = float(now)
            self.state = WAIT_2SEC

        if float(now)-self.loss_time < self.wait_sec:
            self.state = WAIT_2SEC
            return "none", WAIT_2SEC

        if not camera_ok:
            self.state = WAIT_CAMERA
            return "none", WAIT_CAMERA

        self.state = CAMERA_CONTROL
        return "camera", CAMERA_CONTROL

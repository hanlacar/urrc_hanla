import math


class LidarSafetyGate:
    """Front-sector LaserScan safety state machine and drive-command gate."""

    WAIT_SCAN = 'WAIT_SCAN'
    CLEAR = 'CLEAR'
    OBSTACLE_STOP = 'OBSTACLE_STOP'
    LIDAR_TIMEOUT = 'LIDAR_TIMEOUT'

    def __init__(
            self, stop_distance_m=0.50, resume_distance_m=0.60,
            front_sector_deg=30.0, min_valid_range_m=0.05,
            stop_confirm_scans=2, clear_confirm_scans=3,
            scan_timeout_sec=0.5, stop_on_scan_timeout=True):
        self.stop_distance_m = max(0.0, float(stop_distance_m))
        self.resume_distance_m = max(
            self.stop_distance_m, float(resume_distance_m))
        self.front_sector_rad = math.radians(
            min(180.0, max(0.0, float(front_sector_deg))))
        self.min_valid_range_m = max(0.0, float(min_valid_range_m))
        self.stop_confirm_scans = max(1, int(stop_confirm_scans))
        self.clear_confirm_scans = max(1, int(clear_confirm_scans))
        self.scan_timeout_sec = max(0.0, float(scan_timeout_sec))
        self.stop_on_scan_timeout = bool(stop_on_scan_timeout)

        self.state = self.WAIT_SCAN
        self.front_min_distance = math.inf
        self.scan_received = False
        self._stop_count = 0
        self._clear_count = 0

    def _front_minimum(self, scan):
        minimum = math.inf
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if not -self.front_sector_rad <= angle <= self.front_sector_rad:
                continue
            if not math.isfinite(distance) or distance == 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if distance < self.min_valid_range_m:
                continue
            minimum = min(minimum, float(distance))
        return minimum

    def update_scan(self, scan):
        """Consume one scan and return (previous_state, current_state)."""
        previous = self.state
        self.scan_received = True
        self.front_min_distance = self._front_minimum(scan)

        if not math.isfinite(self.front_min_distance):
            self._stop_count = 0
            self._clear_count = 0
            if self.state in (self.WAIT_SCAN, self.LIDAR_TIMEOUT):
                self.state = self.WAIT_SCAN
            return previous, self.state

        if self.state == self.OBSTACLE_STOP:
            self._stop_count = 0
            if self.front_min_distance >= self.resume_distance_m:
                self._clear_count += 1
                if self._clear_count >= self.clear_confirm_scans:
                    self.state = self.CLEAR
                    self._clear_count = 0
            else:
                self._clear_count = 0
            return previous, self.state

        self._clear_count = 0
        if self.front_min_distance <= self.stop_distance_m:
            self._stop_count += 1
            if self._stop_count >= self.stop_confirm_scans:
                self.state = self.OBSTACLE_STOP
                self._stop_count = 0
        else:
            self._stop_count = 0
            self.state = self.CLEAR
        return previous, self.state

    def update_timeout(self, scan_age_sec):
        """Update the watchdog state and return (previous_state, current_state)."""
        previous = self.state
        if self.scan_received and scan_age_sec > self.scan_timeout_sec:
            self.state = self.LIDAR_TIMEOUT
            self._stop_count = 0
            self._clear_count = 0
        return previous, self.state

    @property
    def obstacle_detected(self):
        return self.state == self.OBSTACLE_STOP

    @property
    def should_stop(self):
        if self.state == self.OBSTACLE_STOP:
            return True
        return (
            self.stop_on_scan_timeout
            and self.state in (self.WAIT_SCAN, self.LIDAR_TIMEOUT)
        )

    def filter_command(self, requested_drive, requested_steering):
        """Clamp drive to zero when unsafe without modifying steering."""
        drive = 0.0 if self.should_stop else requested_drive
        return drive, requested_steering

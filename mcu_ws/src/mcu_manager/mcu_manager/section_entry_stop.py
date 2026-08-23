WAIT_SPEED = "SECTION_ENTRY_WAIT_SPEED"
WAIT_STOP = "SECTION_ENTRY_WAIT_ACTUAL_STOP"
HOLD = "SECTION_ENTRY_HOLD_2SEC"
RELEASED = "SECTION_ENTRY_RELEASED"


class SectionEntryStop:
    """Stop on a section change, confirm zero speed, then hold for a duration."""

    def __init__(self, speed_threshold_mps=0.05, stop_confirm_sec=0.2,
                 hold_sec=2.0):
        self.speed_threshold_mps = float(speed_threshold_mps)
        self.stop_confirm_sec = float(stop_confirm_sec)
        self.hold_sec = float(hold_sec)
        self.section = None
        self.active = False
        self.stationary_since = None
        self.hold_since = None

    def set_section(self, section):
        section = int(section)
        if section == self.section:
            return False
        self.section = section
        self.active = True
        self.stationary_since = None
        self.hold_since = None
        return True

    def update(self, now, speed_valid, speed_mps):
        if not self.active:
            return False, RELEASED
        if not speed_valid:
            self.stationary_since = None
            self.hold_since = None
            return True, WAIT_SPEED
        if abs(float(speed_mps)) > self.speed_threshold_mps:
            self.stationary_since = None
            self.hold_since = None
            return True, WAIT_STOP
        if self.stationary_since is None:
            self.stationary_since = float(now)
            return True, WAIT_STOP
        if float(now)-self.stationary_since+1e-9 < self.stop_confirm_sec:
            return True, WAIT_STOP
        if self.hold_since is None:
            self.hold_since = float(now)
        if float(now)-self.hold_since+1e-9 < self.hold_sec:
            return True, HOLD
        self.active = False
        return False, RELEASED

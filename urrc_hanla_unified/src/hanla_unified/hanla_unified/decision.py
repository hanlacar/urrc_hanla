"""ROS-independent source selection for the eleven competition sections."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Candidate:
    speed_mps: float = 0.0
    steering_deg: float = 0.0
    valid: bool = False
    confidence: float = 0.0


@dataclass(frozen=True)
class DecisionInput:
    section: int
    camera: Candidate
    dr: Candidate
    lidar: Candidate
    traffic_go: bool = False
    lidar_stop: bool = False
    pitch_deg: float = 0.0
    ramp_stop_waypoint: bool = False


@dataclass(frozen=True)
class Decision:
    speed_mps: float
    steering_deg: float
    source: str
    stop: bool = False
    slope_hold: bool = False
    reason: str = ""


def requires_emergency_brake(decision: Decision) -> bool:
    """Return true only for an explicit obstacle/safety-stop decision."""
    return decision.stop and decision.source == "lidar_stop"


def limit_rate(previous: float, target: float, rate_per_sec: float,
               elapsed_sec: float) -> float:
    """Limit a command change without changing its eventual target."""
    maximum_change = max(0.0, rate_per_sec) * max(0.0, elapsed_sec)
    return max(previous - maximum_change,
               min(previous + maximum_change, target))


SECTION_PRIORITIES = {
    1: ("dr",),
    2: ("dr",),
    3: ("dr",),
    4: ("dr",), 6: ("dr",), 8: ("dr",),
    # The node invalidates the LiDAR candidate outside an active avoidance
    # manoeuvre, so mode 5 normally follows DR and temporarily yields to LiDAR.
    5: ("lidar", "dr"),
    7: ("lidar", "dr"),
    9: ("dr",),
    10: ("lidar", "dr"),
    11: ("dr",),
}


def decide(data: DecisionInput, camera_confidence_min: float = 0.8) -> Decision:
    if data.section not in SECTION_PRIORITIES:
        return Decision(0.0, 0.0, "none", True, reason="invalid_section")
    if data.lidar_stop:
        reason = "section9_obstacle" if data.section == 9 else "lidar_safety_stop"
        return Decision(0.0, 0.0, "lidar_stop", True, reason=reason)
    # Traffic-light stopping is owned by the DR follower at an exact STOP_LINE
    # waypoint. Gating here would stop as soon as the section number changes,
    # potentially metres before the stop line.
    # TEST COPY:
    # slope + ramp waypoint is diagnostic only.
    # Continue to normal section source selection.
    if (data.section == 2 and abs(data.pitch_deg) >= 5.0
            and data.ramp_stop_waypoint):
        pass

    candidates = {"camera": data.camera, "dr": data.dr, "lidar": data.lidar}
    for source in SECTION_PRIORITIES[data.section]:
        candidate = candidates[source]
        if (candidate.valid and math.isfinite(candidate.speed_mps)
                and math.isfinite(candidate.steering_deg)):
            return Decision(candidate.speed_mps, candidate.steering_deg, source)
    return Decision(0.0, 0.0, "none", True, reason="no_fresh_source")

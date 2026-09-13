"""Center-path generation with explicit degradation modes."""
import numpy as np

INVALID, BOTH_BOUNDARIES, LEFT_ONLY, RIGHT_ONLY, ROAD_ONLY, TEMPORAL_HOLD, TURN_TEMPLATE, LANE_REACQUIRE, OBSTACLE_CORRIDOR = range(9)
MODE_NAMES = {
    INVALID: "INVALID", BOTH_BOUNDARIES: "BOTH", LEFT_ONLY: "Y_ONLY",
    RIGHT_ONLY: "W_ONLY", ROAD_ONLY: "ROAD_ONLY",
    TEMPORAL_HOLD: "TEMPORAL_HOLD", TURN_TEMPLATE: "TURN_TEMPLATE",
    LANE_REACQUIRE: "LANE_REACQUIRE",
    OBSTACLE_CORRIDOR: "OBSTACLE_CORRIDOR",
}


def path_mode_name(mode):
    return MODE_NAMES.get(int(mode), f"UNKNOWN_{int(mode)}")


def _interp(points, xs):
    points = np.asarray(points, dtype=float)
    order = np.argsort(points[:, 0])
    return np.interp(xs, points[order, 0], points[order, 1])


def generate_path(left=None, right=None, road_center=None, lane_width_m=0.8,
                  previous=None, previous_age_s=0., hold_timeout_s=.3,
                  minimum_boundary_clearance_m=0.0):
    left, right = np.asarray(left if left is not None else []).reshape(-1, 2), np.asarray(right if right is not None else []).reshape(-1, 2)
    road = np.asarray(road_center if road_center is not None else []).reshape(-1, 2)
    if len(left) >= 2 and len(right) >= 2:
        lo, hi = max(left[:,0].min(), right[:,0].min()), min(left[:,0].max(), right[:,0].max())
        xs = np.linspace(lo, hi, 30); return np.c_[xs, (_interp(left,xs)+_interp(right,xs))/2], BOTH_BOUNDARIES
    # A single detected marking is a physical vehicle boundary, not a path to
    # follow directly.  Never place the vehicle centre closer than half the
    # lane width or the configured body-clearance distance.
    offset = max(float(lane_width_m)/2, float(minimum_boundary_clearance_m))
    if len(left) >= 2:
        return np.c_[left[:,0], left[:,1]-offset], LEFT_ONLY
    if len(right) >= 2:
        return np.c_[right[:,0], right[:,1]+offset], RIGHT_ONLY
    if len(road) >= 2:
        return road, ROAD_ONLY
    if previous is not None and len(previous) >= 2 and previous_age_s <= hold_timeout_s:
        return np.asarray(previous), TEMPORAL_HOLD
    return np.empty((0,2)), INVALID


def blend_reacquire(template, detected, alpha=0.5):
    template, detected = np.asarray(template), np.asarray(detected)
    xs = detected[:,0]
    return np.c_[xs, (1-alpha)*_interp(template,xs)+alpha*detected[:,1]]


def path_inside_bounds(path, forward_min, forward_max, left, right):
    p=np.asarray(path,float)
    return bool(p.ndim==2 and p.shape[1]==2 and len(p)>0 and np.isfinite(p).all() and np.all((p[:,0]>=forward_min)&(p[:,0]<=forward_max)&(p[:,1]>=-right)&(p[:,1]<=left)))

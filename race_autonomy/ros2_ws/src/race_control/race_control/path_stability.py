"""ROS-independent temporal path-jump measurement."""

import numpy as np


def path_jump_metrics(previous, current, jump_threshold_m=0.25,
                      forward_min_m=0.5, forward_max_m=3.0,
                      sample_count=16):
    """Compare lateral path position at common forward distances.

    Returns (accuracy_0_to_1, median_jump_m, maximum_jump_m, jumped, valid).
    """
    old = np.asarray(previous, dtype=float)
    new = np.asarray(current, dtype=float)
    if old.ndim != 2 or new.ndim != 2 or old.shape[1:] != (2,) or new.shape[1:] != (2,):
        return 0.0, 0.0, 0.0, False, False
    old = old[np.isfinite(old).all(axis=1)]
    new = new[np.isfinite(new).all(axis=1)]
    if len(old) < 2 or len(new) < 2 or jump_threshold_m <= 0.0:
        return 0.0, 0.0, 0.0, False, False
    old = old[np.argsort(old[:, 0])]
    new = new[np.argsort(new[:, 0])]
    lo = max(float(forward_min_m), old[0, 0], new[0, 0])
    hi = min(float(forward_max_m), old[-1, 0], new[-1, 0])
    if hi <= lo:
        return 0.0, 0.0, 0.0, False, False
    xs = np.linspace(lo, hi, max(3, int(sample_count)))
    delta = np.abs(np.interp(xs, old[:, 0], old[:, 1]) -
                   np.interp(xs, new[:, 0], new[:, 1]))
    median = float(np.median(delta))
    maximum = float(np.max(delta))
    # A single noisy far endpoint should not dominate the score. The median
    # represents whole-path stability; maximum remains available diagnostically.
    accuracy = float(np.clip(1.0 - median / float(jump_threshold_m), 0.0, 1.0))
    return accuracy, median, maximum, median >= float(jump_threshold_m), True


def path_spatial_quality(path, required_forward_span_m=1.5,
                         required_near_point_m=1.0,
                         maximum_lateral_step_m=0.15):
    """Score whether one path is usable, not merely stable over time.

    The score combines forward coverage, availability of a near-field point,
    and the 90th-percentile lateral step after sorting by forward distance.
    """
    points = np.asarray(path, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 3 or
            not np.isfinite(points).all() or required_forward_span_m <= 0.0 or
            required_near_point_m <= 0.0 or maximum_lateral_step_m <= 0.0):
        return 0.0, False, {"forward_span_m": 0.0, "nearest_x_m": float("inf"),
                            "lateral_step_p90_m": float("inf")}
    points = points[np.argsort(points[:, 0])]
    # Collapse duplicate X samples before measuring lateral continuity.
    xs = np.unique(points[:, 0])
    ys = np.asarray([np.median(points[points[:, 0] == x, 1]) for x in xs])
    if len(xs) < 3:
        return 0.0, False, {"forward_span_m": 0.0, "nearest_x_m": float(xs[0]),
                            "lateral_step_p90_m": float("inf")}
    span = float(xs[-1] - xs[0])
    nearest = float(xs[0])
    step_p90 = float(np.percentile(np.abs(np.diff(ys)), 90))
    coverage_score = float(np.clip(span / required_forward_span_m, 0.0, 1.0))
    # A path reaching the configured near-field boundary is fully usable.
    # The old expression assigned exactly 0 at that same boundary even though
    # ``valid`` accepted it, which could pin total path accuracy to zero.
    near_score = float(np.clip((2.0*required_near_point_m-nearest) /
                               max(required_near_point_m, 1e-6), 0.0, 1.0))
    smooth_score = float(np.clip(1.0-step_p90/maximum_lateral_step_m, 0.0, 1.0))
    quality = coverage_score * near_score * smooth_score
    valid = bool(span >= required_forward_span_m and
                 nearest <= required_near_point_m and
                 step_p90 <= maximum_lateral_step_m)
    return float(quality), valid, {
        "forward_span_m": span, "nearest_x_m": nearest,
        "lateral_step_p90_m": step_p90}

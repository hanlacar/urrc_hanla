"""Evidence-based confidence for a metric camera path."""

import math
import numpy as np


def _clip01(value):
    return float(np.clip(float(value), 0.0, 1.0))


def _forward_span(points):
    if points is None:
        return 0.0
    points = np.asarray(points, dtype=float)
    return 0.0 if len(points) < 2 else float(np.ptp(points[:, 0]))


def _path_lateral_difference(previous, current):
    if previous is None or current is None:
        return 0.0, 0.0, False
    previous = np.asarray(previous, dtype=float)
    current = np.asarray(current, dtype=float)
    if len(previous) < 2 or len(current) < 2:
        return 0.0, 0.0, False
    lo = max(float(previous[:, 0].min()), float(current[:, 0].min()))
    hi = min(float(previous[:, 0].max()), float(current[:, 0].max()))
    if hi-lo < 0.5:
        return 0.0, 0.0, False
    xs = np.linspace(lo, hi, 20)
    old = previous[np.argsort(previous[:, 0])]
    new = current[np.argsort(current[:, 0])]
    delta = np.abs(np.interp(xs, new[:, 0], new[:, 1]) -
                   np.interp(xs, old[:, 0], old[:, 1]))
    return float(np.median(delta)), float(abs(np.median(
        np.interp(xs, new[:, 0], new[:, 1]) -
        np.interp(xs, old[:, 0], old[:, 1])))), True


def _resampled_max_curvature(path, spacing_m=0.4):
    """Robust 3-point curvature after uniform arc-length resampling."""
    points = np.asarray(path, dtype=float)
    if len(points) < 3:
        return 0.0
    delta = np.diff(points, axis=0)
    segment = np.linalg.norm(delta, axis=1)
    keep = np.r_[True, segment > 1e-6]
    points = points[keep]
    if len(points) < 3:
        return 0.0
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if distance[-1] < 2.0*spacing_m:
        return 0.0
    samples = np.arange(0.0, distance[-1]+1e-9, max(0.05, spacing_m))
    sampled = np.column_stack((np.interp(samples, distance, points[:, 0]),
                               np.interp(samples, distance, points[:, 1])))
    values = []
    for a, b, c in zip(sampled, sampled[1:], sampled[2:]):
        ab, bc, ac = (float(np.linalg.norm(b-a)), float(np.linalg.norm(c-b)),
                      float(np.linalg.norm(c-a)))
        denominator = ab*bc*ac
        if denominator <= 1e-9:
            continue
        twice_area = abs((b[0]-a[0])*(c[1]-a[1]) -
                         (b[1]-a[1])*(c[0]-a[0]))
        values.append(2.0*twice_area/denominator)
    return max(values, default=0.0)


def path_confidence(road, left, right, path, previous=None,
                    desired_forward_span_m=3.0):
    """Return a 0..1 score and the individual evidence values.

    The score measures mask support, boundary support, path extent, temporal
    agreement and geometric plausibility. It intentionally does not depend on
    the selected path mode alone.
    """
    path = np.asarray(path, dtype=float)
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    road_ratio = (float(np.count_nonzero(road))/float(road.size)
                  if np.asarray(road).size else 0.0)
    road_score = _clip01(road_ratio/0.18)
    left_span, right_span = _forward_span(left), _forward_span(right)
    lane_span = max(left_span, right_span)
    lane_length_score = _clip01(lane_span/desired_forward_span_m)
    boundary_score = 1.0 if left_span > 0.5 and right_span > 0.5 else (
        0.55 if lane_span > 0.5 else 0.0)
    path_span = _forward_span(path)
    path_length_score = _clip01(path_span/desired_forward_span_m)
    lateral_delta, center_shift, comparable = _path_lateral_difference(
        previous, path)
    temporal_score = (1.0 if previous is None else
                      (_clip01(1.0-lateral_delta/0.30) if comparable else 0.0))
    center_score = (1.0 if previous is None else
                    (_clip01(1.0-center_shift/0.20) if comparable else 0.0))
    curvature = _resampled_max_curvature(path) if len(path) >= 3 else float("inf")
    curvature_score = (_clip01(1.0-(curvature-0.20)/1.30)
                       if np.isfinite(curvature) else 0.0)
    components = {
        "road_area": road_score,
        "lane_length": lane_length_score,
        "both_boundaries": boundary_score,
        "path_length": path_length_score,
        "previous_path": temporal_score,
        "maximum_curvature": curvature_score,
        "center_shift": center_score,
    }
    weights = {
        "road_area": 0.15, "lane_length": 0.15,
        "both_boundaries": 0.15, "path_length": 0.15,
        "previous_path": 0.20, "maximum_curvature": 0.10,
        "center_shift": 0.10,
    }
    score = sum(components[name]*weight for name, weight in weights.items())
    diagnostics = dict(components)
    diagnostics.update({
        "road_ratio": road_ratio, "left_span_m": left_span,
        "right_span_m": right_span, "path_span_m": path_span,
        "lateral_delta_m": lateral_delta, "center_shift_m": center_shift,
        "max_curvature_per_m": curvature,
    })
    return _clip01(score), diagnostics


def validate_candidate_confidence(score, metrics, maximum_shift_m=0.25,
                                  maximum_curvature_per_m=1.0,
                                  minimum_path_span_m=1.0,
                                  rejected_confidence_cap=0.35):
    """Apply explicit geometric gates before a candidate can replace history."""
    reasons = []
    if float(metrics.get("lateral_delta_m", 0.0)) > float(maximum_shift_m):
        reasons.append("PATH_SHIFT")
    if float(metrics.get("max_curvature_per_m", float("inf"))) > float(
            maximum_curvature_per_m):
        reasons.append("CURVATURE")
    if float(metrics.get("path_span_m", 0.0)) < float(minimum_path_span_m):
        reasons.append("PATH_TOO_SHORT")
    adjusted = min(float(score), float(rejected_confidence_cap)) if reasons else float(score)
    return _clip01(adjusted), reasons

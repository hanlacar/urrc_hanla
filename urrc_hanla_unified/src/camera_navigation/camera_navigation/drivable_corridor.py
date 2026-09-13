"""Vehicle-clearance corridor extraction from a binary BEV road mask."""
import cv2
import numpy as np


def clearance_corridor_path(road_mask, bev, vehicle_width_m,
                            safety_margin_m=0.15, row_step=5,
                            previous_path=None, maximum_lateral_step_m=0.10):
    road = (np.asarray(road_mask) > 0).astype(np.uint8)
    if road.ndim != 2 or not np.any(road):
        return np.empty((0, 2), dtype=float)

    clearance_m = float(vehicle_width_m) * 0.5 + float(safety_margin_m)
    distance_px = cv2.distanceTransform(road, cv2.DIST_L2, 5)
    safe = distance_px * float(bev.resolution) >= clearance_m
    points = []
    previous_col = float(bev.ground_to_bev(bev.forward_min, 0.0)[0])

    # Near to far so each choice stays connected to the vehicle's current side.
    for row in range(safe.shape[0] - 1, -1, -max(1, int(row_step))):
        cols = np.flatnonzero(safe[row])
        if not len(cols):
            continue
        splits = np.where(np.diff(cols) > 1)[0] + 1
        runs = [run for run in np.split(cols, splits) if len(run)]
        # Pick the maximum-clearance point of each passage.  This is the
        # geometric middle between an obstacle wall and the painted boundary,
        # even when perspective/curvature makes the free run asymmetric.
        centers = []
        for run in runs:
            clearances = distance_px[row, run]
            best = run[np.isclose(clearances, np.max(clearances))]
            centers.append(float(np.median(best)))
        center = min(centers, key=lambda value: abs(value - previous_col))
        x, y = bev.bev_to_ground(center, row)
        points.append((x, y))
        previous_col = center

    path = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(path):
        return path
    path = path[np.argsort(path[:, 0])]
    previous = np.asarray(previous_path if previous_path is not None else [], dtype=float).reshape(-1, 2)
    if len(previous) >= 2 and np.isfinite(previous).all():
        order = np.argsort(previous[:, 0])
        prior_y = np.interp(path[:, 0], previous[order, 0], previous[order, 1])
        step = max(0.0, float(maximum_lateral_step_m))
        limited = np.clip(path[:, 1], prior_y-step, prior_y+step)
        # Smoothing must never pull a path back into an inflated obstacle.
        for index, (x, candidate_y) in enumerate(zip(path[:, 0], limited)):
            col, row = bev.ground_to_bev(float(x), float(candidate_y))
            ri, ci = int(round(row)), int(round(col))
            if (0 <= ri < safe.shape[0] and 0 <= ci < safe.shape[1]
                    and safe[ri, ci]):
                path[index, 1] = candidate_y
    return path

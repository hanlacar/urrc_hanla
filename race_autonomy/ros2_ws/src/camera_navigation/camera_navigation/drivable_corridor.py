"""Vehicle-clearance corridor extraction from a binary BEV road mask."""
import cv2
import numpy as np


def clearance_corridor_path(road_mask, bev, vehicle_width_m,
                            safety_margin_m=0.15, row_step=5):
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
        centers = [0.5 * (float(run[0]) + float(run[-1])) for run in runs]
        center = min(centers, key=lambda value: abs(value - previous_col))
        x, y = bev.bev_to_ground(center, row)
        points.append((x, y))
        previous_col = center

    path = np.asarray(points, dtype=float).reshape(-1, 2)
    return path[np.argsort(path[:, 0])] if len(path) else path

"""Extract metric boundary samples from BEV masks."""
import numpy as np


def extract_boundary(mask, bev, min_pixels=5):
    if mask is None or np.count_nonzero(mask) < min_pixels:
        return np.empty((0, 2))
    points = []
    for row in range(mask.shape[0]):
        cols = np.flatnonzero(mask[row] > 0)
        if cols.size:
            points.append(bev.bev_to_ground(float(np.median(cols)), row))
    return np.asarray(points, dtype=float)

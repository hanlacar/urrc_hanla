"""Spatial and temporal filtering for camera-generated paths."""

import numpy as np


def _odd_window(value, length):
    window = max(1, min(int(value), int(length)))
    if window % 2 == 0:
        window = max(1, window-1)
    return window


def _median_filter(values, window):
    window = _odd_window(window, len(values))
    if window <= 1:
        return values.copy()
    radius = window//2
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.asarray([
        np.median(padded[index:index+window])
        for index in range(len(values))
    ])


def stabilize_path(current, previous=None, spatial_window=7,
                   new_path_weight=0.35, maximum_lateral_shift_m=0.15):
    """Remove row-wise spikes and limit frame-to-frame lateral path jumps.

    X coordinates are retained. Temporal filtering is applied only where the
    old and new paths overlap, so a newly visible near/far segment is not
    extrapolated from stale data.
    """
    path = np.asarray(current, dtype=float).reshape(-1, 2)
    if len(path) < 3 or not np.isfinite(path).all():
        return path.copy()
    order = np.argsort(path[:, 0])
    path = path[order]
    xs, indices = np.unique(path[:, 0], return_index=True)
    ys = path[indices, 1]
    ys = _median_filter(ys, spatial_window)

    old = np.asarray(previous if previous is not None else [], dtype=float).reshape(-1, 2)
    if len(old) >= 2 and np.isfinite(old).all():
        old = old[np.argsort(old[:, 0])]
        overlap = (xs >= old[0, 0]) & (xs <= old[-1, 0])
        if np.any(overlap):
            prior = np.interp(xs[overlap], old[:, 0], old[:, 1])
            weight = float(np.clip(new_path_weight, 0.0, 1.0))
            blended = prior + weight*(ys[overlap]-prior)
            limit = max(0.0, float(maximum_lateral_shift_m))
            ys[overlap] = np.clip(blended, prior-limit, prior+limit)
    return np.column_stack((xs, ys))

import math

import numpy as np


def centerline_from_mask(mask, band_count=16, minimum_pixels=20, top_ratio=0.35):
    """Extract near-to-far center pixels from a binary drivable-area mask."""
    if mask is None or mask.ndim != 2 or mask.size == 0:
        return []
    height, _ = mask.shape
    top = int(height * top_ratio)
    band_height = max((height - top) // band_count, 1)
    points = []
    for index in range(band_count):
        bottom = height - index * band_height
        upper = max(top, bottom - band_height)
        ys, xs = np.nonzero(mask[upper:bottom])
        if xs.size < minimum_pixels:
            continue
        points.append((int(round((float(xs.min()) + float(xs.max())) * 0.5)), (upper + bottom - 1) // 2))
    return points


def pixels_to_vehicle_path(points, width, height, near_m=0.5, far_m=6.0,
                           near_span_m=3.0, far_span_m=1.2,
                           top_ratio=0.35):
    """Initial trapezoidal projection; parameters must be physically calibrated."""
    top = height * top_ratio
    usable = max((height - 1) - top, 1.0)
    path = []
    for pixel_x, pixel_y in points:
        ratio = min(max(((height - 1) - pixel_y) / usable, 0.0), 1.0)
        forward = near_m + ratio * (far_m - near_m)
        span = near_span_m + ratio * (far_span_m - near_span_m)
        lateral = ((pixel_x - width * 0.5) / max(width * 0.5, 1.0)) * span
        path.append([round(forward, 4), round(lateral, 4)])
    return path


def pixels_to_ground_path(
    points, fx, fy, cx, cy, camera_height_m, camera_pitch_down_deg,
    camera_forward_offset_m=0.0, camera_lateral_offset_m=0.0,
    minimum_forward_m=0.2, maximum_forward_m=10.0,
):
    """Project rectified image pixels onto flat ground in vehicle coordinates."""
    values = (
        fx, fy, cx, cy, camera_height_m, camera_pitch_down_deg,
        camera_forward_offset_m, camera_lateral_offset_m,
        minimum_forward_m, maximum_forward_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("ground projection parameters must be finite")
    if fx <= 0.0 or fy <= 0.0 or camera_height_m <= 0.0:
        raise ValueError("fx, fy, and camera_height_m must be positive")
    if not 0.0 < camera_pitch_down_deg < 90.0:
        raise ValueError("camera_pitch_down_deg must be between 0 and 90")
    if minimum_forward_m < 0.0 or maximum_forward_m <= minimum_forward_m:
        raise ValueError("invalid forward projection limits")

    pitch = math.radians(float(camera_pitch_down_deg))
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    path = []
    for pixel_x, pixel_y in points:
        ray_right = (float(pixel_x) - float(cx)) / float(fx)
        ray_down = (float(pixel_y) - float(cy)) / float(fy)
        ray_forward = cos_pitch - sin_pitch * ray_down
        ray_up = -sin_pitch - cos_pitch * ray_down
        if ray_up >= -1e-9 or ray_forward <= 0.0:
            continue
        scale = float(camera_height_m) / -ray_up
        forward = float(camera_forward_offset_m) + scale * ray_forward
        lateral = float(camera_lateral_offset_m) + scale * ray_right
        if minimum_forward_m <= forward <= maximum_forward_m:
            path.append([round(forward, 4), round(lateral, 4)])
    return path


def quaternion_to_pitch_deg(x, y, z, w):
    """Return ROS REP-103 pitch in degrees from a unit quaternion."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion values must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = (value / norm for value in values)
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    return math.degrees(math.asin(sin_pitch))

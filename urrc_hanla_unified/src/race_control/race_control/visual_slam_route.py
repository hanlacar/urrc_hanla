"""Geometry and persistence helpers for visual-SLAM route following."""

import json
import math
import os
import tempfile


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def align_route_to_pose(points, target_x, target_y, target_yaw):
    """Rigidly place the route's first pose at a requested map-frame pose."""
    if not points:
        return []
    origin_x, origin_y, origin_yaw = points[0]
    rotation = wrap_angle(target_yaw - origin_yaw)
    cosine, sine = math.cos(rotation), math.sin(rotation)
    aligned = []
    for x, y, yaw in points:
        dx, dy = x - origin_x, y - origin_y
        aligned.append((
            target_x + cosine * dx - sine * dy,
            target_y + sine * dx + cosine * dy,
            wrap_angle(yaw + rotation),
        ))
    return aligned


def nearest_stamp_skew(stamps, target):
    if not stamps or target is None or not math.isfinite(target):
        return float("inf")
    return min(abs(float(stamp) - float(target)) for stamp in stamps)


def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def should_sample(points, x, y, yaw, minimum_distance_m, minimum_yaw_rad):
    if not points:
        return True
    previous = points[-1]
    distance = math.hypot(x - previous[0], y - previous[1])
    heading = abs(wrap_angle(yaw - previous[2]))
    return distance >= minimum_distance_m or heading >= minimum_yaw_rad


def save_route(path, points, frame_id="map"):
    """Atomically save x/y/yaw route points as a human-readable JSON file."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    payload = {
        "format": "urrc_visual_slam_route_v1",
        "frame_id": frame_id,
        "points": [
            {"x": float(x), "y": float(y), "yaw": float(yaw)}
            for x, y, yaw in points
        ],
    }
    fd, temporary = tempfile.mkstemp(prefix=".route-", suffix=".tmp",
                                     dir=directory, text=True)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(payload, output, ensure_ascii=False, indent=2)
            output.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_route(path):
    with open(path, "r", encoding="utf-8") as source:
        payload = json.load(source)
    if payload.get("format") != "urrc_visual_slam_route_v1":
        raise ValueError("unsupported route format")
    points = []
    for item in payload.get("points", []):
        point = (float(item["x"]), float(item["y"]), float(item.get("yaw", 0.0)))
        if not all(math.isfinite(value) for value in point):
            raise ValueError("route contains a non-finite coordinate")
        points.append(point)
    if len(points) < 2:
        raise ValueError("route must contain at least two points")
    return str(payload.get("frame_id", "map")), points


def nearest_index(points, x, y, start=0, backward=10, forward=250):
    first = max(0, int(start) - int(backward))
    last = min(len(points), int(start) + int(forward) + 1)
    if first >= last:
        first, last = 0, len(points)
    return min(range(first, last),
               key=lambda index: ((points[index][0] - x) ** 2
                                  + (points[index][1] - y) ** 2))


def route_to_base(points, start, vehicle_x, vehicle_y, vehicle_yaw,
                  forward_min_m, forward_max_m):
    cosine, sine = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
    local = []
    for map_x, map_y, _ in points[start:]:
        dx, dy = map_x - vehicle_x, map_y - vehicle_y
        x = cosine * dx + sine * dy
        y = -sine * dx + cosine * dy
        if x >= forward_min_m and x <= forward_max_m:
            local.append((x, y))
        elif x > forward_max_m and local:
            break
    return local


def interpolate_y(path, x):
    ordered = sorted(path)
    if not ordered or x < ordered[0][0] or x > ordered[-1][0]:
        return None
    for (x0, y0), (x1, y1) in zip(ordered, ordered[1:]):
        if x0 <= x <= x1:
            if abs(x1 - x0) < 1.0e-9:
                return 0.5 * (y0 + y1)
            ratio = (x - x0) / (x1 - x0)
            return y0 + ratio * (y1 - y0)
    return ordered[-1][1]


def blend_bev_correction(route, bev, gain, maximum_correction_m):
    """Pull a SLAM route toward the BEV lane centre with bounded lateral error."""
    output, corrections = [], []
    for x, route_y in route:
        bev_y = interpolate_y(bev, x)
        if bev_y is None:
            output.append((x, route_y))
            continue
        error = max(-maximum_correction_m,
                    min(maximum_correction_m, bev_y - route_y))
        correction = gain * error
        output.append((x, route_y + correction))
        corrections.append(correction)
    mean = sum(corrections) / len(corrections) if corrections else 0.0
    return output, mean

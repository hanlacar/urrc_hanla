import math


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def dynamic_lookahead(speed_mps, minimum_m, gain_s, maximum_m):
    """Return Ld = Lmin + Kv, bounded to the configured range."""
    return clamp(minimum_m + gain_s * abs(speed_mps), minimum_m, maximum_m)


def rpm_to_speed_mps(rpm, wheel_radius_m, rpm_per_wheel_rpm=1.0):
    if rpm_per_wheel_rpm <= 0.0 or wheel_radius_m <= 0.0:
        return 0.0
    wheel_rpm = float(rpm) / rpm_per_wheel_rpm
    return wheel_rpm * (2.0 * math.pi * wheel_radius_m) / 60.0


def select_lookahead_point(points, lookahead_m):
    """Select the first forward path point at least lookahead_m from the axle."""
    valid = [
        (float(point[0]), float(point[1]))
        for point in points
        if len(point) >= 2 and math.isfinite(point[0])
        and math.isfinite(point[1]) and point[0] > 0.0
    ]
    if not valid:
        return None
    for point in valid:
        if math.hypot(*point) >= lookahead_m:
            return point
    return valid[-1]


def steering_angle_deg(point, wheelbase_m, maximum_deg):
    """Pure-pursuit steering for (forward, lateral-right) coordinates."""
    if point is None:
        return 0.0
    forward, lateral = point
    distance_sq = forward * forward + lateral * lateral
    if distance_sq <= 1e-9:
        return 0.0
    curvature = 2.0 * lateral / distance_sq
    steering = math.degrees(math.atan(wheelbase_m * curvature))
    return clamp(steering, -abs(maximum_deg), abs(maximum_deg))

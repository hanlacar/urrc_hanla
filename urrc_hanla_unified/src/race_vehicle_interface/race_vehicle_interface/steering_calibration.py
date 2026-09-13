import math


def wrapped_angle_delta_deg(end_deg, start_deg):
    """Return the shortest signed angular displacement in degrees."""
    return (float(end_deg) - float(start_deg) + 180.0) % 360.0 - 180.0


def straight_run_trim_deg(
    yaw_delta_deg,
    distance_m,
    wheelbase_m,
    yaw_to_steering_sign=1.0,
    maximum_trim_deg=5.0,
):
    """Estimate the opposite steering trim from a nominally straight run.

    Positive IMU yaw is left/CCW while this vehicle's positive steering command
    is right, so the installed default sign is +1.
    """
    values = (
        yaw_delta_deg, distance_m, wheelbase_m,
        yaw_to_steering_sign, maximum_trim_deg,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("calibration values must be finite")
    if distance_m <= 0.0 or wheelbase_m <= 0.0 or maximum_trim_deg <= 0.0:
        raise ValueError("distance, wheelbase and maximum trim must be positive")
    estimate = math.degrees(math.atan(
        float(wheelbase_m) * math.radians(float(yaw_delta_deg)) / float(distance_m)
    ))
    estimate *= float(yaw_to_steering_sign)
    return max(-float(maximum_trim_deg), min(float(maximum_trim_deg), estimate))

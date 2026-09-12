"""ROS-independent Ackermann trajectory generation and obstacle scoring."""

import math
import numpy as np


def generate_trajectory(steering_deg, wheelbase_m, horizon_m, step_m):
    """Return columns x, y, yaw for a constant-steering bicycle trajectory."""
    steering = math.radians(float(steering_deg))
    curvature = math.tan(steering)/float(wheelbase_m)
    distances = np.arange(0.0, float(horizon_m)+0.5*step_m, float(step_m))
    if abs(curvature) < 1.0e-8:
        return np.column_stack((distances, np.zeros_like(distances),
                                np.zeros_like(distances)))
    yaw = distances*curvature
    x = np.sin(yaw)/curvature
    y = (1.0-np.cos(yaw))/curvature
    return np.column_stack((x, y, yaw))


def trajectory_clearance(trajectory, obstacles, vehicle_length_m,
                         vehicle_width_m, safety_margin_m):
    """Minimum footprint clearance; negative/zero means collision."""
    obstacles = np.asarray(obstacles, dtype=float).reshape((-1, 2))
    if not len(obstacles):
        return math.inf
    half_length = 0.5*float(vehicle_length_m)+float(safety_margin_m)
    half_width = 0.5*float(vehicle_width_m)+float(safety_margin_m)
    minimum = math.inf
    for x, y, yaw in np.asarray(trajectory):
        dx, dy = obstacles[:, 0]-x, obstacles[:, 1]-y
        c, s = math.cos(yaw), math.sin(yaw)
        longitudinal = c*dx+s*dy
        lateral = -s*dx+c*dy
        outside_x = np.maximum(np.abs(longitudinal)-half_length, 0.0)
        outside_y = np.maximum(np.abs(lateral)-half_width, 0.0)
        clearance = np.hypot(outside_x, outside_y)
        inside = ((np.abs(longitudinal) <= half_length) &
                  (np.abs(lateral) <= half_width))
        if np.any(inside):
            return 0.0
        minimum = min(minimum, float(np.min(clearance)))
    return minimum


def reference_error(trajectory, reference_path):
    reference = np.asarray(reference_path, dtype=float).reshape((-1, 2))
    if not len(reference):
        return 0.0
    points = np.asarray(trajectory)[:, :2]
    squared = np.sum((points[:, None, :]-reference[None, :, :])**2, axis=2)
    return float(np.mean(np.sqrt(np.min(squared, axis=1))))


def choose_trajectory(steering_candidates_deg, obstacles, reference_path,
                      wheelbase_m=0.73, horizon_m=2.5, step_m=0.1,
                      vehicle_length_m=1.30, vehicle_width_m=0.77,
                      safety_margin_m=0.15, previous_steering_deg=0.0):
    """Reject collisions then score clearance, reference fit and smoothness."""
    best = None
    diagnostics = []
    for steering in steering_candidates_deg:
        trajectory = generate_trajectory(
            steering, wheelbase_m, horizon_m, step_m)
        clearance = trajectory_clearance(
            trajectory, obstacles, vehicle_length_m, vehicle_width_m,
            safety_margin_m)
        collision = clearance <= 0.0
        error = reference_error(trajectory, reference_path)
        # Clearance is capped so an extremely distant point cannot dominate.
        score = (2.0*min(clearance, 1.0)-1.4*error
                 -0.015*abs(float(steering))
                 -0.025*abs(float(steering)-float(previous_steering_deg)))
        diagnostics.append((float(steering), collision, clearance, error, score))
        if not collision and (best is None or score > best[0]):
            best = (score, float(steering), trajectory, clearance, error)
    return best, diagnostics


def pure_pursuit_steering(trajectory, lookahead_m, wheelbase_m,
                          maximum_steering_deg):
    points = np.asarray(trajectory)
    distance = np.hypot(points[:, 0], points[:, 1])
    index = int(np.argmin(np.abs(distance-float(lookahead_m))))
    x, y = points[index, :2]
    ld2 = max(1.0e-6, x*x+y*y)
    steering = math.degrees(math.atan2(2.0*float(wheelbase_m)*y, ld2))
    return float(np.clip(steering, -maximum_steering_deg,
                         maximum_steering_deg))

"""ROS-independent path-curvature speed planning."""

import math


def resample_polyline(points, spacing_m):
    """Return points separated by approximately ``spacing_m`` along the path."""
    clean = []
    for point in points:
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if not clean or math.hypot(x-clean[-1][0], y-clean[-1][1]) > 1e-9:
            clean.append((x, y))
    if len(clean) < 2:
        return clean

    spacing = max(0.05, float(spacing_m))
    sampled = [clean[0]]
    target = spacing
    travelled = 0.0
    for start, end in zip(clean, clean[1:]):
        dx, dy = end[0]-start[0], end[1]-start[1]
        segment = math.hypot(dx, dy)
        if segment <= 1e-9:
            continue
        while travelled + segment >= target:
            ratio = (target-travelled) / segment
            sampled.append((start[0]+ratio*dx, start[1]+ratio*dy))
            target += spacing
        travelled += segment
    if math.hypot(clean[-1][0]-sampled[-1][0], clean[-1][1]-sampled[-1][1]) >= spacing*0.5:
        sampled.append(clean[-1])
    return sampled


def three_point_curvature(a, b, c):
    """Unsigned circle curvature through three points, in 1/m."""
    ab = math.hypot(b[0]-a[0], b[1]-a[1])
    bc = math.hypot(c[0]-b[0], c[1]-b[1])
    ac = math.hypot(c[0]-a[0], c[1]-a[1])
    denominator = ab * bc * ac
    if denominator <= 1e-9:
        return 0.0
    twice_area = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
    return 2.0 * twice_area / denominator


def maximum_path_curvature(points, spacing_m=0.4):
    sampled = resample_polyline(points, spacing_m)
    if len(sampled) < 3:
        return None
    return max(three_point_curvature(*sampled[index:index+3])
               for index in range(len(sampled)-2))


def stage_for_curvature(curvature, slow_threshold=0.25, stop_threshold=0.60,
                        cruise_stage=2, slow_stage=1):
    if curvature is None or not math.isfinite(float(curvature)):
        return 0
    value = abs(float(curvature))
    if value >= float(stop_threshold):
        return 0
    if value >= float(slow_threshold):
        return int(slow_stage)
    return int(cruise_stage)


class CurvatureStagePlanner:
    """Stop once on entry to a sharp curve, then continue at the slow stage."""

    def __init__(self):
        self.high_curvature_active = False
        self.stop_started_at = None

    def reset(self):
        self.high_curvature_active = False
        self.stop_started_at = None

    def update(self, curvature, now, slow_threshold=0.25, stop_threshold=0.60,
               stop_hold_sec=1.0, cruise_stage=2, slow_stage=1):
        if curvature is None or not math.isfinite(float(curvature)):
            self.reset()
            return 0, "INVALID_PATH_STOP"

        value = abs(float(curvature))
        if value >= float(stop_threshold):
            if not self.high_curvature_active:
                self.high_curvature_active = True
                self.stop_started_at = float(now)
            if float(now)-self.stop_started_at < max(0.0, float(stop_hold_sec)):
                return 0, "HIGH_CURVATURE_STOP"
            return int(slow_stage), "HIGH_CURVATURE_STAGE1"

        self.reset()
        stage = stage_for_curvature(
            value, slow_threshold, stop_threshold, cruise_stage, slow_stage)
        return (stage, "CURVE_SLOW" if stage == int(slow_stage) else "CRUISE")

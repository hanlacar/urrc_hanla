"""Curvature-safe reference geometry derived from route waypoint positions.

CSV yaw remains source metadata.  Vehicle reference heading is derived from
the smoothed XY curve, avoiding the outgoing-chord-as-Hermite-tangent failure.
"""

import bisect
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ReferencePoint:
    x: float
    y: float
    yaw: float
    source_position: float


@dataclass(frozen=True)
class ReferenceGeometryReport:
    smoothing_strength: float
    input_waypoint_count: int
    output_point_count: int
    raw_length_m: float
    reference_length_m: float
    length_change_m: float
    max_waypoint_displacement_m: float
    rms_waypoint_displacement_m: float
    adjusted_waypoint_count: int
    validation_interval_m: float
    max_validation_spacing_m: float
    max_curvature: float
    max_steering_rad: float
    over_limit_sample_count: int
    peak_s_m: float
    heading_continuous: bool
    curvature_continuous: bool


class ReferenceGeometryError(ValueError):
    pass


def _xy(point):
    return float(point.x), float(point.y)


def _distance(first, second):
    return math.hypot(second[0]-first[0], second[1]-first[1])


def _polyline_length(points):
    return sum(_distance(first, second)
               for first, second in zip(points, points[1:]))


def _smoothed_controls(points, strength):
    result = [points[0]]
    center = 1.0-2.0*strength
    for previous, current, following in zip(points, points[1:], points[2:]):
        result.append((
            strength*previous[0]+center*current[0]+strength*following[0],
            strength*previous[1]+center*current[1]+strength*following[1]))
    result.append(points[-1])
    return tuple(result)


def _control(controls, index):
    if index < 0:
        return (2.0*controls[0][0]-controls[1][0],
                2.0*controls[0][1]-controls[1][1])
    if index >= len(controls):
        return (2.0*controls[-1][0]-controls[-2][0],
                2.0*controls[-1][1]-controls[-2][1])
    return controls[index]


def _bspline_point(controls, segment, ratio):
    p0, p1, p2, p3 = (
        _control(controls, segment-1), _control(controls, segment),
        _control(controls, segment+1), _control(controls, segment+2))
    t2, t3 = ratio*ratio, ratio*ratio*ratio
    weights = (
        (1.0-3.0*ratio+3.0*t2-t3)/6.0,
        (4.0-6.0*t2+3.0*t3)/6.0,
        (1.0+3.0*ratio+3.0*t2-3.0*t3)/6.0,
        t3/6.0)
    return (sum(weight*point[0] for weight, point in zip(
                weights, (p0, p1, p2, p3))),
            sum(weight*point[1] for weight, point in zip(
                weights, (p0, p1, p2, p3))))


def _bspline_derivative(controls, segment, ratio):
    points = tuple(_control(controls, segment+offset)
                   for offset in (-1, 0, 1, 2))
    t2 = ratio*ratio
    weights = (
        -0.5*(1.0-ratio)*(1.0-ratio),
        -2.0*ratio+1.5*t2,
        0.5+ratio-1.5*t2,
        0.5*t2)
    return (sum(weight*point[0] for weight, point in zip(weights, points)),
            sum(weight*point[1] for weight, point in zip(weights, points)))


def _sample_bspline(points, strength, interval):
    controls = _smoothed_controls(points, strength)
    sampled = []
    source_positions = []
    for segment in range(len(controls)-1):
        local = tuple(_control(controls, segment+offset)
                      for offset in (-1, 0, 1, 2))
        scale = max(_distance(first, second)
                    for first, second in zip(local, local[1:]))
        # The 1.5 derivative bound keeps direct curve samples no farther apart
        # than the requested interval without polygonal resampling artifacts.
        count = max(2, math.ceil(1.5*scale/interval))
        for index in range(count):
            ratio = index/count
            sampled.append(_bspline_point(controls, segment, ratio))
            source_positions.append(segment+ratio)
    sampled.append(points[-1])
    source_positions.append(float(len(points)-1))
    return tuple(sampled), tuple(source_positions)


def _signed_curvatures(points):
    if len(points) < 3:
        return (0.0,)*len(points)
    values = []
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = _distance(first, middle)
        b = _distance(middle, last)
        c = _distance(first, last)
        cross = ((middle[0]-first[0])*(last[1]-first[1]) -
                 (middle[1]-first[1])*(last[0]-first[0]))
        values.append(0.0 if a*b*c <= 1.0e-15 else 2.0*cross/(a*b*c))
    return (values[0], *values, values[-1])


def _point_segment_distance(point, first, second):
    vx, vy = second[0]-first[0], second[1]-first[1]
    denominator = vx*vx+vy*vy
    ratio = 0.0 if denominator <= 1.0e-15 else max(
        0.0, min(1.0, ((point[0]-first[0])*vx +
                       (point[1]-first[1])*vy)/denominator))
    return math.hypot(
        point[0]-(first[0]+ratio*vx), point[1]-(first[1]+ratio*vy))


def _waypoint_displacements(waypoints, curve, source_positions):
    # Source-position locality bounds this to a small neighbourhood rather
    # than an O(N*M) global search on long recorded routes.
    result = []
    for index, point in enumerate(waypoints):
        estimate = bisect.bisect_left(source_positions, float(index))
        low, high = max(0, estimate-120), min(len(curve)-1, estimate+120)
        result.append(min(
            _point_segment_distance(point, curve[item], curve[item+1])
            for item in range(low, max(low+1, high))))
    return tuple(result)


def _with_yaws(points, source_positions):
    result = []
    for index, point in enumerate(points):
        previous = points[max(0, index-1)]
        following = points[min(len(points)-1, index+1)]
        yaw = math.atan2(following[1]-previous[1],
                         following[0]-previous[0])
        result.append(ReferencePoint(
            point[0], point[1], yaw, source_positions[index]))
    return tuple(result)


def _with_bspline_yaws(points, source_positions, controls):
    result = []
    last_segment = len(controls)-2
    for point, position in zip(points, source_positions):
        segment = min(last_segment, int(math.floor(position)))
        ratio = 1.0 if position >= len(controls)-1 else position-segment
        dx, dy = _bspline_derivative(controls, segment, ratio)
        if math.hypot(dx, dy) <= 1.0e-15:
            raise ReferenceGeometryError('reference has a zero tangent')
        result.append(ReferencePoint(
            point[0], point[1], math.atan2(dy, dx), position))
    return tuple(result)


def _sample_polyline(points, interval):
    sampled, positions = [], []
    for segment, (first, second) in enumerate(zip(points, points[1:])):
        count = max(1, math.ceil(_distance(first, second)/interval))
        for index in range(count):
            ratio = index/count
            sampled.append((first[0]+ratio*(second[0]-first[0]),
                            first[1]+ratio*(second[1]-first[1])))
            positions.append(segment+ratio)
    sampled.append(points[-1])
    positions.append(float(len(points)-1))
    return tuple(sampled), tuple(positions)


def build_curvature_safe_reference(
        waypoints, wheelbase_m=0.73, max_steering_deg=22.0,
        output_interval_m=0.05, validation_interval_m=0.005,
        steering_reserve_deg=2.0, max_waypoint_displacement_m=0.02):
    """Build and verify a C2 cubic B-spline reference without editing CSV.

    The smallest Laplacian smoothing strength in 0.01 increments that leaves
    the requested steering reserve is selected.  Failure to meet either the
    physical curvature bound or displacement contract raises an error.
    """
    raw = tuple(_xy(point) for point in waypoints)
    if len(raw) < 2:
        raise ReferenceGeometryError('reference requires at least two points')
    if wheelbase_m <= 0.0 or not 0.0 < max_steering_deg <= 22.0:
        raise ReferenceGeometryError('invalid vehicle steering geometry')
    if not 0.0 < validation_interval_m <= 0.01:
        raise ReferenceGeometryError(
            'validation_interval_m must be in (0, 0.01]')
    if output_interval_m <= 0.0:
        raise ReferenceGeometryError('output_interval_m must be positive')
    kappa_max = math.tan(math.radians(max_steering_deg))/wheelbase_m
    target_deg = max(1.0, max_steering_deg-max(0.0, steering_reserve_deg))
    target_kappa = math.tan(math.radians(target_deg))/wheelbase_m
    if len(raw) < 4:
        fine, fine_positions = _sample_polyline(raw, validation_interval_m)
        curvature = _signed_curvatures(fine)
        peak = max(range(len(curvature)), key=lambda i: abs(curvature[i]))
        over_limit = sum(abs(value) > kappa_max+1.0e-9
                         for value in curvature)
        if over_limit:
            raise ReferenceGeometryError(
                f'short reference has {over_limit} samples above steering limit')
        output, output_positions = _sample_polyline(raw, output_interval_m)
        fine_lengths = [0.0]
        spacing = []
        for first, second in zip(fine, fine[1:]):
            distance = _distance(first, second)
            spacing.append(distance)
            fine_lengths.append(fine_lengths[-1]+distance)
        report = ReferenceGeometryReport(
            0.0, len(raw), len(output), _polyline_length(raw),
            _polyline_length(raw), 0.0, 0.0, 0.0, 0,
            validation_interval_m, max(spacing, default=0.0),
            abs(curvature[peak]), abs(math.atan(wheelbase_m*curvature[peak])),
            0, fine_lengths[peak], len(raw) == 2, len(raw) == 2)
        return _with_yaws(output, output_positions), report
    selected = None
    for step in range(26):
        strength = step/100.0
        fine, fine_positions = _sample_bspline(
            raw, strength, validation_interval_m)
        curvature = _signed_curvatures(fine)
        peak = max(range(len(curvature)), key=lambda i: abs(curvature[i]))
        if abs(curvature[peak]) <= target_kappa+1.0e-9:
            selected = strength, fine, fine_positions, curvature, peak
            break
    if selected is None:
        raise ReferenceGeometryError(
            'no curvature-safe reference within allowed smoothing range')
    strength, fine, fine_positions, curvature, peak = selected
    spacing = tuple(_distance(first, second)
                    for first, second in zip(fine, fine[1:]))
    if max(spacing, default=0.0) > 0.01+1.0e-9:
        raise ReferenceGeometryError('validation sampling exceeded 0.01 m')
    over_limit = sum(abs(value) > kappa_max+1.0e-9
                     for value in curvature)
    if over_limit:
        raise ReferenceGeometryError(
            f'reference has {over_limit} samples above steering limit')
    displacement = _waypoint_displacements(raw, fine, fine_positions)
    if max(displacement, default=0.0) > max_waypoint_displacement_m+1.0e-9:
        raise ReferenceGeometryError(
            'curvature-safe reference exceeds waypoint displacement limit')
    output, output_positions = _sample_bspline(
        raw, strength, output_interval_m)
    output_curvature = _signed_curvatures(output)
    if max((abs(value) for value in output_curvature), default=0.0) > (
            kappa_max+1.0e-9):
        raise ReferenceGeometryError(
            'runtime reference discretization exceeds steering limit')
    fine_lengths = [0.0]
    for first, second in zip(fine, fine[1:]):
        fine_lengths.append(fine_lengths[-1]+_distance(first, second))
    report = ReferenceGeometryReport(
        strength, len(raw), len(output), _polyline_length(raw),
        _polyline_length(fine), _polyline_length(fine)-_polyline_length(raw),
        max(displacement, default=0.0),
        math.sqrt(sum(value*value for value in displacement)/len(displacement)),
        sum(value > 1.0e-6 for value in displacement),
        validation_interval_m, max(spacing, default=0.0),
        abs(curvature[peak]), abs(math.atan(wheelbase_m*curvature[peak])),
        over_limit, fine_lengths[peak], True, True)
    controls = _smoothed_controls(raw, strength)
    return _with_bspline_yaws(
        output, output_positions, controls), report

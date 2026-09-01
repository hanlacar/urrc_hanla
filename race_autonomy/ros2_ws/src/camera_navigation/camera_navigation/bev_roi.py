"""Mission-aware lateral bounds for the camera bird's-eye-view ROI."""


INTERSECTION_SECTIONS = frozenset((4, 6, 8, 11))
FIXED_NORMAL_SECTIONS = frozenset((1, 2, 7, 9, 10))
CURVE_SECTIONS = frozenset((3, 5))


def lateral_extent_m(section, turn_direction, normal_m=1.2, turn_m=1.5,
                     s_curve_m=1.5, intersection_m=1.0):
    """Return the symmetric lateral BEV extent for the active mission."""
    section = int(section)
    if section in INTERSECTION_SECTIONS:
        return float(intersection_m)
    if section in FIXED_NORMAL_SECTIONS:
        return float(normal_m)
    if section == 5:
        return float(s_curve_m)
    if section in CURVE_SECTIONS:
        return float(turn_m)
    if int(turn_direction) != 0:
        return float(turn_m)
    return float(normal_m)

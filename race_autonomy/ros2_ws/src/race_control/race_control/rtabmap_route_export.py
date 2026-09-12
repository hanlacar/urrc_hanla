"""Export the saved optimized RTAB-Map trajectory, without modifying its DB."""

import argparse
import math
from pathlib import Path
import sqlite3
import struct
import zlib

from .visual_slam_route import save_route, wrap_angle


def decode_matrix(blob, cv_type, scalar_size):
    if not blob or len(blob) < 12:
        raise ValueError("missing optimized matrix; save optimized poses in RTAB-Map first")
    rows, cols, actual_type = struct.unpack("<3i", blob[-12:])
    data = zlib.decompress(blob[:-12])
    if actual_type != cv_type or rows <= 0 or cols <= 0 or len(data) != rows * cols * scalar_size:
        raise ValueError("unsupported optimized matrix layout")
    return data


def read_optimized_route(database):
    with sqlite3.connect(Path(database).resolve().as_uri() + "?mode=ro", uri=True) as db:
        ids_blob, poses_blob = db.execute("SELECT opt_ids,opt_poses FROM Admin").fetchone()
        ids_data = decode_matrix(ids_blob, 4, 4)  # CV_32SC1
        poses_data = decode_matrix(poses_blob, 5, 4)  # CV_32FC1
        ids = [item[0] for item in struct.iter_unpack("<i", ids_data)]
        matrices = list(struct.iter_unpack("<12f", poses_data))
        if len(ids) != len(matrices) or len(set(ids)) != len(ids):
            raise ValueError("optimized IDs/poses do not match")
        nodes = {row[0]: row[1:] for row in db.execute("SELECT id,map_id,stamp FROM Node")}
    selected = [(nodes[i][1], i, nodes[i][0], m) for i, m in zip(ids, matrices) if i > 0 and i in nodes]
    selected.sort()
    if len(selected) < 2 or len({row[2] for row in selected}) != 1:
        raise ValueError("expected one mapping session with at least two optimized nodes")
    points = []
    for _, _, _, m in selected:
        if not all(math.isfinite(v) for v in m):
            raise ValueError("non-finite optimized pose")
        points.append((m[3], m[7], math.atan2(m[4], m[0])))
    return points


def resample_route(points, spacing=0.10, maximum_gap=2.0):
    if not math.isfinite(spacing) or spacing <= 0 or not math.isfinite(maximum_gap) or maximum_gap <= 0:
        raise ValueError("spacing and maximum gap must be finite and positive")
    result = [points[0]]
    for a, b in zip(points, points[1:]):
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        if distance > maximum_gap:
            raise ValueError(f"trajectory discontinuity: {distance:.2f} m exceeds {maximum_gap:.2f} m")
        steps = max(1, math.ceil(distance / spacing))
        for step in range(1, steps + 1):
            t = step / steps
            result.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                           wrap_angle(a[2] + t * wrap_angle(b[2] - a[2]))))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database")
    parser.add_argument("output", help="new route JSON; existing files are refused")
    parser.add_argument("--spacing", type=float, default=0.10)
    parser.add_argument("--maximum-gap", type=float, default=2.0)
    args = parser.parse_args()
    if Path(args.output).exists():
        parser.error("output already exists; choose a new route filename")
    original = read_optimized_route(args.database)
    points = resample_route(original, args.spacing, args.maximum_gap)
    save_route(args.output, points)
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    print(f"Saved {len(points)} points from {len(original)} optimized poses, length={length:.2f} m")
    print("This is the recorded trajectory, not a verified lane centre or collision-free route.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Convert mmission full-course routes into the DR follower input contract."""

import argparse
import csv
import math
from pathlib import Path

import yaml


SECTION_BY_SEGMENT = {
    "SEG01": 1, "SEG02": 2, "SEG03": 3, "SEG04": 4,
    "SEG05": 5, "SEG06": 6, "SEG07": 7, "SEG08": 8,
    "SEG09": 9, "SEG10": 9, "SEG11": 10, "SEG12": 11,
}


def convert(route_path: Path, segments_path: Path, output_path: Path):
    with route_path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    with segments_path.open(encoding="utf-8") as stream:
        segments = yaml.safe_load(stream)["segments"]
    section_for_index = {}
    segment_for_index = {}
    for segment in segments:
        section = SECTION_BY_SEGMENT[segment["id"]]
        for index in range(int(segment["start_index"]), int(segment["end_index"]) + 1):
            section_for_index[index] = section
            segment_for_index[index] = segment["id"]
    if set(range(len(rows))) != set(section_for_index):
        raise ValueError(f"segment coverage does not match route: {route_path}")
    output = []
    for index, row in enumerate(rows):
        next_row = rows[min(index + 1, len(rows) - 1)]
        prev_row = rows[max(index - 1, 0)]
        dx = float(next_row["x_m"]) - float(prev_row["x_m"])
        dy = float(next_row["y_m"]) - float(prev_row["y_m"])
        yaw = math.degrees(math.atan2(dy, dx)) if dx or dy else 0.0
        output.append({
            "index": index, "dr_x_m": row["x_m"], "dr_y_m": row["y_m"],
            "dr_yaw_deg": f"{yaw:.6f}", "direction": row["direction"],
            "mode": section_for_index[index], "drive_level": row["drive_level"],
            "event": "NONE", "source_segment_id": segment_for_index[index],
            "source_point_index": index,
        })
    # The endpoint must be an explicit latched stop.  Otherwise a controller
    # step can skip over the small goal-radius check and retain forward drive.
    output[-1]["event"] = "GOAL_STOP"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=output[0].keys())
        writer.writeheader(); writer.writerows(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("route", type=Path)
    parser.add_argument("segments", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    convert(args.route, args.segments, args.output)


if __name__ == "__main__":
    main()

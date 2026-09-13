#!/usr/bin/env python3
"""Build an RViz reference CSV from the exact segmented follower network."""

import argparse
import csv
import math
from pathlib import Path


FIELDS = ('index', 'dr_x_m', 'dr_y_m', 'dr_yaw_deg', 'direction', 'mode',
          'drive_level', 'event', 'source_segment_id', 'source_point_index')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('network', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('segments', nargs='+')
    args = parser.parse_args()

    with args.network.open(encoding='utf-8-sig', newline='') as stream:
        network = list(csv.DictReader(stream))

    points = []
    for segment in args.segments:
        part = [row for row in network if row['segment_id'] == segment]
        if not part:
            raise SystemExit(f'missing segment: {segment}')
        for row in part:
            points.append({
                'x': float(row['x_m']), 'y': float(row['y_m']),
                'direction': row['direction'], 'mode': row['mode'],
                'drive_level': row['drive_level'], 'event': row['event'],
                'segment': segment, 'point_index': row['point_index'],
            })

    for index, point in enumerate(points):
        other = points[index + 1] if index + 1 < len(points) else points[index - 1]
        dx, dy = other['x'] - point['x'], other['y'] - point['y']
        if index == len(points) - 1:
            dx, dy = -dx, -dy
        point['yaw'] = math.degrees(math.atan2(dy, dx))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for index, point in enumerate(points):
            writer.writerow({
                'index': index, 'dr_x_m': f"{point['x']:.6f}",
                'dr_y_m': f"{point['y']:.6f}",
                'dr_yaw_deg': f"{point['yaw']:.6f}",
                'direction': point['direction'], 'mode': point['mode'],
                'drive_level': point['drive_level'], 'event': point['event'],
                'source_segment_id': point['segment'],
                'source_point_index': point['point_index'],
            })


if __name__ == '__main__':
    main()

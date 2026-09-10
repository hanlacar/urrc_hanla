"""Read mission_manager routes and validate GPS waypoint events without ROS."""

import csv
import math
import re
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Waypoint:
    index: int
    section: int
    mode: str
    segment: str
    x: float
    y: float
    latitude: float
    longitude: float
    stop_line: bool
    direction: int = 1


def load_waypoints(csv_path, segments_path=""):
    with open(csv_path, newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("route is empty")
    segments = {}
    if segments_path:
        with open(segments_path, encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        for item in metadata["segments"]:
            start, end = int(item["start_index"]), int(item["end_index"])
            if not 0 <= start <= end < len(rows):
                raise ValueError("segment range is outside route")
            for index in range(start, end + 1):
                if index in segments:
                    raise ValueError("overlapping segment ranges")
                segments[index] = item
        if len(segments) != len(rows):
            raise ValueError("segments must cover the entire selected route")
    points = []
    for index, row in enumerate(rows):
        if int(row["index"]) != index:
            raise ValueError("route indices must be contiguous from zero")
        segment = segments.get(index, {})
        mode = row["mode"].strip()
        if segment and str(segment["mode"]) != mode:
            raise ValueError(f"CSV/segment mode mismatch at {index}")
        section = int(segment.get("section", mode))
        if not 1 <= section <= 11:
            raise ValueError(f"section must be 1..11 at {index}")
        coords = [float(row[key]) for key in ("x_m", "y_m", "latitude", "longitude")]
        if not all(math.isfinite(value) for value in coords):
            raise ValueError(f"non-finite coordinate at {index}")
        event = row.get("event", "NONE").strip().upper()
        if event not in {"NONE", "STOP_LINE"}:
            raise ValueError(f"unsupported event at {index}: {event}")
        direction = int(row.get("direction", "1"))
        if direction not in (-1, 1):
            raise ValueError(f"invalid route direction at {index}")
        points.append(Waypoint(index, section, mode, str(segment.get("id", "")),
                               *coords, event == "STOP_LINE", direction))
    return points


class RouteEvents:
    """Use tracker index for sections, explicit arrival identity for stop lines.

    The upstream tracker index is a path segment, NOT an arrival confirmation.
    A legacy follower only gives the stop index in its first arrival reason;
    newer publishers can include stop_line_index on every stopped status.
    """

    VALID_STATES = {"TRACKING", "ALIGNING", "APPROACH_STOP_LINE",
                    "STOPPED_AT_STOP_LINE", "APPROACH_CUSP", "STOPPED_AT_CUSP"}

    def __init__(self, points):
        self.points = points
        self.arrival_index = None

    def update(self, status):
        if not isinstance(status, dict):
            raise ValueError("status must be an object")
        if (status.get("gps_healthy") is not True or
                status.get("imu_healthy") is not True or
                status.get("state") not in self.VALID_STATES):
            self.arrival_index = None
            raise ValueError("navigation is not healthy/active")
        index = status.get("route_index")
        if type(index) is not int or not 0 <= index < len(self.points):
            raise ValueError("invalid route_index")
        point = self.points[index]
        if str(status.get("mode")) != point.mode:
            raise ValueError("status mode does not match the selected route")
        if point.segment and status.get("segment_id") != point.segment:
            raise ValueError("status segment does not match the selected route")
        if status["state"] != "STOPPED_AT_STOP_LINE":
            self.arrival_index = None
        else:
            arrival = status.get("stop_line_index")
            match = re.fullmatch(r"stop line reached index=(\d+)", str(status.get("reason", "")))
            if arrival is None and match:
                arrival = int(match[1])
            if arrival is not None:
                if (type(arrival) is not int or not 0 <= arrival < len(self.points)
                        or not self.points[arrival].stop_line
                        or self.points[arrival].section != point.section):
                    raise ValueError("invalid stop-line arrival identity")
                self.arrival_index = arrival
            if (self.arrival_index is not None and
                    self.points[self.arrival_index].section != point.section):
                self.arrival_index = None
            if self.arrival_index is None:
                raise ValueError("stopped status is missing stop-line identity")
        return point, self.arrival_index

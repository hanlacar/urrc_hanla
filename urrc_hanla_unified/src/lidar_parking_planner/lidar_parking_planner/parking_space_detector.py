"""Pure OccupancyGrid parking-space detection utilities.

The classes in this module deliberately do not import ROS.  They can therefore
be unit-tested with ordinary NumPy arrays and reused by tools that read saved
maps without starting an rclpy context.
"""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Tuple

import numpy as np


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def axial_angle_difference(first: float, second: float) -> float:
    """Smallest difference between rectangle axes (yaw and yaw + pi are equal)."""
    difference = abs(normalize_angle(first - second))
    return min(difference, math.pi - difference)


@dataclass(frozen=True)
class MapMetadata:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def to_dict(self):
        return {
            'resolution': float(self.resolution),
            'width': int(self.width),
            'height': int(self.height),
            'origin': {
                'x': float(self.origin_x),
                'y': float(self.origin_y),
                'yaw': float(self.origin_yaw),
            },
        }


class GridMap:
    """An occupancy grid plus coordinate conversion helpers."""

    def __init__(self, data: np.ndarray, metadata: MapMetadata):
        values = np.asarray(data, dtype=np.int16)
        expected = (metadata.height, metadata.width)
        if values.shape != expected:
            raise ValueError(
                f'grid shape {values.shape} does not match metadata {expected}')
        if metadata.resolution <= 0.0:
            raise ValueError('map resolution must be positive')
        self.data = values
        self.metadata = metadata
        self._origin_cos = math.cos(metadata.origin_yaw)
        self._origin_sin = math.sin(metadata.origin_yaw)

    @property
    def resolution(self) -> float:
        return self.metadata.resolution

    @property
    def width(self) -> int:
        return self.metadata.width

    @property
    def height(self) -> int:
        return self.metadata.height

    def grid_to_world(
            self, grid_x, grid_y, cell_center: bool = True) -> Tuple:
        """Convert integer/scalar or array grid coordinates to map coordinates."""
        offset = 0.5 if cell_center else 0.0
        local_x = (np.asarray(grid_x, dtype=float) + offset) * self.resolution
        local_y = (np.asarray(grid_y, dtype=float) + offset) * self.resolution
        world_x = (
            self.metadata.origin_x
            + self._origin_cos * local_x
            - self._origin_sin * local_y)
        world_y = (
            self.metadata.origin_y
            + self._origin_sin * local_x
            + self._origin_cos * local_y)
        if np.ndim(world_x) == 0 and np.ndim(world_y) == 0:
            return float(world_x), float(world_y)
        return world_x, world_y

    def world_to_grid_float(self, world_x, world_y) -> Tuple:
        """Convert map coordinates to continuous grid coordinates."""
        dx = np.asarray(world_x, dtype=float) - self.metadata.origin_x
        dy = np.asarray(world_y, dtype=float) - self.metadata.origin_y
        local_x = self._origin_cos * dx + self._origin_sin * dy
        local_y = -self._origin_sin * dx + self._origin_cos * dy
        grid_x = local_x / self.resolution
        grid_y = local_y / self.resolution
        if np.ndim(grid_x) == 0 and np.ndim(grid_y) == 0:
            return float(grid_x), float(grid_y)
        return grid_x, grid_y

    def world_to_grid(
            self, world_x: float, world_y: float) -> Optional[Tuple[int, int]]:
        """Return the containing (x, y) cell, or None outside the map."""
        grid_x, grid_y = self.world_to_grid_float(world_x, world_y)
        index_x = math.floor(grid_x)
        index_y = math.floor(grid_y)
        if 0 <= index_x < self.width and 0 <= index_y < self.height:
            return index_x, index_y
        return None

    def contains_world_point(self, world_x: float, world_y: float) -> bool:
        grid_x, grid_y = self.world_to_grid_float(world_x, world_y)
        return 0.0 <= grid_x < self.width and 0.0 <= grid_y < self.height

    def known_ratio(self) -> float:
        if self.data.size == 0:
            return 0.0
        return float(np.count_nonzero(self.data >= 0) / self.data.size)


@dataclass(frozen=True)
class ParkingCandidate:
    parking_type: str
    center_x: float
    center_y: float
    yaw: float
    width: float
    length: float
    confidence: float
    free_ratio: float = 0.0
    unknown_ratio: float = 1.0
    occupied_ratio: float = 0.0
    boundary_support: float = 0.0


@dataclass(frozen=True)
class RegionMetrics:
    inside_map: bool
    cell_count: int
    free_ratio: float
    unknown_ratio: float
    occupied_ratio: float
    boundary_support: float = 0.0

    @property
    def is_empty(self) -> bool:
        return (
            self.inside_map
            and self.cell_count > 0
            and self.occupied_ratio == 0.0)


@dataclass
class DetectorConfig:
    occupied_threshold: int = 65
    free_threshold: int = 25
    min_free_ratio: float = 0.85
    max_unknown_ratio: float = 0.15
    safety_margin_m: float = 0.30
    vehicle_length_m: float = 1.0
    vehicle_width_m: float = 0.6
    t_slot_width_m: float = 1.2
    t_slot_depth_m: float = 1.8
    parallel_slot_length_m: float = 2.0
    parallel_slot_width_m: float = 1.0
    search_forward_m: float = 4.0
    search_backward_m: float = 2.0
    search_right_min_m: float = 0.5
    search_right_max_m: float = 3.0
    sample_step_m: float = 0.40
    nms_distance_m: float = 0.75
    boundary_band_m: float = 0.20
    max_candidates_per_type: int = 12


class ParkingSpaceDetector:
    """Search a limited, robot-relative ROI for clear parking rectangles."""

    VALID_TYPES = ('T', 'PARALLEL')

    def __init__(self, config: Optional[DetectorConfig] = None):
        self.config = config or DetectorConfig()
        if self.config.free_threshold >= self.config.occupied_threshold:
            raise ValueError('free_threshold must be below occupied_threshold')
        if self.config.sample_step_m <= 0.0:
            raise ValueError('sample_step_m must be positive')

    @staticmethod
    def _rectangle_corners(
            center_x: float, center_y: float, yaw: float,
            length: float, width: float) -> np.ndarray:
        half_length = length / 2.0
        half_width = width / 2.0
        local = np.asarray([
            [-half_length, -half_width],
            [-half_length, half_width],
            [half_length, -half_width],
            [half_length, half_width],
        ])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.asarray([[cosine, -sine], [sine, cosine]])
        return local @ rotation.T + np.asarray([center_x, center_y])

    def _rectangle_values(
            self, grid: GridMap, center_x: float, center_y: float,
            yaw: float, length: float, width: float) -> Optional[np.ndarray]:
        corners = self._rectangle_corners(
            center_x, center_y, yaw, length, width)
        if not all(grid.contains_world_point(x, y) for x, y in corners):
            return None

        grid_x, grid_y = grid.world_to_grid_float(
            corners[:, 0], corners[:, 1])
        min_x = max(0, int(math.floor(float(np.min(grid_x)))))
        max_x = min(grid.width - 1, int(math.floor(float(np.max(grid_x)))))
        min_y = max(0, int(math.floor(float(np.min(grid_y)))))
        max_y = min(grid.height - 1, int(math.floor(float(np.max(grid_y)))))
        if min_x > max_x or min_y > max_y:
            return np.asarray([], dtype=np.int16)

        columns, rows = np.meshgrid(
            np.arange(min_x, max_x + 1),
            np.arange(min_y, max_y + 1))
        world_x, world_y = grid.grid_to_world(columns, rows)
        dx = world_x - center_x
        dy = world_y - center_y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        epsilon = grid.resolution * 1e-6
        mask = (
            (np.abs(local_x) <= length / 2.0 + epsilon)
            & (np.abs(local_y) <= width / 2.0 + epsilon))
        return grid.data[rows[mask], columns[mask]]

    def region_metrics(
            self, grid: GridMap, center_x: float, center_y: float,
            yaw: float, length: float, width: float,
            include_boundary: bool = True) -> RegionMetrics:
        values = self._rectangle_values(
            grid, center_x, center_y, yaw, length, width)
        if values is None:
            return RegionMetrics(False, 0, 0.0, 1.0, 0.0)
        count = int(values.size)
        if count == 0:
            return RegionMetrics(True, 0, 0.0, 1.0, 0.0)

        unknown_ratio = float(np.count_nonzero(values < 0) / count)
        free_ratio = float(np.count_nonzero(
            (values >= 0) & (values <= self.config.free_threshold)) / count)
        occupied_ratio = float(np.count_nonzero(
            values >= self.config.occupied_threshold) / count)

        boundary_support = 0.0
        band = self.config.boundary_band_m
        if include_boundary and band > 0.0:
            expanded = self._rectangle_values(
                grid, center_x, center_y, yaw,
                length + 2.0 * band, width + 2.0 * band)
            if expanded is not None and expanded.size > count:
                expanded_occupied = np.count_nonzero(
                    expanded >= self.config.occupied_threshold)
                inside_occupied = np.count_nonzero(
                    values >= self.config.occupied_threshold)
                boundary_count = expanded.size - count
                boundary_support = float(
                    max(0, expanded_occupied - inside_occupied) / boundary_count)

        return RegionMetrics(
            True, count, free_ratio, unknown_ratio,
            occupied_ratio, boundary_support)

    def required_clearance(self, parking_type: str) -> Tuple[float, float]:
        """Return (length, width) including vehicle safety requirements."""
        vehicle_length = (
            self.config.vehicle_length_m + 2.0 * self.config.safety_margin_m)
        vehicle_width = (
            self.config.vehicle_width_m + 2.0 * self.config.safety_margin_m)
        if parking_type == 'T':
            return (
                max(self.config.t_slot_depth_m, vehicle_length),
                max(self.config.t_slot_width_m, vehicle_width),
            )
        if parking_type == 'PARALLEL':
            return (
                max(self.config.parallel_slot_length_m, vehicle_length),
                max(self.config.parallel_slot_width_m, vehicle_width),
            )
        raise ValueError(f'unsupported parking_type: {parking_type}')

    def evaluate_candidate(
            self, grid: GridMap, candidate: ParkingCandidate) -> RegionMetrics:
        clearance_length, clearance_width = self.required_clearance(
            candidate.parking_type)
        return self.region_metrics(
            grid, candidate.center_x, candidate.center_y, candidate.yaw,
            clearance_length, clearance_width)

    def _candidate_if_clear(
            self, grid: GridMap, parking_type: str, center_x: float,
            center_y: float, yaw: float, length: float,
            width: float) -> Optional[ParkingCandidate]:
        clearance_length, clearance_width = self.required_clearance(parking_type)
        metrics = self.region_metrics(
            grid, center_x, center_y, yaw,
            clearance_length, clearance_width)
        if (
                not metrics.inside_map
                or metrics.cell_count == 0
                or metrics.occupied_ratio > 0.0
                or metrics.free_ratio < self.config.min_free_ratio
                or metrics.unknown_ratio > self.config.max_unknown_ratio):
            return None
        confidence = min(
            1.0,
            0.80 * metrics.free_ratio
            + 0.20 * metrics.boundary_support)
        return ParkingCandidate(
            parking_type=parking_type,
            center_x=float(center_x),
            center_y=float(center_y),
            yaw=normalize_angle(yaw),
            width=float(width),
            length=float(length),
            confidence=confidence,
            free_ratio=metrics.free_ratio,
            unknown_ratio=metrics.unknown_ratio,
            occupied_ratio=metrics.occupied_ratio,
            boundary_support=metrics.boundary_support,
        )

    @staticmethod
    def _sample_range(start: float, stop: float, step: float) -> Iterable[float]:
        count = max(0, int(math.floor((stop - start) / step)))
        for index in range(count + 1):
            yield start + index * step
        if count == 0 or start + count * step < stop - 1e-9:
            yield stop

    def detect(
            self, grid: GridMap, robot_x: float, robot_y: float,
            robot_yaw: float) -> List[ParkingCandidate]:
        """Detect T and parallel candidates in the right-side local ROI."""
        candidates: List[ParkingCandidate] = []
        cosine = math.cos(robot_yaw)
        sine = math.sin(robot_yaw)
        step = self.config.sample_step_m
        for forward in self._sample_range(
                -self.config.search_backward_m,
                self.config.search_forward_m, step):
            for right in self._sample_range(
                    self.config.search_right_min_m,
                    self.config.search_right_max_m, step):
                # ROS base convention is +x forward, +y left, hence right=-y.
                center_x = robot_x + cosine * forward + sine * right
                center_y = robot_y + sine * forward - cosine * right
                specifications = (
                    (
                        'T',
                        normalize_angle(robot_yaw - math.pi / 2.0),
                        self.config.t_slot_depth_m,
                        self.config.t_slot_width_m,
                    ),
                    (
                        'PARALLEL',
                        normalize_angle(robot_yaw),
                        self.config.parallel_slot_length_m,
                        self.config.parallel_slot_width_m,
                    ),
                )
                for parking_type, yaw, length, width in specifications:
                    candidate = self._candidate_if_clear(
                        grid, parking_type, center_x, center_y,
                        yaw, length, width)
                    if candidate is not None:
                        candidates.append(candidate)
        return self.non_maximum_suppression(candidates)

    def non_maximum_suppression(
            self, candidates: Iterable[ParkingCandidate]
    ) -> List[ParkingCandidate]:
        """Keep spatially distinct high-scoring candidates of each type."""
        ordered = sorted(
            candidates,
            key=lambda item: (
                -item.confidence,
                item.parking_type,
                item.center_x,
                item.center_y,
            ))
        kept: List[ParkingCandidate] = []
        counts = {parking_type: 0 for parking_type in self.VALID_TYPES}
        for candidate in ordered:
            if counts.get(candidate.parking_type, 0) >= int(
                    self.config.max_candidates_per_type):
                continue
            duplicate = any(
                existing.parking_type == candidate.parking_type
                and math.hypot(
                    existing.center_x - candidate.center_x,
                    existing.center_y - candidate.center_y)
                < self.config.nms_distance_m
                and axial_angle_difference(existing.yaw, candidate.yaw)
                < math.radians(10.0)
                for existing in kept)
            if duplicate:
                continue
            kept.append(candidate)
            counts[candidate.parking_type] = (
                counts.get(candidate.parking_type, 0) + 1)
        return kept

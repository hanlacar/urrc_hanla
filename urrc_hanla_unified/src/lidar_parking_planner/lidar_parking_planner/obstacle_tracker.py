"""ROS-independent local LiDAR obstacle tracking and motion classification.

Sign convention used throughout this module:

* positive ``ego_speed_mps`` means the vehicle moves forward;
* negative range rate means an obstacle range is decreasing;
* positive object radial speed means motion away from the LiDAR.

For straight-line ego motion the world-referenced radial object speed is
``range_rate + ego_speed_mps * cos(ray_angle)``.  The encoder sign and the
LaserScan zero direction must still be verified on the real vehicle before
``encoder_source_verified`` is enabled in the ROS configuration.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple


UNKNOWN = 'UNKNOWN'
STATIC = 'STATIC'
DYNAMIC_APPROACHING = 'DYNAMIC_APPROACHING'
DYNAMIC_RECEDING = 'DYNAMIC_RECEDING'


def _angle_difference(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


@dataclass
class TrackerConfig:
    front_sector_deg: float = 30.0
    min_valid_range_m: float = 0.05
    cluster_distance_threshold_m: float = 0.15
    min_cluster_points: int = 3
    association_distance_m: float = 0.35
    association_angle_deg: float = 10.0
    track_timeout_sec: float = 0.5
    min_dt_sec: float = 0.02
    max_dt_sec: float = 0.5
    velocity_filter_alpha: float = 0.35
    max_reasonable_object_speed_mps: float = 5.0
    static_speed_threshold_mps: float = 0.15
    dynamic_speed_threshold_mps: float = 0.30
    classification_confirm_frames: int = 3


@dataclass(frozen=True)
class ScanPoint:
    x: float
    y: float
    distance: float
    angle: float


@dataclass
class Cluster:
    center_x: float
    center_y: float
    center_range: float
    center_angle: float
    min_range: float
    width: float
    point_count: int
    timestamp: float
    front_min_range: float

    @property
    def intersects_front_sector(self) -> bool:
        return math.isfinite(self.front_min_range)


@dataclass
class Track:
    track_id: str
    center_x: float
    center_y: float
    center_range: float
    center_angle: float
    min_range: float
    width: float
    point_count: int
    timestamp: float
    front_min_range: float
    first_seen: float
    last_seen: float
    hits: int = 1
    missed_frames: int = 0
    raw_range_rate: Optional[float] = None
    ego_radial_speed: Optional[float] = None
    object_radial_speed: Optional[float] = None
    filtered_object_speed: Optional[float] = None
    classification: str = UNKNOWN
    confidence: float = 0.0
    velocity_samples: int = 0
    candidate_classification: str = UNKNOWN
    candidate_count: int = 0

    @property
    def intersects_front_sector(self) -> bool:
        return math.isfinite(self.front_min_range)


@dataclass
class SafetyConfig:
    static_stop_distance_m: float = 0.50
    dynamic_stop_distance_m: float = 0.50
    unknown_stop_distance_m: float = 0.50
    static_resume_distance_m: float = 0.60
    dynamic_resume_distance_m: float = 0.60
    unknown_resume_distance_m: float = 0.60
    stop_confirm_frames: int = 2
    clear_confirm_frames: int = 3
    scan_timeout_sec: float = 0.5
    stop_on_scan_timeout: bool = True
    enable_ttc_stop: bool = False
    ttc_stop_sec: float = 1.5


class ObstacleTracker:
    """Cluster consecutive scan points and maintain frame-to-frame tracks."""

    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config or TrackerConfig()
        self._tracks: Dict[str, Track] = {}
        self._next_track_number = 1

    @property
    def tracks(self) -> List[Track]:
        return list(self._tracks.values())

    def scan_to_clusters(self, scan, timestamp: float) -> List[Cluster]:
        groups: List[List[ScanPoint]] = []
        current: List[ScanPoint] = []
        previous: Optional[ScanPoint] = None
        angle = float(scan.angle_min)
        sector = math.radians(max(0.0, self.config.front_sector_deg))

        for value in scan.ranges:
            distance = float(value)
            valid = (
                math.isfinite(distance)
                and distance != 0.0
                and distance >= float(scan.range_min)
                and distance <= float(scan.range_max)
                and distance >= self.config.min_valid_range_m
            )
            if not valid:
                if current:
                    groups.append(current)
                current = []
                previous = None
                angle += float(scan.angle_increment)
                continue

            point = ScanPoint(
                x=distance * math.cos(angle),
                y=distance * math.sin(angle),
                distance=distance,
                angle=angle,
            )
            if previous is not None:
                separation = math.hypot(point.x - previous.x, point.y - previous.y)
                if separation > self.config.cluster_distance_threshold_m:
                    if current:
                        groups.append(current)
                    current = []
            current.append(point)
            previous = point
            angle += float(scan.angle_increment)
        if current:
            groups.append(current)

        clusters = []
        for points in groups:
            if len(points) < max(1, self.config.min_cluster_points):
                continue
            center_x = sum(point.x for point in points) / len(points)
            center_y = sum(point.y for point in points) / len(points)
            front_ranges = [
                point.distance for point in points
                if _angle_difference(point.angle, 0.0) <= sector
            ]
            clusters.append(Cluster(
                center_x=center_x,
                center_y=center_y,
                center_range=math.hypot(center_x, center_y),
                center_angle=math.atan2(center_y, center_x),
                min_range=min(point.distance for point in points),
                width=math.hypot(
                    points[-1].x - points[0].x,
                    points[-1].y - points[0].y),
                point_count=len(points),
                timestamp=timestamp,
                front_min_range=min(front_ranges, default=math.inf),
            ))
        return clusters

    def update(
            self, scan, timestamp: float, ego_speed_mps: float = 0.0,
            encoder_valid: bool = False,
            motion_classification_allowed: bool = True) -> List[Track]:
        clusters = self.scan_to_clusters(scan, timestamp)
        self._expire_tracks(timestamp)
        associations = self._associate(clusters, timestamp)
        observed_ids = set()

        for cluster_index, track_id in associations.items():
            track = self._tracks[track_id]
            self._update_track(
                track, clusters[cluster_index], ego_speed_mps, encoder_valid,
                motion_classification_allowed)
            observed_ids.add(track_id)

        for index, cluster in enumerate(clusters):
            if index in associations:
                continue
            track = self._new_track(cluster)
            self._tracks[track.track_id] = track
            observed_ids.add(track.track_id)

        for track_id, track in self._tracks.items():
            if track_id not in observed_ids:
                track.missed_frames += 1

        # Safety and debug output describe current observations.  Missed tracks
        # remain internally available for association until track_timeout_sec.
        return sorted(
            (self._tracks[track_id] for track_id in observed_ids),
            key=lambda track: track.track_id)

    def _expire_tracks(self, timestamp: float) -> None:
        expired = [
            track_id for track_id, track in self._tracks.items()
            if timestamp - track.last_seen > self.config.track_timeout_sec
        ]
        for track_id in expired:
            del self._tracks[track_id]

    def _associate(
            self, clusters: Sequence[Cluster], timestamp: float) -> Dict[int, str]:
        candidates: List[Tuple[float, int, str]] = []
        angle_limit = math.radians(max(0.0, self.config.association_angle_deg))
        for cluster_index, cluster in enumerate(clusters):
            for track_id, track in self._tracks.items():
                dt = max(0.0, timestamp - track.last_seen)
                predicted_range = track.center_range
                if track.raw_range_rate is not None:
                    predicted_range = max(
                        0.0, predicted_range + track.raw_range_rate * dt)
                predicted_x = predicted_range * math.cos(track.center_angle)
                predicted_y = predicted_range * math.sin(track.center_angle)
                position_error = math.hypot(
                    cluster.center_x - predicted_x,
                    cluster.center_y - predicted_y)
                angle_error = _angle_difference(
                    cluster.center_angle, track.center_angle)
                if position_error > self.config.association_distance_m:
                    continue
                if angle_error > angle_limit:
                    continue
                width_error = abs(cluster.width - track.width)
                score = (
                    position_error
                    + 0.20 * angle_error / max(angle_limit, 1e-6)
                    + 0.10 * width_error
                )
                candidates.append((score, cluster_index, track_id))

        associations: Dict[int, str] = {}
        used_tracks = set()
        for _score, cluster_index, track_id in sorted(candidates):
            if cluster_index in associations or track_id in used_tracks:
                continue
            associations[cluster_index] = track_id
            used_tracks.add(track_id)
        return associations

    def _new_track(self, cluster: Cluster) -> Track:
        track_id = f'OBS_{self._next_track_number:04d}'
        self._next_track_number += 1
        return Track(
            track_id=track_id,
            center_x=cluster.center_x,
            center_y=cluster.center_y,
            center_range=cluster.center_range,
            center_angle=cluster.center_angle,
            min_range=cluster.min_range,
            width=cluster.width,
            point_count=cluster.point_count,
            timestamp=cluster.timestamp,
            front_min_range=cluster.front_min_range,
            first_seen=cluster.timestamp,
            last_seen=cluster.timestamp,
        )

    def _update_track(
            self, track: Track, cluster: Cluster, ego_speed_mps: float,
            encoder_valid: bool, motion_classification_allowed: bool) -> None:
        dt = cluster.timestamp - track.last_seen
        track.hits += 1
        track.missed_frames = 0

        velocity_valid = self.config.min_dt_sec <= dt <= self.config.max_dt_sec
        if velocity_valid:
            raw_rate = (cluster.center_range - track.center_range) / dt
            track.raw_range_rate = raw_rate
            if (encoder_valid and motion_classification_allowed
                    and math.isfinite(ego_speed_mps)):
                # Straight-line ego-motion compensation.  The current cluster
                # angle is used because its radial component corresponds to the
                # current range measurement.
                ego_radial = ego_speed_mps * math.cos(cluster.center_angle)
                object_speed = raw_rate + ego_radial
                track.ego_radial_speed = ego_radial
                track.object_radial_speed = object_speed
                if abs(object_speed) <= self.config.max_reasonable_object_speed_mps:
                    alpha = min(1.0, max(0.0, self.config.velocity_filter_alpha))
                    if track.filtered_object_speed is None:
                        track.filtered_object_speed = object_speed
                    else:
                        track.filtered_object_speed = (
                            alpha * object_speed
                            + (1.0 - alpha) * track.filtered_object_speed)
                    track.velocity_samples += 1
                    self._update_classification(track)
                # A rejected glitch supplies no classification evidence.
            else:
                track.ego_radial_speed = None
                track.object_radial_speed = None
                self._force_unknown(track)
        elif not encoder_valid or not motion_classification_allowed:
            self._force_unknown(track)

        track.center_x = cluster.center_x
        track.center_y = cluster.center_y
        track.center_range = cluster.center_range
        track.center_angle = cluster.center_angle
        track.min_range = cluster.min_range
        track.width = cluster.width
        track.point_count = cluster.point_count
        track.timestamp = cluster.timestamp
        track.front_min_range = cluster.front_min_range
        track.last_seen = cluster.timestamp

    def _update_classification(self, track: Track) -> None:
        speed = track.filtered_object_speed
        if speed is None:
            candidate = UNKNOWN
        elif abs(speed) <= self.config.static_speed_threshold_mps:
            candidate = STATIC
        elif speed <= -self.config.dynamic_speed_threshold_mps:
            candidate = DYNAMIC_APPROACHING
        elif speed >= self.config.dynamic_speed_threshold_mps:
            candidate = DYNAMIC_RECEDING
        else:
            candidate = UNKNOWN

        if candidate == track.candidate_classification:
            track.candidate_count += 1
        else:
            track.candidate_classification = candidate
            track.candidate_count = 1
        required = max(1, self.config.classification_confirm_frames)
        if track.candidate_count >= required:
            track.classification = candidate
        track.confidence = min(1.0, track.candidate_count / required)

    @staticmethod
    def _force_unknown(track: Track) -> None:
        # Encoder loss invalidates world-motion classification immediately;
        # distance-based UNKNOWN stopping remains active.
        track.classification = UNKNOWN
        track.candidate_classification = UNKNOWN
        track.candidate_count = 0
        track.confidence = 0.0


class ObstacleSafetyGate:
    """Distance/TTC stop latch for classified front-sector obstacle tracks."""

    WAIT_SCAN = 'WAIT_SCAN'
    CLEAR = 'CLEAR'
    OBSTACLE_STOP = 'OBSTACLE_STOP'
    LIDAR_TIMEOUT = 'LIDAR_TIMEOUT'

    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self.state = self.WAIT_SCAN
        self.scan_received = False
        self.nearest_distance = math.inf
        self.nearest_type = 'NONE'
        self._stop_count = 0
        self._clear_count = 0

    def update(self, tracks: Sequence[Track]) -> Tuple[str, str]:
        previous = self.state
        self.scan_received = True
        front_tracks = [track for track in tracks if track.intersects_front_sector]
        nearest = min(
            front_tracks, key=lambda track: track.front_min_range,
            default=None)
        self.nearest_distance = (
            nearest.front_min_range if nearest is not None else math.inf)
        self.nearest_type = (
            nearest.classification if nearest is not None else 'NONE')

        hazard = any(self._requires_stop(track) for track in front_tracks)
        clear = all(self._is_beyond_resume(track) for track in front_tracks)
        if self.state == self.LIDAR_TIMEOUT and hazard:
            # A restored scan that still sees danger must never briefly reopen
            # the drive gate while stop confirmation is rebuilt.
            self.state = self.OBSTACLE_STOP
            self._stop_count = 0
            self._clear_count = 0
            return previous, self.state

        if self.state in (self.OBSTACLE_STOP, self.LIDAR_TIMEOUT):
            self._stop_count = 0
            if clear:
                self._clear_count += 1
                if self._clear_count >= max(1, self.config.clear_confirm_frames):
                    self.state = self.CLEAR
                    self._clear_count = 0
            else:
                self._clear_count = 0
            return previous, self.state

        self._clear_count = 0
        if hazard:
            self._stop_count += 1
            if self._stop_count >= max(1, self.config.stop_confirm_frames):
                self.state = self.OBSTACLE_STOP
                self._stop_count = 0
        else:
            self._stop_count = 0
            self.state = self.CLEAR
        return previous, self.state

    def update_timeout(self, scan_age_sec: float) -> Tuple[str, str]:
        previous = self.state
        if self.scan_received and scan_age_sec > self.config.scan_timeout_sec:
            self.state = self.LIDAR_TIMEOUT
            self._stop_count = 0
            self._clear_count = 0
        return previous, self.state

    def filter_drive(self, drive: float) -> float:
        if self.should_stop:
            return 0.0
        return drive

    @property
    def should_stop(self) -> bool:
        if self.state == self.OBSTACLE_STOP:
            return True
        if self.config.stop_on_scan_timeout:
            return self.state in (self.WAIT_SCAN, self.LIDAR_TIMEOUT)
        return False

    @property
    def obstacle_detected(self) -> bool:
        return self.state == self.OBSTACLE_STOP

    def _requires_stop(self, track: Track) -> bool:
        if track.front_min_range <= self._distance_for(track, stop=True):
            return True
        if not self.config.enable_ttc_stop:
            return False
        if track.classification != DYNAMIC_APPROACHING:
            return False
        speed = track.filtered_object_speed
        if speed is None or speed >= 0.0:
            return False
        return track.front_min_range / -speed <= self.config.ttc_stop_sec

    def _is_beyond_resume(self, track: Track) -> bool:
        return track.front_min_range >= self._distance_for(track, stop=False)

    def _distance_for(self, track: Track, stop: bool) -> float:
        classification = track.classification
        if classification == STATIC:
            return (
                self.config.static_stop_distance_m if stop
                else self.config.static_resume_distance_m)
        if classification in (DYNAMIC_APPROACHING, DYNAMIC_RECEDING):
            return (
                self.config.dynamic_stop_distance_m if stop
                else self.config.dynamic_resume_distance_m)
        return (
            self.config.unknown_stop_distance_m if stop
            else self.config.unknown_resume_distance_m)


def track_to_dict(track: Track) -> dict:
    """Return a JSON-safe diagnostic representation of one current track."""
    def finite_or_none(value):
        return value if value is not None and math.isfinite(value) else None

    speed = track.filtered_object_speed
    ttc = None
    if speed is not None and speed < 0.0:
        ttc = track.front_min_range / -speed if track.intersects_front_sector else None
    return {
        'id': track.track_id,
        'center_x_m': round(track.center_x, 4),
        'center_y_m': round(track.center_y, 4),
        'distance_m': round(track.front_min_range, 4)
        if track.intersects_front_sector else round(track.min_range, 4),
        'center_distance_m': round(track.center_range, 4),
        'angle_deg': round(math.degrees(track.center_angle), 3),
        'width_m': round(track.width, 4),
        'point_count': track.point_count,
        'raw_range_rate_mps': finite_or_none(track.raw_range_rate),
        'ego_radial_speed_mps': finite_or_none(track.ego_radial_speed),
        'object_speed_mps': finite_or_none(track.filtered_object_speed),
        'classification': track.classification,
        'confidence': round(track.confidence, 3),
        'ttc_sec': finite_or_none(ttc),
    }

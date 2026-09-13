"""Pure runtime tracking and JSON persistence for map-frame parking slots."""

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .parking_space_detector import (
    MapMetadata,
    ParkingCandidate,
    axial_angle_difference,
    normalize_angle,
)


VALID_PARKING_TYPES = ('T', 'PARALLEL', 'UNKNOWN')
VALID_STATUSES = ('CANDIDATE', 'CONFIRMED', 'TEMPORARILY_LOST', 'OCCUPIED')


@dataclass
class ParkingSlot:
    slot_id: str
    parking_type: str
    center_x: float
    center_y: float
    yaw: float
    width: float
    length: float
    confidence: float
    observation_count: int
    first_seen_time: float
    last_seen_time: float
    status: str

    def to_dict(self) -> Dict:
        result = asdict(self)
        result['center_x'] = float(self.center_x)
        result['center_y'] = float(self.center_y)
        result['yaw'] = float(self.yaw)
        result['width'] = float(self.width)
        result['length'] = float(self.length)
        result['confidence'] = float(self.confidence)
        result['observation_count'] = int(self.observation_count)
        result['first_seen_time'] = float(self.first_seen_time)
        result['last_seen_time'] = float(self.last_seen_time)
        return result

    @classmethod
    def from_dict(cls, value: Dict):
        parking_type = str(value.get('parking_type', 'UNKNOWN')).upper()
        status = str(value.get('status', 'TEMPORARILY_LOST')).upper()
        if parking_type not in VALID_PARKING_TYPES:
            parking_type = 'UNKNOWN'
        if status not in VALID_STATUSES:
            status = 'TEMPORARILY_LOST'
        return cls(
            slot_id=str(value['slot_id']),
            parking_type=parking_type,
            center_x=float(value['center_x']),
            center_y=float(value['center_y']),
            yaw=normalize_angle(float(value['yaw'])),
            width=float(value['width']),
            length=float(value['length']),
            confidence=max(0.0, min(1.0, float(value['confidence']))),
            observation_count=max(1, int(value['observation_count'])),
            first_seen_time=float(value['first_seen_time']),
            last_seen_time=float(value['last_seen_time']),
            status=status,
        )


@dataclass
class TrackerConfig:
    association_distance_m: float = 0.5
    association_yaw_deg: float = 15.0
    association_size_tolerance_m: float = 0.5
    confirm_observations: int = 5
    lost_timeout_sec: float = 10.0
    delete_timeout_sec: float = 60.0


class SlotTracker:
    """Associate observations with stable IDs and retain temporarily lost slots."""

    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config or TrackerConfig()
        if self.config.delete_timeout_sec < self.config.lost_timeout_sec:
            raise ValueError('delete_timeout_sec must be >= lost_timeout_sec')
        self._slots: Dict[str, ParkingSlot] = {}
        self._next_ids = {'T': 1, 'PARALLEL': 1, 'UNKNOWN': 1}

    @property
    def slots(self) -> List[ParkingSlot]:
        return [self._slots[key] for key in sorted(self._slots)]

    def get(self, slot_id: str) -> Optional[ParkingSlot]:
        return self._slots.get(slot_id)

    @staticmethod
    def _prefix(parking_type: str) -> str:
        return {'T': 'T', 'PARALLEL': 'P', 'UNKNOWN': 'U'}[parking_type]

    def _allocate_id(self, parking_type: str) -> str:
        normalized = (
            parking_type if parking_type in VALID_PARKING_TYPES else 'UNKNOWN')
        number = self._next_ids[normalized]
        self._next_ids[normalized] += 1
        return f'{self._prefix(normalized)}_{number:04d}'

    def _association_cost(
            self, slot: ParkingSlot, candidate: ParkingCandidate
    ) -> Optional[float]:
        if (
                slot.parking_type != candidate.parking_type
                and slot.parking_type != 'UNKNOWN'
                and candidate.parking_type != 'UNKNOWN'):
            return None
        distance = math.hypot(
            slot.center_x - candidate.center_x,
            slot.center_y - candidate.center_y)
        yaw_difference = axial_angle_difference(slot.yaw, candidate.yaw)
        size_difference = max(
            abs(slot.width - candidate.width),
            abs(slot.length - candidate.length))
        if (
                distance > self.config.association_distance_m
                or yaw_difference > math.radians(self.config.association_yaw_deg)
                or size_difference > self.config.association_size_tolerance_m):
            return None
        return (
            distance / max(self.config.association_distance_m, 1e-6)
            + yaw_difference / max(
                math.radians(self.config.association_yaw_deg), 1e-6)
            + size_difference / max(
                self.config.association_size_tolerance_m, 1e-6))

    @staticmethod
    def _blend_axis(old_yaw: float, new_yaw: float, weight: float) -> float:
        # Doubling the angle makes axes pi-periodic instead of 2*pi-periodic.
        old_x = math.cos(2.0 * old_yaw)
        old_y = math.sin(2.0 * old_yaw)
        new_x = math.cos(2.0 * new_yaw)
        new_y = math.sin(2.0 * new_yaw)
        blended = math.atan2(
            (1.0 - weight) * old_y + weight * new_y,
            (1.0 - weight) * old_x + weight * new_x) / 2.0
        return normalize_angle(blended)

    def _update_slot(
            self, slot: ParkingSlot, candidate: ParkingCandidate,
            now: float) -> None:
        # Reduce observation jitter while still allowing the map estimate to settle.
        weight = min(0.35, 1.0 / (slot.observation_count + 1.0))
        slot.center_x = (
            (1.0 - weight) * slot.center_x + weight * candidate.center_x)
        slot.center_y = (
            (1.0 - weight) * slot.center_y + weight * candidate.center_y)
        slot.yaw = self._blend_axis(slot.yaw, candidate.yaw, weight)
        slot.width = (
            (1.0 - weight) * slot.width + weight * candidate.width)
        slot.length = (
            (1.0 - weight) * slot.length + weight * candidate.length)
        slot.confidence = min(
            1.0,
            0.75 * slot.confidence + 0.25 * candidate.confidence + 0.05)
        slot.observation_count += 1
        slot.last_seen_time = float(now)
        slot.status = (
            'CONFIRMED'
            if slot.observation_count >= self.config.confirm_observations
            else 'CANDIDATE')

    def update(
            self, candidates: Sequence[ParkingCandidate], now: float,
            occupied_slot_ids: Optional[Iterable[str]] = None
    ) -> Tuple[List[ParkingSlot], List[str]]:
        """Update tracks and return (current slots, deleted slot IDs)."""
        occupied: Set[str] = set(occupied_slot_ids or ())
        unmatched_slot_ids: Set[str] = set(self._slots)
        for candidate in sorted(
                candidates, key=lambda value: value.confidence, reverse=True):
            matches = []
            for slot_id in unmatched_slot_ids:
                cost = self._association_cost(self._slots[slot_id], candidate)
                if cost is not None:
                    matches.append((cost, slot_id))
            if matches:
                _, slot_id = min(matches)
                self._update_slot(self._slots[slot_id], candidate, now)
                unmatched_slot_ids.remove(slot_id)
                continue

            parking_type = (
                candidate.parking_type
                if candidate.parking_type in VALID_PARKING_TYPES
                else 'UNKNOWN')
            slot_id = self._allocate_id(parking_type)
            self._slots[slot_id] = ParkingSlot(
                slot_id=slot_id,
                parking_type=parking_type,
                center_x=float(candidate.center_x),
                center_y=float(candidate.center_y),
                yaw=normalize_angle(candidate.yaw),
                width=float(candidate.width),
                length=float(candidate.length),
                confidence=max(0.0, min(1.0, candidate.confidence)),
                observation_count=1,
                first_seen_time=float(now),
                last_seen_time=float(now),
                status=(
                    'CONFIRMED'
                    if self.config.confirm_observations <= 1
                    else 'CANDIDATE'),
            )
        deleted = []
        for slot_id in list(unmatched_slot_ids):
            slot = self._slots[slot_id]
            age = max(0.0, float(now) - slot.last_seen_time)
            if age >= self.config.delete_timeout_sec:
                deleted.append(slot_id)
                del self._slots[slot_id]
            elif slot_id in occupied:
                slot.status = 'OCCUPIED'
                slot.confidence = max(0.0, slot.confidence * 0.95)
            elif age >= self.config.lost_timeout_sec:
                slot.status = 'TEMPORARILY_LOST'
                slot.confidence = max(0.0, slot.confidence * 0.98)
        return self.slots, sorted(deleted)

    def clear(self) -> List[str]:
        deleted = sorted(self._slots)
        self._slots.clear()
        return deleted

    def replace_slots(
            self, slots: Iterable[ParkingSlot],
            rebase_last_seen_time: Optional[float] = None) -> None:
        self._slots = {}
        self._next_ids = {'T': 1, 'PARALLEL': 1, 'UNKNOWN': 1}
        pattern = re.compile(r'^(T|P|U)_(\d+)$')
        reverse_prefix = {'T': 'T', 'P': 'PARALLEL', 'U': 'UNKNOWN'}
        for slot in slots:
            if rebase_last_seen_time is not None:
                slot.last_seen_time = float(rebase_last_seen_time)
                if slot.status in ('CONFIRMED', 'OCCUPIED'):
                    slot.status = 'TEMPORARILY_LOST'
            self._slots[slot.slot_id] = slot
            match = pattern.match(slot.slot_id)
            if match:
                parking_type = reverse_prefix[match.group(1)]
                self._next_ids[parking_type] = max(
                    self._next_ids[parking_type], int(match.group(2)) + 1)


def map_metadata_compatible(
        saved: Dict, current: MapMetadata,
        resolution_tolerance: float = 0.001,
        origin_tolerance_m: float = 0.5,
        origin_yaw_tolerance_deg: float = 5.0) -> Tuple[bool, str]:
    """Compare stored map resolution/origin with the active map."""
    try:
        saved_resolution = float(saved['map_resolution'])
        saved_origin = saved['map_origin']
        saved_x = float(saved_origin['x'])
        saved_y = float(saved_origin['y'])
        saved_yaw = float(saved_origin.get('yaw', 0.0))
    except (KeyError, TypeError, ValueError) as error:
        return False, f'invalid saved map metadata: {error}'

    resolution_difference = abs(saved_resolution - current.resolution)
    if resolution_difference > resolution_tolerance:
        return (
            False,
            f'map resolution differs by {resolution_difference:.6f} m/cell')
    origin_distance = math.hypot(
        saved_x - current.origin_x, saved_y - current.origin_y)
    if origin_distance > origin_tolerance_m:
        return False, f'map origin differs by {origin_distance:.3f} m'
    # Unlike a slot rectangle axis, a map origin is directional: yaw and
    # yaw + pi describe different world coordinates.
    yaw_difference = abs(normalize_angle(saved_yaw - current.origin_yaw))
    if yaw_difference > math.radians(origin_yaw_tolerance_deg):
        return (
            False,
            f'map origin yaw differs by {math.degrees(yaw_difference):.3f} deg')
    return True, 'compatible'


def slots_payload(
        slots: Iterable[ParkingSlot], map_frame: str,
        metadata: MapMetadata, saved_at: Optional[str] = None) -> Dict:
    return {
        'format_version': 1,
        'map_frame': str(map_frame),
        'map_resolution': float(metadata.resolution),
        'map_origin': {
            'x': float(metadata.origin_x),
            'y': float(metadata.origin_y),
            'yaw': float(metadata.origin_yaw),
        },
        'map_size': {
            'width': int(metadata.width),
            'height': int(metadata.height),
        },
        'saved_at': saved_at or datetime.now(timezone.utc).isoformat(),
        'slots': [slot.to_dict() for slot in slots],
    }


def save_slots_json(
        path: str, slots: Iterable[ParkingSlot], map_frame: str,
        metadata: MapMetadata, saved_at: Optional[str] = None) -> Dict:
    """Atomically save slots and return the serialized payload."""
    payload = slots_payload(slots, map_frame, metadata, saved_at)
    destination = Path(os.path.expanduser(path))
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=str(destination.parent),
                prefix=f'.{destination.name}.', suffix='.tmp',
                delete=False) as temporary:
            temporary_name = temporary.name
            json.dump(payload, temporary, ensure_ascii=False, indent=2)
            temporary.write('\n')
        os.replace(temporary_name, destination)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)
    return payload


def load_slots_json(path: str) -> Tuple[List[ParkingSlot], Dict]:
    source = Path(os.path.expanduser(path))
    with source.open('r', encoding='utf-8') as stream:
        payload = json.load(stream)
    if payload.get('format_version') != 1:
        raise ValueError(
            f'unsupported format_version: {payload.get("format_version")}')
    raw_slots = payload.get('slots')
    if not isinstance(raw_slots, list):
        raise ValueError('slots must be a JSON array')
    return [ParkingSlot.from_dict(item) for item in raw_slots], payload

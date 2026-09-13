import math

import pytest

from lidar_parking_planner.parking_space_detector import (
    MapMetadata,
    ParkingCandidate,
)
from lidar_parking_planner.slot_tracker import (
    SlotTracker,
    TrackerConfig,
    load_slots_json,
    map_metadata_compatible,
    save_slots_json,
)


def candidate(x=1.0, y=-1.5, confidence=0.9):
    return ParkingCandidate(
        parking_type='T',
        center_x=x,
        center_y=y,
        yaw=-math.pi / 2.0,
        width=1.2,
        length=1.8,
        confidence=confidence,
    )


def test_same_location_updates_the_same_slot_id():
    tracker = SlotTracker(TrackerConfig(confirm_observations=3))
    tracker.update([candidate()], now=1.0)
    first_id = tracker.slots[0].slot_id
    tracker.update([candidate(x=1.1, y=-1.45)], now=2.0)
    tracker.update([candidate(x=0.95, y=-1.55)], now=3.0)

    assert len(tracker.slots) == 1
    assert tracker.slots[0].slot_id == first_id == 'T_0001'
    assert tracker.slots[0].observation_count == 3
    assert tracker.slots[0].status == 'CONFIRMED'


def test_temporary_missed_observation_does_not_delete_slot():
    tracker = SlotTracker(TrackerConfig(
        confirm_observations=1,
        lost_timeout_sec=10.0,
        delete_timeout_sec=60.0,
    ))
    tracker.update([candidate()], now=0.0)
    tracker.update([], now=5.0)
    assert len(tracker.slots) == 1
    assert tracker.slots[0].status == 'CONFIRMED'

    tracker.update([], now=11.0)
    assert len(tracker.slots) == 1
    assert tracker.slots[0].status == 'TEMPORARILY_LOST'

    _, deleted = tracker.update([], now=61.0)
    assert deleted == ['T_0001']
    assert tracker.slots == []


def test_occupied_evidence_changes_status_without_deleting():
    tracker = SlotTracker(TrackerConfig(confirm_observations=1))
    tracker.update([candidate()], now=0.0)
    tracker.update([], now=1.0, occupied_slot_ids=['T_0001'])
    assert tracker.slots[0].status == 'OCCUPIED'


def test_save_load_json_round_trip(tmp_path):
    tracker = SlotTracker(TrackerConfig(confirm_observations=1))
    tracker.update([candidate()], now=123.0)
    metadata = MapMetadata(
        resolution=0.05,
        width=200,
        height=100,
        origin_x=-5.0,
        origin_y=-2.5,
        origin_yaw=0.0,
    )
    path = tmp_path / 'parking_slots.json'
    saved = save_slots_json(
        str(path), tracker.slots, 'map', metadata,
        saved_at='2026-01-02T03:04:05+00:00')
    loaded_slots, loaded = load_slots_json(str(path))

    assert loaded == saved
    assert loaded['format_version'] == 1
    assert loaded['map_frame'] == 'map'
    assert loaded_slots[0].to_dict() == tracker.slots[0].to_dict()


def test_map_metadata_mismatch_is_detected():
    current = MapMetadata(
        resolution=0.05,
        width=200,
        height=100,
        origin_x=-5.0,
        origin_y=-2.5,
        origin_yaw=0.0,
    )
    compatible_payload = {
        'map_resolution': 0.05,
        'map_origin': {'x': -5.0, 'y': -2.5, 'yaw': 0.0},
    }
    assert map_metadata_compatible(compatible_payload, current)[0]

    wrong_resolution = dict(compatible_payload, map_resolution=0.10)
    compatible, reason = map_metadata_compatible(
        wrong_resolution, current)
    assert not compatible
    assert 'resolution' in reason

    wrong_origin = {
        **compatible_payload,
        'map_origin': {'x': 2.0, 'y': -2.5, 'yaw': 0.0},
    }
    compatible, reason = map_metadata_compatible(wrong_origin, current)
    assert not compatible
    assert 'origin' in reason

    wrong_yaw = {
        **compatible_payload,
        'map_origin': {'x': -5.0, 'y': -2.5, 'yaw': math.pi},
    }
    compatible, reason = map_metadata_compatible(wrong_yaw, current)
    assert not compatible
    assert 'yaw' in reason


def test_loaded_ids_advance_the_runtime_counter(tmp_path):
    tracker = SlotTracker(TrackerConfig(confirm_observations=1))
    tracker.update([candidate()], now=1.0)
    restored = SlotTracker(TrackerConfig(confirm_observations=1))
    restored.replace_slots(tracker.slots)
    restored.update([candidate(x=3.0)], now=2.0)
    assert {slot.slot_id for slot in restored.slots} == {'T_0001', 'T_0002'}


def test_invalid_delete_timeout_is_rejected():
    with pytest.raises(ValueError):
        SlotTracker(TrackerConfig(
            lost_timeout_sec=10.0,
            delete_timeout_sec=5.0,
        ))

from camera_yolo_inference.stop_detection import (
    contains_stop,
    update_box_confirmation,
)


def test_detects_stop_and_stop_line_names():
    assert contains_stop([{"class_id": 1, "confidence": 0.8}], {1: "stop"}, 0.25)
    assert contains_stop([{"class_id": 2, "confidence": 0.8}], {2: "stop_line"}, 0.25)


def test_rejects_other_classes_and_low_confidence():
    assert not contains_stop([{"class_id": 1, "confidence": 0.8}], {1: "road"}, 0.25)
    assert not contains_stop([{"class_id": 1, "confidence": 0.2}], {1: "stop"}, 0.25)


def test_three_frames_at_matching_position_are_confirmed():
    tracker = None
    for expected in (False, False, True):
        confirmed, tracker = update_box_confirmation(
            (100, 100, 150, 150), tracker, required_frames=3,
            minimum_iou=0.3)
        assert confirmed is expected


def test_position_jump_resets_the_confirmation_streak():
    tracker = None
    _, tracker = update_box_confirmation((100, 100, 150, 150), tracker)
    _, tracker = update_box_confirmation((102, 100, 152, 150), tracker)
    confirmed, tracker = update_box_confirmation((300, 100, 350, 150), tracker)
    assert not confirmed
    assert tracker["frames"] == 1

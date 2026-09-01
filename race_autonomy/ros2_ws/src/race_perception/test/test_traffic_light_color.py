import numpy as np

from race_perception.traffic_light_color import (
    best_labeled_light,
    finish_signal_from_bgr,
    state_from_class_name,
    update_light_confirmation,
)


def test_opencv_finish_signal_classifies_red_and_green_lamps():
    red=np.zeros((200,300,3),np.uint8);red[40:70,130:160]=(0,0,255)
    green=np.zeros((200,300,3),np.uint8);green[40:70,130:160]=(0,255,0)
    assert finish_signal_from_bgr(red)[0]=="RED"
    assert finish_signal_from_bgr(green)[0]=="GREEN"


def test_opencv_finish_signal_abstains_without_dominant_color():
    image=np.zeros((200,300,3),np.uint8)
    image[40:70,100:130]=(0,0,255)
    image[40:70,170:200]=(0,255,0)
    assert finish_signal_from_bgr(image)[0]=="UNKNOWN"


def test_color_requires_three_consecutive_frames():
    state, tracker = update_light_confirmation("RED", [0,0,10,10], True, None, 3)
    assert state == "UNKNOWN"
    state, tracker = update_light_confirmation("RED", [0,0,10,10], True, tracker, 3)
    assert state == "UNKNOWN"
    state, tracker = update_light_confirmation("RED", [0,0,10,10], True, tracker, 3)
    assert state == "RED"


def test_unknown_resets_confirmation():
    _, tracker = update_light_confirmation("GREEN", [0,0,10,10], True, None, 3)
    _, tracker = update_light_confirmation("GREEN", [0,0,10,10], True, tracker, 3)
    state, tracker = update_light_confirmation("UNKNOWN", [0,0,10,10], True, tracker, 3)
    assert state == "UNKNOWN"
    assert tracker is None
    state, tracker = update_light_confirmation("GREEN", [0,0,10,10], True, tracker, 3)
    assert state == "UNKNOWN"


def test_confirmation_resets_outside_eligible_zone():
    tracker = {"candidate": "GREEN", "box": [0,0,10,10],
               "consecutive_frames": 2}
    assert update_light_confirmation(
        "GREEN", [0,0,10,10], False, tracker, 3) == ("UNKNOWN", None)


def test_position_jump_restarts_three_frame_confirmation():
    _, tracker=update_light_confirmation("RED",[0,0,10,10],True,None,3)
    _, tracker=update_light_confirmation("RED",[1,0,11,10],True,tracker,3)
    state,tracker=update_light_confirmation("RED",[30,0,40,10],True,tracker,3)
    assert state=="UNKNOWN" and tracker["consecutive_frames"]==1


def test_different_color_restarts_confirmation():
    _, tracker = update_light_confirmation("RED", [0,0,10,10], True, None, 3)
    _, tracker = update_light_confirmation("RED", [0,0,10,10], True, tracker, 3)
    state, tracker = update_light_confirmation("YELLOW", [0,0,10,10], True, tracker, 3)
    assert state == "UNKNOWN"
    assert tracker["candidate"] == "YELLOW"
    assert tracker["consecutive_frames"] == 1


def test_yolo_labels_map_directly_to_signal_states():
    assert state_from_class_name("R_light") == "RED"
    assert state_from_class_name("Y_light") == "YELLOW"
    assert state_from_class_name("G_light") == "GREEN"
    assert state_from_class_name("Left") == "LEFT"
    assert state_from_class_name("etc_light") == "UNKNOWN"


def test_highest_confidence_color_label_wins():
    detections = [
        {"class_name": "R_light", "confidence": 0.61, "xyxy": [1, 2, 3, 4]},
        {"class_name": "G_light", "confidence": 0.88, "xyxy": [5, 6, 7, 8]},
    ]
    best, count = best_labeled_light(
        detections, ["R_light", "Y_light", "G_light", "etc_light"], 0.25)
    assert count == 2
    assert best["state"] == "GREEN"
    assert best["confidence"] == 0.88
    assert best["source"] == "yolo_label"


def test_unknown_and_low_confidence_labels_do_not_override_color():
    detections = [
        {"class_name": "etc_light", "confidence": 0.99},
        {"class_name": "R_light", "confidence": 0.20},
    ]
    best, count = best_labeled_light(
        detections, ["R_light", "Y_light", "G_light", "etc_light"], 0.25)
    assert count == 1
    assert best["state"] == "UNKNOWN"

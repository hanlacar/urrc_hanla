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


def test_color_requires_three_seconds_of_matching_observations():
    state, tracker = update_light_confirmation("RED", True, None, 10.0, 3.0)
    assert state == "UNKNOWN"
    state, tracker = update_light_confirmation("RED", True, tracker, 12.99, 3.0)
    assert state == "UNKNOWN"
    state, tracker = update_light_confirmation("RED", True, tracker, 13.0, 3.0)
    assert state == "RED"


def test_unknown_pauses_without_resetting_confirmation():
    _, tracker = update_light_confirmation("GREEN", True, None, 1.0, 3.0)
    _, tracker = update_light_confirmation("GREEN", True, tracker, 2.0, 3.0)
    state, tracker = update_light_confirmation("UNKNOWN", True, tracker, 5.0, 3.0)
    assert state == "UNKNOWN"
    assert tracker["candidate"] == "GREEN"
    assert tracker["accumulated_sec"] == 1.0
    state, tracker = update_light_confirmation("GREEN", True, tracker, 7.0, 3.0)
    assert state == "GREEN"


def test_confirmation_resets_outside_eligible_zone():
    tracker = {"candidate": "GREEN", "accumulated_sec": 2.0, "last_time": 3.0}
    assert update_light_confirmation(
        "GREEN", False, tracker, 4.0, 3.0) == ("UNKNOWN", None)


def test_different_color_restarts_confirmation():
    _, tracker = update_light_confirmation("RED", True, None, 1.0, 3.0)
    _, tracker = update_light_confirmation("RED", True, tracker, 3.0, 3.0)
    state, tracker = update_light_confirmation("YELLOW", True, tracker, 3.1, 3.0)
    assert state == "UNKNOWN"
    assert tracker["candidate"] == "YELLOW"
    assert tracker["accumulated_sec"] == 0.0


def test_yolo_labels_map_directly_to_signal_states():
    assert state_from_class_name("R_light") == "RED"
    assert state_from_class_name("Y_light") == "YELLOW"
    assert state_from_class_name("G_light") == "GREEN"
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

import cv2
import numpy as np

from race_perception.traffic_light_color import (classify_traffic_light_bgr,
                                                 clipped_box,
                                                 fuse_traffic_light_state,
                                                 update_light_vote)


def test_color_uses_majority_of_five_valid_frames_inside_zone():
    votes = []
    for candidate in ("RED", "GREEN", "RED", "YELLOW"):
        state, votes = update_light_vote(candidate, True, votes, 5)
        assert state == "UNKNOWN"
    state, votes = update_light_vote("RED", True, votes, 5)
    assert state == "RED"
    assert votes == ["RED", "GREEN", "RED", "YELLOW", "RED"]


def test_unknown_abstains_without_resetting_votes():
    votes = ["GREEN", "RED"]
    state, updated = update_light_vote("UNKNOWN", True, votes, 5)
    assert state == "UNKNOWN"
    assert updated == votes


def test_vote_resets_only_outside_eligible_zone():
    state, votes = update_light_vote("GREEN", False, ["GREEN", "RED"], 5)
    assert (state, votes) == ("UNKNOWN", [])


def test_tied_or_non_majority_vote_stays_unknown():
    votes = []
    for candidate in ("RED", "RED", "GREEN", "GREEN", "YELLOW"):
        state, votes = update_light_vote(candidate, True, votes, 5)
    assert state == "UNKNOWN"


def solid_hsv(hue):
    hsv = np.full((20, 20, 3), (hue, 255, 255), dtype=np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_primary_colors():
    assert classify_traffic_light_bgr(solid_hsv(0))[0] == "RED"
    assert classify_traffic_light_bgr(solid_hsv(28))[0] == "YELLOW"
    assert classify_traffic_light_bgr(solid_hsv(60))[0] == "GREEN"


def test_dark_crop_is_unknown():
    assert classify_traffic_light_bgr(np.zeros((20, 20, 3), np.uint8))[0] == "UNKNOWN"


def test_box_is_clipped_to_image():
    assert clipped_box([-10, -20, 110, 120], 100, 100) == (0, 0, 100, 100)


def test_yolo_color_fallback_and_hsv_conflict():
    assert fuse_traffic_light_state("G_light",.8,"UNKNOWN",0.) == ("GREEN",.8,"yolo_class_fallback")
    assert fuse_traffic_light_state("R_light",.9,"GREEN",.5) == ("UNKNOWN",0.,"conflict")

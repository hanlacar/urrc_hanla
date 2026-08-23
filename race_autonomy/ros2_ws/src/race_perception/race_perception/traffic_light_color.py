import cv2
import numpy as np


UNKNOWN = "UNKNOWN"

YOLO_LIGHT_STATES = {"R_light":"RED","Y_light":"YELLOW","G_light":"GREEN"}


def fuse_traffic_light_state(class_name,yolo_confidence,hsv_state,hsv_confidence,allow_yolo_fallback=True):
    """Fuse the learned color class with HSV, rejecting explicit conflicts."""
    yolo_state=YOLO_LIGHT_STATES.get(str(class_name),UNKNOWN)
    if hsv_state!=UNKNOWN and yolo_state!=UNKNOWN:
        if hsv_state!=yolo_state:return UNKNOWN,0.0,"conflict"
        return hsv_state,float(yolo_confidence)*float(hsv_confidence),"yolo_hsv_confirmed"
    if hsv_state!=UNKNOWN:return hsv_state,float(yolo_confidence)*float(hsv_confidence),"hsv"
    if allow_yolo_fallback and yolo_state!=UNKNOWN:return yolo_state,float(yolo_confidence),"yolo_class_fallback"
    return UNKNOWN,0.0,"unknown"


def classify_traffic_light_bgr(
    crop,
    saturation_min=90,
    value_min=100,
    minimum_pixels=20,
    dominance_ratio=1.35,
):
    """Classify a traffic-light crop using HSV pixel support."""
    if crop is None or crop.ndim != 3 or crop.size == 0:
        return UNKNOWN, 0.0, {"RED": 0, "YELLOW": 0, "GREEN": 0}
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    sat = int(saturation_min)
    val = int(value_min)
    ranges = {
        "RED": [((0, sat, val), (10, 255, 255)), ((170, sat, val), (179, 255, 255))],
        "YELLOW": [((18, sat, val), (38, 255, 255))],
        "GREEN": [((40, sat, val), (95, 255, 255))],
    }
    counts = {}
    for name, intervals in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in intervals:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8)),
            )
        counts[name] = int(cv2.countNonZero(mask))
    ordered = sorted(counts.items(), key=lambda item: item[1], reverse=True)
    winner, winner_count = ordered[0]
    runner_count = ordered[1][1]
    if winner_count < int(minimum_pixels):
        return UNKNOWN, 0.0, counts
    if runner_count > 0 and winner_count / runner_count < float(dominance_ratio):
        return UNKNOWN, 0.0, counts
    confidence = winner_count / max(crop.shape[0] * crop.shape[1], 1)
    return winner, float(confidence), counts


def clipped_box(xyxy, width, height, padding_ratio=0.05):
    if not isinstance(xyxy, (list, tuple)) or len(xyxy) != 4:
        return None
    x1, y1, x2, y2 = (float(value) for value in xyxy)
    pad_x = max((x2 - x1) * padding_ratio, 0.0)
    pad_y = max((y2 - y1) * padding_ratio, 0.0)
    x1 = max(0, min(width, int(x1 - pad_x)))
    y1 = max(0, min(height, int(y1 - pad_y)))
    x2 = max(0, min(width, int(x2 + pad_x)))
    y2 = max(0, min(height, int(y2 + pad_y)))
    return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

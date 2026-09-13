"""Helpers for publishing a compact stop-detection signal."""


def box_iou(a, b):
    if a is None or b is None or len(a) != 4 or len(b) != 4:
        return 0.0
    ax1, ay1, ax2, ay2 = (float(v) for v in a)
    bx1, by1, bx2, by2 = (float(v) for v in b)
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = iw * ih
    union = max(0.0, (ax2-ax1)*(ay2-ay1)) + max(0.0, (bx2-bx1)*(by2-by1)) - intersection
    return intersection / union if union > 0.0 else 0.0


def update_box_confirmation(box, tracker, required_frames=3, minimum_iou=0.3):
    if box is None:
        return False, None
    if tracker is not None and box_iou(box, tracker["box"]) >= float(minimum_iou):
        current = {"box": list(box), "frames": int(tracker["frames"]) + 1}
    else:
        current = {"box": list(box), "frames": 1}
    return current["frames"] >= max(1, int(required_frames)), current


def best_box(instances, names, accepted_names, minimum_confidence):
    accepted = {str(v).strip().lower().replace("_", "") for v in accepted_names}
    matches = []
    for item in instances:
        class_id = int(item["class_id"])
        name = names.get(class_id, str(class_id)) if isinstance(names, dict) else names[class_id]
        if (str(name).strip().lower().replace("_", "") in accepted and
                float(item.get("confidence", 0.0)) >= float(minimum_confidence) and
                len(item.get("xyxy", [])) == 4):
            matches.append(item)
    return (max(matches, key=lambda v: float(v.get("confidence", 0.0)))["xyxy"]
            if matches else None)


def contains_stop(instances, names, minimum_confidence):
    stop_names = {"stop", "stop_line"}
    for item in instances:
        class_id = int(item["class_id"])
        name = names.get(class_id, str(class_id)) if isinstance(names, dict) else names[class_id]
        if (
            str(name).strip().lower() in stop_names
            and float(item.get("confidence", 0.0)) >= float(minimum_confidence)
        ):
            return True
    return False

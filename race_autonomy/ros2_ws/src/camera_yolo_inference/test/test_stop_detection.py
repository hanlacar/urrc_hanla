from camera_yolo_inference.stop_detection import contains_stop


def test_detects_stop_and_stop_line_names():
    assert contains_stop([{"class_id": 1, "confidence": 0.8}], {1: "stop"}, 0.25)
    assert contains_stop([{"class_id": 2, "confidence": 0.8}], {2: "stop_line"}, 0.25)


def test_rejects_other_classes_and_low_confidence():
    assert not contains_stop([{"class_id": 1, "confidence": 0.8}], {1: "road"}, 0.25)
    assert not contains_stop([{"class_id": 1, "confidence": 0.2}], {1: "stop"}, 0.25)

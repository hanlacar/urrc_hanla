import numpy as np

from camera_navigation.camera_geometry import CameraGeometry
from camera_yolo_inference.stop_line_ground_node import stop_box_distance_m


def test_calibrated_bbox_pixel_returns_forward_camera_distance():
    geometry = CameraGeometry(
        [500, 0, 320, 0, 500, 240, 0, 0, 1],
        camera_xyz=(0.245, 0.004, 0.845),
        mount_rpy_deg=(0.0, -5.0, 0.0),
    )
    pixel = geometry.ground_to_pixel(2.0, 0.0)
    box = [pixel[0] - 1, pixel[1] - 1, pixel[0] + 1, pixel[1] + 1]
    assert np.isclose(stop_box_distance_m(box, geometry, 0.245), 1.755)


def test_invalid_bbox_is_rejected():
    geometry = CameraGeometry([500, 0, 320, 0, 500, 240, 0, 0, 1])
    assert stop_box_distance_m([1, 2, 1, 4], geometry, 0.0) is None

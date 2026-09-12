import math

from race_control.visual_slam_route import (align_route_to_pose,
                                            blend_bev_correction, load_route,
                                            nearest_index, nearest_stamp_skew,
                                            route_to_base,
                                            save_route, should_sample)


def test_sampling_and_round_trip(tmp_path):
    points = [(0.0, 0.0, 0.0)]
    assert not should_sample(points, 0.02, 0.0, 0.01, 0.1, 0.1)
    assert should_sample(points, 0.11, 0.0, 0.01, 0.1, 0.1)
    path = tmp_path / "route.json"
    save_route(str(path), points + [(1.0, 0.0, 0.0)])
    frame, loaded = load_route(str(path))
    assert frame == "map"
    assert loaded == points + [(1.0, 0.0, 0.0)]


def test_route_is_transformed_to_vehicle_frame():
    points = [(0.0, 0.0, 0.0), (0.0, 1.0, math.pi / 2),
              (0.0, 2.0, math.pi / 2)]
    local = route_to_base(points, 1, 0.0, 0.0, math.pi / 2, 0.2, 3.0)
    assert local[0][0] == 1.0
    assert abs(local[0][1]) < 1.0e-9
    assert nearest_index(points, 0.0, 1.1) == 1


def test_route_first_pose_can_be_aligned_to_current_vehicle_pose():
    route = [(2.0, 3.0, 0.0), (3.0, 3.0, 0.0)]
    aligned = align_route_to_pose(route, 10.0, -4.0, math.pi / 2)
    assert all(abs(a - b) < 1.0e-9
               for a, b in zip(aligned[0], (10.0, -4.0, math.pi / 2)))
    assert abs(aligned[1][0] - 10.0) < 1.0e-9
    assert abs(aligned[1][1] - -3.0) < 1.0e-9
    assert abs(aligned[1][2] - math.pi / 2) < 1.0e-9


def test_bev_correction_is_bounded():
    route = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    bev = [(1.0, 1.0), (3.0, 1.0)]
    fused, correction = blend_bev_correction(route, bev, 0.5, 0.30)
    assert all(abs(y - 0.15) < 1.0e-9 for _, y in fused)
    assert abs(correction - 0.15) < 1.0e-9


def test_nearest_timestamp_skew():
    assert abs(nearest_stamp_skew([10.0, 10.1, 10.2], 10.08)-0.02) < 1.0e-9
    assert math.isinf(nearest_stamp_skew([], 10.0))

import numpy as np

from camera_navigation.drivable_corridor import clearance_corridor_path


class FakeBev:
    resolution = 0.02
    forward_min = 0.3
    forward_max = 4.3
    left = 2.0

    def ground_to_bev(self, x, y):
        return ((self.left-y)/self.resolution,
                (self.forward_max-x)/self.resolution)

    def bev_to_ground(self, col, row):
        return (self.forward_max-row*self.resolution,
                self.left-col*self.resolution)


def test_vehicle_sized_road_hole_creates_a_side_bypass():
    road = np.zeros((200, 200), np.uint8)
    road[:, 25:175] = 255
    road[70:125, 80:120] = 0
    path = clearance_corridor_path(road, FakeBev(), 0.77, 0.10, row_step=2)
    obstacle_zone = path[(path[:, 0] >= 1.8) & (path[:, 0] <= 2.9)]
    assert len(path) > 20
    assert len(obstacle_zone) > 0
    assert np.max(np.abs(obstacle_zone[:, 1])) > 0.45


def test_corridor_rejects_a_road_narrower_than_vehicle_clearance():
    road = np.zeros((200, 200), np.uint8)
    road[:, 85:115] = 255
    path = clearance_corridor_path(road, FakeBev(), 0.77, 0.15)
    assert len(path) == 0


def test_previous_path_limits_lateral_jump():
    road = np.zeros((200, 200), np.uint8)
    road[:, 25:175] = 255
    previous = np.array([[0.3, 0.5], [4.3, 0.5]])
    path = clearance_corridor_path(
        road, FakeBev(), 0.77, 0.10, row_step=2,
        previous_path=previous, maximum_lateral_step_m=0.10)
    assert np.max(np.abs(path[:, 1]-0.5)) <= 0.100001


def test_path_uses_maximum_clearance_between_wall_and_boundary():
    road = np.zeros((200, 200), np.uint8)
    road[:, 25:175] = 255
    road[70:125, 100:150] = 0
    path = clearance_corridor_path(road, FakeBev(), 0.30, 0.05, row_step=2)
    obstacle_zone = path[(path[:, 0] >= 1.8) & (path[:, 0] <= 2.9)]
    assert len(obstacle_zone)
    assert np.all(obstacle_zone[:, 1] > 0.0)

import numpy as np

from camera_navigation.road_center_extractor import connected_road_center


class FakeBev:
    forward_min=.3
    def ground_to_bev(self,x,y):return 50.,99.
    def bev_to_ground(self,col,row):return (100.-row,(col-50.)*.02)


def test_ignores_disconnected_neighbouring_road():
    mask=np.zeros((100,100),np.uint8)
    mask[:,35:65]=255
    mask[:70,75:98]=255
    path=connected_road_center(mask,FakeBev(),close_kernel_pixels=1)
    assert len(path)==100
    assert np.max(np.abs(path[:,1])) < .02


def test_closes_word_sized_hole_without_switching_sides():
    mask=np.zeros((100,100),np.uint8);mask[:,30:70]=255
    mask[45:52,42:58]=0
    path=connected_road_center(mask,FakeBev(),close_kernel_pixels=17)
    assert len(path)==100
    assert np.max(np.abs(path[:,1])) < .02

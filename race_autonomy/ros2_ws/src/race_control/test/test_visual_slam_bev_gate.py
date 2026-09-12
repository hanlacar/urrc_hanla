import time
from types import SimpleNamespace
from unittest.mock import Mock

from race_control.visual_slam_route_node import VisualSlamRouteNode


def follower(bev_path, stamp=10., fresh=True):
    params = dict(minimum_route_points=2, map_frame='map', base_frame='base_link',
                  maximum_route_error_m=1.5, maximum_heading_error_deg=75.,
                  forward_min_m=.3, forward_max_m=6., minimum_local_points=2,
                  bev_timeout_sec=.3, slam_bev_max_skew_sec=.1,
                  bev_minimum_confidence=.45, require_bev=True,
                  align_route_to_start=False, bev_correction_gain=.55,
                  bev_maximum_correction_m=.35)
    return SimpleNamespace(
        p=params.__getitem__, points=[(i*.1, 0., 0.) for i in range(61)],
        source_points=[(i*.1, 0., 0.) for i in range(61)],
        origin_aligned=False, origin_pose=None,
        progress=0, recorded_pub=Mock(), make_path=Mock(), publish_invalid=Mock(),
        bev_time=time.monotonic() if fresh else time.monotonic()-1.,
        bev_stamp=stamp, odom_stamps=[10.], bev_valid=True,
        bev_confidence=.9, bev_path=bev_path)


def test_missing_bev_stops_even_with_valid_route_pose():
    node = follower([], stamp=None)
    VisualSlamRouteNode.follow_tick(node, 0., 0., 0.)
    node.publish_invalid.assert_called_once_with('bev_unavailable_or_unsynchronized')


def test_stale_bev_stops():
    node = follower([(1., .1), (3., .1)], fresh=False)
    VisualSlamRouteNode.follow_tick(node, 0., 0., 0.)
    node.publish_invalid.assert_called_once_with('bev_unavailable_or_unsynchronized')


def test_unsynchronized_bev_stops():
    node = follower([(1., .1), (3., .1)], stamp=9.)
    VisualSlamRouteNode.follow_tick(node, 0., 0., 0.)
    node.publish_invalid.assert_called_once_with('bev_unavailable_or_unsynchronized')


def test_bev_behind_vehicle_cannot_enable_following():
    node = follower([(-3., .1), (-1., .1)])
    VisualSlamRouteNode.follow_tick(node, 0., 0., 0.)
    node.publish_invalid.assert_called_once_with('bev_has_no_route_overlap')

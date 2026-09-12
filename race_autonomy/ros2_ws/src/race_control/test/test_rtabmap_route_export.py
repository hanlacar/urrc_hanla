import math
import sqlite3
import struct
import zlib

import pytest

from race_control.rtabmap_route_export import read_optimized_route, resample_route
from race_control.visual_slam_route import nearest_stamp_skew


def matrix(values, code, cv_type):
    return zlib.compress(struct.pack('<' + code * len(values), *values)) + struct.pack('<3i', 1, len(values), cv_type)


def test_uses_optimized_poses_in_timestamp_order(tmp_path):
    path = tmp_path / 'map.db'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE Node(id,map_id,stamp)')
        db.executemany('INSERT INTO Node VALUES(?,?,?)', [(1, 0, 1.), (2, 0, 2.)])
        db.execute('CREATE TABLE Admin(opt_ids,opt_poses)')
        a = [1,0,0,4, 0,1,0,5, 0,0,1,0]
        b = [1,0,0,6, 0,1,0,5, 0,0,1,0]
        db.execute('INSERT INTO Admin VALUES(?,?)', (matrix([2,1], 'i', 4), matrix(b+a, 'f', 5)))
    before = path.read_bytes()
    assert read_optimized_route(path) == [(4.,5.,0.), (6.,5.,0.)]
    assert before == path.read_bytes()


def test_resampling_preserves_heading_across_pi():
    points = resample_route([(0,0,math.radians(179)), (1,0,math.radians(-179))], .1)
    assert len(points) == 11
    assert abs(abs(points[5][2]) - math.pi) < 1e-6
    assert points[-1][:2] == (1., 0.)


def test_discontinuity_is_not_silently_bridged():
    with pytest.raises(ValueError, match='discontinuity'):
        resample_route([(0,0,0), (5,0,0)])


def test_no_bev_stamp_does_not_crash_follower():
    assert math.isinf(nearest_stamp_skew([1.], None))

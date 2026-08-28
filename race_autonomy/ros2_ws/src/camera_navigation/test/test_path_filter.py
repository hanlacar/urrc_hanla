import numpy as np

from camera_navigation.path_filter import stabilize_path


def test_spatial_filter_removes_single_row_spike():
    x=np.linspace(.3,3.,21);y=np.zeros_like(x);y[10]=.8
    filtered=stabilize_path(np.column_stack((x,y)),spatial_window=7)
    assert abs(filtered[10,1]) < .01


def test_temporal_filter_limits_lateral_jump():
    x=np.linspace(.3,3.,21)
    old=np.column_stack((x,np.zeros_like(x)))
    new=np.column_stack((x,np.ones_like(x)))
    filtered=stabilize_path(new,old,new_path_weight=.5,
                            maximum_lateral_shift_m=.15)
    assert np.allclose(filtered[:,1],.15)


def test_new_nonoverlapping_segment_is_not_held_by_old_path():
    old=np.array([[1.,0.],[2.,0.]])
    new=np.array([[.3,.4],[1.,.4],[2.,.4],[3.,.4]])
    filtered=stabilize_path(new,old,spatial_window=1,new_path_weight=.5,
                            maximum_lateral_shift_m=.5)
    assert filtered[0,1] == .4 and filtered[-1,1] == .4
    assert np.allclose(filtered[1:3,1],.2)

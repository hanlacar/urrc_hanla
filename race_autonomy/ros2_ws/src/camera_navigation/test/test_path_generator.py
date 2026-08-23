import numpy as np
from camera_navigation.path_generator import *
x=np.linspace(1,5,20); left=np.c_[x,.8+.05*x*x]; right=np.c_[x,-.8+.05*x*x]
def test_both_straight_center():
    p,m=generate_path(np.c_[x,np.ones(20)],np.c_[x,-np.ones(20)]);assert m==BOTH_BOUNDARIES and np.max(abs(p[:,1]))<1e-8
def test_left_curve():
    p,m=generate_path(left,right);assert m==BOTH_BOUNDARIES and p[-1,1]>p[0,1]
def test_right_curve():
    p,m=generate_path(np.c_[x,.8-.05*x*x],np.c_[x,-.8-.05*x*x]);assert m==BOTH_BOUNDARIES and p[-1,1]<p[0,1]
def test_left_only_precedes_road():assert generate_path(left,None,np.c_[x,np.zeros(20)])[1]==LEFT_ONLY
def test_right_only_precedes_road():assert generate_path(None,right,np.c_[x,np.zeros(20)])[1]==RIGHT_ONLY
def test_road_only():assert generate_path(None,None,np.c_[x,np.zeros(20)])[1]==ROAD_ONLY
def test_temporal_hold_within_timeout():assert generate_path(previous=np.c_[x,np.zeros(20)],previous_age_s=.2,hold_timeout_s=.3)[1]==TEMPORAL_HOLD
def test_temporal_hold_expires():assert generate_path(previous=np.c_[x,np.zeros(20)],previous_age_s=.4,hold_timeout_s=.3)[1]==INVALID
def test_invalid_empty():assert generate_path()[1]==INVALID
def test_road_bounds_reject_outside():assert not path_inside_bounds([[1,0],[9,0]],.3,8,2,2)
def test_road_bounds_accept_inside():assert path_inside_bounds([[1,0],[7,1]],.3,8,2,2)
def test_reacquire_jump_is_blended():
    detected=np.c_[x,np.ones(20)]; out=blend_reacquire(np.c_[x,np.zeros(20)],detected,.25);assert np.max(abs(out[:,1]))==.25

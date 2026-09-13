import numpy as np
from camera_navigation.speed_planner import target_speed
from camera_navigation.path_generator import BOTH_BOUNDARIES,LEFT_ONLY,RIGHT_ONLY,ROAD_ONLY
x=np.linspace(.1,3,30); straight=np.c_[x,np.zeros(30)]; curve=np.c_[x,.5*x*x]
def test_curvature_reduces_speed():assert target_speed(curve,BOTH_BOUNDARIES,max_lateral_accel=.25)<target_speed(straight,BOTH_BOUNDARIES,max_lateral_accel=.25)
def test_turn_speed_limit():assert target_speed(straight,BOTH_BOUNDARIES,turn_active=True,turn=.4)==.4
def test_left_single_limit():assert target_speed(straight,LEFT_ONLY,single=.6)==.6
def test_right_single_limit():assert target_speed(straight,RIGHT_ONLY,single=.6)==.6
def test_road_limit():assert target_speed(straight,ROAD_ONLY,road=.35)==.35
def test_invalid_immediate_zero():assert target_speed(straight,BOTH_BOUNDARIES,valid=False)==0
def test_stale_immediate_zero():assert target_speed(straight,BOTH_BOUNDARIES,stale=True)==0
def test_abort_immediate_zero():assert target_speed(straight,BOTH_BOUNDARIES,turn_abort=True)==0

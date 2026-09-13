import numpy as np
from camera_navigation.speed_planner import max_curvature
from camera_navigation.turn_path_generator import TurnState,TurnStateMachine,bezier_turn
def test_left_bezier_positive_y():assert np.mean(bezier_turn(-1)[:,1])>0
def test_right_bezier_negative_y():assert np.mean(bezier_turn(1)[:,1])<0
def test_start_point_and_heading_continuity():
    p=bezier_turn(-1);assert np.allclose(p[0],0) and abs(np.arctan2(*(p[1]-p[0])[::-1]))<.05
def test_minimum_radius():assert max_curvature(bezier_turn(-1,min_turn_radius_m=1.5))<=1/1.5+.03
def execute_machine(valid=True):
    s=TurnStateMachine(progress_timeout_s=1);s.update(-1,0,valid);s.update(-1,0,valid,intersection=True);s.update(-1,0,valid,now=0);return s
def test_imu_yaw_progress():
    s=execute_machine();assert s.state==TurnState.TURN_EXECUTE and s.update(-1,60,True,now=.5)==TurnState.EXIT_SEARCH
def test_invalid_yaw_reduces_confidence():
    s=execute_machine(False);s.update(-1,None,False,now=.5);assert s.confidence<1 and s.state==TurnState.TURN_EXECUTE
def test_invalid_yaw_timeout_aborts():
    s=execute_machine(False);assert s.update(-1,None,False,now=1.1)==TurnState.ABORT
def test_exit_and_reacquire():
    s=execute_machine();s.update(-1,60,True,now=.5);assert s.update(-1,60,True,exit_visible=True)==TurnState.LANE_REACQUIRE;assert s.update(0,60,True,lane_visible=True)==TurnState.LANE_FOLLOW
def test_all_points_finite_and_bounded():
    p=bezier_turn(-1,min_turn_radius_m=1.5);assert np.isfinite(p).all() and np.max(abs(p))<=1.5+1e-9

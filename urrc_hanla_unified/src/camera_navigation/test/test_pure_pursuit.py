import numpy as np
from camera_navigation.pure_pursuit import rate_limit, steering_angle_deg
x=np.linspace(.2,3,20); args=(1.,.3,.5,2.,.5,25.)
def test_straight_zero():assert abs(steering_angle_deg(np.c_[x,np.zeros(20)],*args))<1e-8
def test_left_positive():assert steering_angle_deg(np.c_[x,.1*x*x],*args)>0
def test_right_negative():assert steering_angle_deg(np.c_[x,-.1*x*x],*args)<0
def test_saturation():assert abs(steering_angle_deg([[.1,10]],*args))<=25
def test_rate_limit():assert rate_limit(20,0,10,.5)==5
def test_no_forward_point():assert steering_angle_deg([[-2,1],[-1,1]],*args)==0
def test_empty_path():assert steering_angle_deg([], *args)==0
def test_nan_path_safe():assert steering_angle_deg([[1,np.nan],[2,np.nan]],*args)==0
def test_nan_parameter_safe():assert steering_angle_deg([[1,0]],np.nan,.3,.5,2,.5,25)==0

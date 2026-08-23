import cv2
import numpy as np
import pytest

from camera_navigation.camera_geometry import CameraGeometry
from camera_navigation.geometry_validation import validate_camera_info

K=[387.36212158203125,0.,317.412841796875,0.,386.8159484863281,247.76675415039062,0.,0.,1.]
D=[-0.05530029907822609,0.0644579529762268,0.0000892298630787991,0.0009998686145991087,-0.02068493515253067]
R=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
P=[387.36212158203125,0.,317.412841796875,0.,0.,386.8159484863281,247.76675415039062,0.,0.,0.,1.,0.]

def actual_info(**changes):
    value={"frame_id":"camera_color_optical_frame","width":640,"height":480,"distortion_model":"plumb_bob","d":D,"k":K,"r":R,"p":P,"binning_x":0,"binning_y":0,"roi":{"x_offset":0,"y_offset":0,"width":640,"height":480,"do_rectify":False},"timestamp":1.}
    value.update(changes);return value
def geometry(d=D):return CameraGeometry(K,distortion_coeffs=d,distortion_model="plumb_bob")
def ray_to_raw(g,ray):
    pixels,_=cv2.projectPoints(np.asarray(ray).reshape(1,1,3),np.zeros(3),np.zeros(3),g.k,g.distortion);return pixels.reshape(2)

def test_actual_camera_info_passes():assert validate_camera_info(actual_info())==(True,"ok")
def test_actual_frame_and_resolution():
    data=actual_info();assert data["frame_id"]=="camera_color_optical_frame" and (data["width"],data["height"])==(640,480)
def test_center_raw_undistortion_finite():assert np.isfinite(geometry().optical_ray(317.4,247.8)).all()
@pytest.mark.parametrize("pixel",[(0.,247.8),(639.,247.8),(317.4,0.),(317.4,479.)])
def test_image_edges_undistort_finite(pixel):assert np.isfinite(geometry().optical_ray(*pixel)).all()
@pytest.mark.parametrize("pixel",[(317.4,400.),(0.,400.),(639.,400.),(100.,470.),(540.,470.)])
def test_raw_undistort_distort_roundtrip(pixel):
    g=geometry();raw=ray_to_raw(g,g.optical_ray(*pixel));assert np.linalg.norm(raw-pixel)<=1.
def test_zero_distortion_matches_pinhole():
    g=geometry([0.]*5);ray=g.optical_ray(100,400);expected=np.linalg.inv(np.asarray(K).reshape(3,3))@np.array([100,400,1]);expected/=np.linalg.norm(expected);assert np.allclose(ray,expected)
@pytest.mark.parametrize("bad_d",[[0.]*4,[0.]*6,[]])
def test_invalid_distortion_length_rejected(bad_d):
    with pytest.raises(ValueError):geometry(bad_d)
@pytest.mark.parametrize("bad",[np.nan,np.inf])
def test_nonfinite_distortion_rejected(bad):
    d=list(D);d[0]=bad
    with pytest.raises(ValueError):geometry(d)
def test_unsupported_distortion_model_rejected():
    with pytest.raises(ValueError):CameraGeometry(K,distortion_coeffs=D,distortion_model="equidistant")
def test_rectified_pixel_contract_rejected():
    with pytest.raises(ValueError):CameraGeometry(K,distortion_coeffs=D,pixel_contract="rectified")
def test_left_raw_pixel_maps_base_positive_y():assert geometry().pixel_to_ground(0,400)[1]>0
def test_right_raw_pixel_maps_base_negative_y():assert geometry().pixel_to_ground(639,400)[1]<0
def test_camera_info_invalid_d_length():assert not validate_camera_info(actual_info(d=D[:4]))[0]
def test_camera_info_nonfinite_d():
    d=list(D);d[2]=np.nan;assert not validate_camera_info(actual_info(d=d))[0]
def test_camera_info_unsupported_model():assert not validate_camera_info(actual_info(distortion_model="equidistant"))[0]
def test_actual_rectification_binning_and_roi_contract():assert validate_camera_info(actual_info())[0]

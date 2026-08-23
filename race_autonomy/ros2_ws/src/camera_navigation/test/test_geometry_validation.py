from pathlib import Path
import numpy as np
import pytest
import yaml

from camera_navigation.camera_geometry import CameraGeometry
from camera_navigation.geometry_validation import (evaluate_reference_points,pitch_diagnostics,pitch_projection_diagnostics,save_report,validate_camera_info,validate_output_path,validate_reference)

K=[500.,0.,320.,0.,500.,240.,0.,0.,1.]
def info(**changes):
    value={"frame_id":"rgb_optical","width":640,"height":480,"distortion_model":"plumb_bob","d":[0.]*5,"k":K,"r":[1.,0.,0.,0.,1.,0.,0.,0.,1.],"p":[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.],"timestamp":10.}
    value.update(changes);return value
def point(name,x,y,u=None,v=None):return {"name":name,"pixel_u":u,"pixel_v":v,"expected_x_m":x,"expected_y_m":y}
def exact_point(name,x,y):
    uv=CameraGeometry(K).ground_to_pixel(x,y);return point(name,x,y,*uv)
def reference(points):return {"frame_id":"rgb_optical","image_width":640,"image_height":480,"points":points}

def test_camera_info_k_valid():assert validate_camera_info(info())[0]
@pytest.mark.parametrize("key,value",[("width",320),("height",240)])
def test_camera_info_resolution_rejected(key,value):assert not validate_camera_info(info(**{key:value}))[0]
def test_camera_info_frame_change_rejected():assert validate_camera_info(info(),initial_frame_id="other")== (False,"frame_id_changed")
@pytest.mark.parametrize("k",[[0.,0,320,0,500,240,0,0,1],[500,0,700,0,500,240,0,0,1],[500,0,320,0,np.nan,240,0,0,1]])
def test_invalid_camera_info_rejected(k):assert not validate_camera_info(info(k=k))[0]
def test_reference_resolution_rejected():assert not validate_reference({"image_width":320,"image_height":480},info())[0]
def test_null_reference_is_not_measured():
    result=evaluate_reference_points(info(),reference([point("pending",1,0)]));assert result["status"]=="NOT_RUN" and result["points"][0]["invalid_reason"]=="unmeasured_null_pixel"
def test_exact_reference_error_and_roundtrip():
    result=evaluate_reference_points(info(),reference([exact_point("center",1,0)]));item=result["points"][0];assert result["status"]=="PASS" and item["planar_error_m"]<1e-9 and item["pixel_round_trip_error_px"]<=1
def test_near_tolerance_passes():
    p=exact_point("near",.5,0);p["expected_x_m"]+=.14;assert evaluate_reference_points(info(),reference([p]))["status"]=="PASS"
def test_near_tolerance_fails():
    p=exact_point("near",.5,0);p["expected_x_m"]+=.16;assert evaluate_reference_points(info(),reference([p]))["status"]=="FAIL"
def test_far_tolerance_passes():
    p=exact_point("far",2,0);p["expected_x_m"]+=.24;assert evaluate_reference_points(info(),reference([p]))["status"]=="PASS"
def test_far_tolerance_fails():
    p=exact_point("far",2,0);p["expected_x_m"]+=.26;assert evaluate_reference_points(info(),reference([p]))["status"]=="FAIL"
def test_point_beyond_far_validation_range_fails():
    p=exact_point("outside",2,0);p["expected_x_m"]=3.1;assert evaluate_reference_points(info(),reference([p]))["status"]=="FAIL"
def test_lateral_tolerance():
    p=exact_point("lateral",1,.5);p["expected_y_m"]+=.14;assert evaluate_reference_points(info(),reference([p]))["status"]=="PASS"
def test_left_expected_positive():assert evaluate_reference_points(info(),reference([exact_point("left",1,.5)]))["status"]=="PASS"
def test_right_expected_negative():assert evaluate_reference_points(info(),reference([exact_point("right",1,-.5)]))["status"]=="PASS"
def test_lateral_sign_reversal_fails():
    p=exact_point("reversed",1,-.5);p["expected_y_m"]=.5;result=evaluate_reference_points(info(),reference([p]));assert result["status"]=="FAIL" and result["points"][0]["invalid_reason"]=="direction_or_lateral_sign"
def test_nonfinite_pixel_fails():
    result=evaluate_reference_points(info(),reference([point("bad",1,0,np.nan,2)]));assert result["status"]=="FAIL"
def test_partial_when_one_of_three_exceeds():
    bad=exact_point("bad",2,0);bad["expected_x_m"]+=.3;result=evaluate_reference_points(info(),reference([exact_point("a",1,0),exact_point("b",1,.5),bad]));assert result["status"]=="PARTIAL"
def test_result_yaml_saved(tmp_path):
    destination=save_report({"status":"PASS"},tmp_path/"report.yaml");assert yaml.safe_load(destination.read_text())["status"]=="PASS"
@pytest.mark.parametrize("tree",["src","install","build"])
def test_result_forbidden_in_workspace_trees(tmp_path,tree):
    with pytest.raises(ValueError):validate_output_path(tmp_path/tree/"camera_navigation"/"report.yaml")
def test_pitch_axes_point_increasingly_down():
    result=pitch_diagnostics();z=[item["base_z"] for item in result["samples"]];assert result["negative_pitch_points_down"] and z[0]>z[1]>z[2]
def test_active_vector_contract_is_explicit():assert pitch_diagnostics()["convention"].startswith("active vector rotation")
def test_pitch_projection_is_physically_consistent():assert pitch_projection_diagnostics(K)["more_negative_pitch_estimates_nearer_ground_for_same_lower_pixel"]

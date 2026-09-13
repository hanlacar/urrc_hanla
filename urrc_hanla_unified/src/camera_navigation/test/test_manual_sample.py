from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml

from camera_navigation.manual_sample import evaluate_manual_sample,load_manual_sample,load_metadata
from camera_navigation.mock_scenario_generator import make_masks

K=[387.36212158203125,0.,317.412841796875,0.,386.8159484863281,247.76675415039062,0.,0.,1.]
D=[-0.05530029907822609,0.0644579529762268,0.0000892298630787991,0.0009998686145991087,-0.02068493515253067]
def metadata():return {"image_topic":"/camera/image_raw","timestamp":1.,"frame_id":"camera_color_optical_frame","width":640,"height":480,"encoding":"rgb8","camera_info":{"k":K,"d":D,"distortion_model":"plumb_bob"},"camera_extrinsic":{"camera_x_m":0.,"camera_y_m":0.,"camera_z_m":.45,"camera_mount_roll_deg":0.,"camera_mount_pitch_deg":-5.,"camera_mount_yaw_deg":0.}}
def write_sample(root,scenario="STRAIGHT_BOTH"):
    root.mkdir();cv2.imwrite(str(root/"image_raw.png"),np.zeros((480,640,3),np.uint8));(root/"metadata.yaml").write_text(yaml.safe_dump(metadata()))
    for name,mask in zip(("road_mask.png","white_line_mask.png","yellow_line_mask.png"),make_masks(scenario)):cv2.imwrite(str(root/name),mask)
    return root

def test_rgb_640x480_pass(tmp_path):assert load_manual_sample(write_sample(tmp_path/"sample"))[1].shape==(480,640,3)
def test_missing_rgb_rejected_before_planning(tmp_path):
    root=write_sample(tmp_path/"sample");(root/"image_raw.png").unlink()
    with pytest.raises(ValueError):load_manual_sample(root)
def test_missing_mask_rejected_before_planning(tmp_path):
    root=write_sample(tmp_path/"sample");(root/"road_mask.png").unlink()
    with pytest.raises(ValueError):load_manual_sample(root)
def test_invalid_mask_never_produces_control_report(tmp_path):
    root=write_sample(tmp_path/"sample");mask=cv2.imread(str(root/"road_mask.png"),0);mask[0,0]=12;cv2.imwrite(str(root/"road_mask.png"),mask)
    with pytest.raises(ValueError):evaluate_manual_sample(root)
def test_resized_mask_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");cv2.imwrite(str(root/"road_mask.png"),np.zeros((240,320),np.uint8))
    with pytest.raises(ValueError):load_manual_sample(root)
def test_cropped_mask_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");cv2.imwrite(str(root/"white_line_mask.png"),np.zeros((470,640),np.uint8))
    with pytest.raises(ValueError):load_manual_sample(root)
def test_color_mask_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");cv2.imwrite(str(root/"yellow_line_mask.png"),np.zeros((480,640,3),np.uint8))
    with pytest.raises(ValueError):load_manual_sample(root)
def test_nonbinary_mask_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");mask=cv2.imread(str(root/"road_mask.png"),0);mask[400,320]=17;cv2.imwrite(str(root/"road_mask.png"),mask)
    with pytest.raises(ValueError):load_manual_sample(root)
@pytest.mark.parametrize("lane",["white_line_mask.png","yellow_line_mask.png"])
def test_zero_optional_lane_allowed(tmp_path,lane):
    root=write_sample(tmp_path/"sample");cv2.imwrite(str(root/lane),np.zeros((480,640),np.uint8));assert load_manual_sample(root)
def test_empty_road_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");cv2.imwrite(str(root/"road_mask.png"),np.zeros((480,640),np.uint8))
    with pytest.raises(ValueError):load_manual_sample(root)
def test_lane_outside_road_rejected(tmp_path):
    root=write_sample(tmp_path/"sample");mask=cv2.imread(str(root/"white_line_mask.png"),0);mask[0,0]=255;cv2.imwrite(str(root/"white_line_mask.png"),mask)
    with pytest.raises(ValueError):load_manual_sample(root)
def test_metadata_required_fields(tmp_path):
    path=tmp_path/"metadata.yaml";path.write_text("width: 640\n")
    with pytest.raises(ValueError):load_metadata(path)
def test_metadata_keeps_actual_k_d(tmp_path):
    path=tmp_path/"metadata.yaml";path.write_text(yaml.safe_dump(metadata()));loaded=load_metadata(path);assert loaded["camera_info"]["k"]==K and loaded["camera_info"]["d"]==D
def test_metadata_original_coordinates_required(tmp_path):
    value=metadata();value["width"]=320;path=tmp_path/"metadata.yaml";path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError):load_metadata(path)
def test_straight_manual_path(tmp_path):
    report=evaluate_manual_sample(write_sample(tmp_path/"sample"));assert report["path_valid"] and report["path_mode"]==1 and report["pose_count"]>0
@pytest.mark.parametrize("scenario,sign",[("CURVE_LEFT_BOTH",1),("CURVE_RIGHT_BOTH",-1)])
def test_curve_steering_sign(tmp_path,scenario,sign):
    report=evaluate_manual_sample(write_sample(tmp_path/"sample",scenario));assert report["target_steering_deg"]*sign>0
@pytest.mark.parametrize("scenario,mode",[("LEFT_BOUNDARY_ONLY",2),("RIGHT_BOUNDARY_ONLY",3),("ROAD_ONLY",4)])
def test_degraded_modes(tmp_path,scenario,mode):assert evaluate_manual_sample(write_sample(tmp_path/"sample",scenario))["path_mode"]==mode
def test_debug_outputs_optional_and_written(tmp_path):
    root=write_sample(tmp_path/"sample");debug=tmp_path/"debug";evaluate_manual_sample(root,debug);assert len(list(debug.glob("*.png")))==8
def test_no_debug_by_default(tmp_path):
    root=write_sample(tmp_path/"sample");evaluate_manual_sample(root);assert not (tmp_path/"debug").exists()
def test_report_metrics_finite(tmp_path):
    report=evaluate_manual_sample(write_sample(tmp_path/"sample"));assert report["finite"] and report["path_x_min"]>0 and np.isfinite(report["max_curvature"])

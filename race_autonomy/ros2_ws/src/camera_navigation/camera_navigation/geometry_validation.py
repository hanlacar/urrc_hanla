"""Pure Stage 1B CameraInfo and surveyed-point validation logic."""
from pathlib import Path
import math
import numpy as np
import yaml

from .camera_geometry import CameraGeometry, OPTICAL_TO_FORWARD, rpy_matrix

DEFAULT_TOLERANCES={"near_forward_limit_m":1.0,"near_position_tolerance_m":.15,"far_forward_limit_m":3.0,"far_position_tolerance_m":.25,"lateral_tolerance_m":.15,"pixel_round_trip_tolerance_px":1.0}


def camera_info_dict(msg):
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    roi=msg.roi
    return {"frame_id":msg.header.frame_id,"width":int(msg.width),"height":int(msg.height),"distortion_model":msg.distortion_model,"d":list(msg.d),"k":list(msg.k),"r":list(msg.r),"p":list(msg.p),"binning_x":int(msg.binning_x),"binning_y":int(msg.binning_y),"roi":{"x_offset":int(roi.x_offset),"y_offset":int(roi.y_offset),"width":int(roi.width),"height":int(roi.height),"do_rectify":bool(roi.do_rectify)},"timestamp":stamp}


def validate_camera_info(info, expected_width=640, expected_height=480, initial_frame_id=None):
    try:
        width,height=int(info["width"]),int(info["height"]); k=np.asarray(info["k"],float).reshape(3,3)
        arrays=np.r_[np.asarray(info.get("d",[]),float),k.ravel(),np.asarray(info.get("r",[]),float),np.asarray(info.get("p",[]),float)]
    except (KeyError,TypeError,ValueError):return False,"malformed_camera_info"
    if not info.get("frame_id"):return False,"empty_frame_id"
    if initial_frame_id is not None and info["frame_id"]!=initial_frame_id:return False,"frame_id_changed"
    if (width,height)!=(expected_width,expected_height):return False,"resolution_mismatch"
    if not np.isfinite(arrays).all():return False,"nonfinite_camera_info"
    if info.get("distortion_model")!="plumb_bob":return False,"unsupported_distortion_model"
    if np.asarray(info.get("d",[])).shape!=(5,):return False,"invalid_distortion_length"
    if np.asarray(info.get("p",[])).shape!=(12,):return False,"invalid_projection_matrix"
    if k[0,0]<=0 or k[1,1]<=0:return False,"invalid_focal_length"
    if not (0<=k[0,2]<width and 0<=k[1,2]<height):return False,"principal_point_outside_image"
    r=np.asarray(info.get("r",[]),float)
    if r.size!=9 or not np.allclose(r.reshape(3,3),np.eye(3),atol=1e-8):return False,"nonidentity_rectification"
    if int(info.get("binning_x",0))!=0 or int(info.get("binning_y",0))!=0:return False,"binning_not_supported"
    roi=info.get("roi",{});roi_width=int(roi.get("width",0));roi_height=int(roi.get("height",0))
    if int(roi.get("x_offset",0))!=0 or int(roi.get("y_offset",0))!=0 or roi_width not in (0,width) or roi_height not in (0,height):return False,"cropped_roi_not_supported"
    return True,"ok"


def pitch_diagnostics(pitches=(-5.,-10.,-15.)):
    """Active rotations: R acts on optical vectors to express them in base."""
    output=[]
    for pitch in pitches:
        axis=rpy_matrix(0.,pitch,0.)@OPTICAL_TO_FORWARD@np.array([0.,0.,1.])
        output.append({"pitch_deg":float(pitch),"optical_forward_axis_base":[float(v) for v in axis],"base_z":float(axis[2])})
    consistent=all(output[i+1]["base_z"]<output[i]["base_z"] for i in range(len(output)-1))
    return {"convention":"active vector rotation; v_base = R_mount @ R_optical_to_base @ v_optical","negative_pitch_points_down":bool(consistent and output[1]["base_z"]<0),"samples":output}


def pitch_projection_diagnostics(k, pixel=(320.,400.), camera_height=.85, pitches=(-5.,-10.,-15.), distortion_coeffs=None, distortion_model="plumb_bob"):
    samples=[]
    for pitch in pitches:
        point=CameraGeometry(k,(0.,0.,camera_height),(0.,pitch,0.),distortion_coeffs=distortion_coeffs,distortion_model=distortion_model).pixel_to_ground(*pixel)
        samples.append({"pitch_deg":float(pitch),"pixel":[float(pixel[0]),float(pixel[1])],"ground_x_m":None if point is None else float(point[0])})
    distances=[sample["ground_x_m"] for sample in samples]
    consistent=all(value is not None and np.isfinite(value) for value in distances) and distances[0]>distances[1]>distances[2]>0
    return {"more_negative_pitch_estimates_nearer_ground_for_same_lower_pixel":bool(consistent),"samples":samples}


def validate_reference(reference, info):
    if int(reference.get("image_width",-1))!=int(info["width"]) or int(reference.get("image_height",-1))!=int(info["height"]):return False,"reference_resolution_mismatch"
    frame=reference.get("frame_id","")
    if frame and frame!=info["frame_id"]:return False,"reference_frame_mismatch"
    return True,"ok"


def _empty_point(point, reason):
    return {"name":point.get("name","unnamed"),"measured":False,"valid":False,"invalid_reason":reason}


def evaluate_reference_points(info, reference, camera_xyz=(0.,0.,.845), mount_rpy=(0.,-10.,0.), tolerances=None):
    tolerances={**DEFAULT_TOLERANCES,**(tolerances or {})}; geometry=CameraGeometry(info["k"],camera_xyz,mount_rpy,distortion_coeffs=info.get("d"),distortion_model=info.get("distortion_model",""))
    results=[]; direction_failure=False; exceeded=0; measured=0
    for point in reference.get("points",[]):
        if point.get("pixel_u") is None or point.get("pixel_v") is None:
            results.append(_empty_point(point,"unmeasured_null_pixel"));continue
        measured+=1; u,v=float(point["pixel_u"]),float(point["pixel_v"]); expected=np.array([float(point["expected_x_m"]),float(point["expected_y_m"])])
        if expected[0] < 0 or expected[0] > tolerances["far_forward_limit_m"]:
            results.append(_empty_point(point,"expected_point_outside_validation_range"));direction_failure=True;continue
        predicted=geometry.pixel_to_ground(u,v)
        if predicted is None or not np.isfinite(predicted).all():
            results.append(_empty_point(point,"invalid_or_nonfinite_projection"));direction_failure=True;continue
        projected=geometry.ground_to_pixel(predicted[0],predicted[1]); error=predicted[:2]-expected; planar=float(np.linalg.norm(error)); roundtrip=float(np.linalg.norm(projected-[u,v]))
        forward_tol=tolerances["near_position_tolerance_m"] if expected[0]<=tolerances["near_forward_limit_m"] else tolerances["far_position_tolerance_m"]
        within=abs(error[0])<=forward_tol and abs(error[1])<=tolerances["lateral_tolerance_m"] and roundtrip<=tolerances["pixel_round_trip_tolerance_px"]
        sign_ok=expected[1]==0 or predicted[1]*expected[1]>0
        direction_ok=predicted[0]>0 and sign_ok; direction_failure|=not direction_ok; exceeded+=not within
        results.append({"name":point.get("name","unnamed"),"measured":True,"valid":bool(within and direction_ok),"invalid_reason":"" if within and direction_ok else ("direction_or_lateral_sign" if not direction_ok else "tolerance_exceeded"),"predicted_x_m":float(predicted[0]),"predicted_y_m":float(predicted[1]),"error_x_m":float(error[0]),"error_y_m":float(error[1]),"planar_error_m":planar,"projected_u":float(projected[0]),"projected_v":float(projected[1]),"pixel_round_trip_error_px":roundtrip})
    if measured==0:status="NOT_RUN"
    elif direction_failure or exceeded>measured/2:status="FAIL"
    elif exceeded:status="PARTIAL"
    else:status="PASS"
    return {"status":status,"measured_points":measured,"total_reference_points":len(reference.get("points",[])),"tolerances":tolerances,"points":results}


def validate_output_path(path):
    resolved=Path(path).expanduser().resolve(); parts=resolved.parts
    if "src" in parts or "install" in parts or "build" in parts:raise ValueError("validation output must not be stored in source/build/install trees")
    return resolved


def save_report(report,path):
    destination=validate_output_path(path);destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open("w",encoding="utf-8") as stream:yaml.safe_dump(report,stream,sort_keys=False,allow_unicode=True)
    return destination

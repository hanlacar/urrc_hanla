"""Stage 1C manual RGB/mask sample validation and offline path evaluation."""
from pathlib import Path
import cv2
import numpy as np
import yaml

from .bev_transform import BevTransform
from .boundary_extractor import extract_boundary
from .camera_geometry import CameraGeometry
from .path_generator import INVALID, generate_path
from .path_validator import validate_path
from .geometry_validation import validate_output_path
from .pure_pursuit import steering_angle_deg
from .speed_planner import max_curvature, target_speed

MASK_FILES={"road":"road_mask.png","white":"white_line_mask.png","yellow":"yellow_line_mask.png"}
REQUIRED_SIZE=(640,480)


def playback_contract(metadata,masks,timestamp):
    """ROS-independent description used to build synchronized Image messages."""
    return {key:{"encoding":"mono8","width":int(mask.shape[1]),"height":int(mask.shape[0]),"frame_id":metadata["frame_id"],"timestamp":timestamp} for key,mask in masks.items()}


def load_metadata(path):
    with Path(path).open(encoding="utf-8") as stream:data=yaml.safe_load(stream) or {}
    required=("image_topic","timestamp","frame_id","width","height","encoding","camera_info","camera_extrinsic")
    missing=[key for key in required if key not in data]
    if missing:raise ValueError(f"metadata missing: {', '.join(missing)}")
    if not data["frame_id"] or not np.isfinite(float(data["timestamp"])):raise ValueError("invalid metadata frame_id/timestamp")
    if (int(data["width"]),int(data["height"]))!=REQUIRED_SIZE:raise ValueError("metadata must describe original 640x480 image")
    camera=data["camera_info"];k=np.asarray(camera.get("k",[]),float);d=np.asarray(camera.get("d",[]),float)
    if k.shape!=(9,) or d.shape!=(5,) or not np.isfinite(np.r_[k,d]).all():raise ValueError("metadata CameraInfo K/D invalid")
    if camera.get("distortion_model")!="plumb_bob":raise ValueError("only plumb_bob metadata is supported")
    return data


def load_manual_sample(sample_directory):
    root=Path(sample_directory).expanduser();metadata=load_metadata(root/"metadata.yaml")
    image=cv2.imread(str(root/"image_raw.png"),cv2.IMREAD_COLOR)
    if image is None:raise ValueError("image_raw.png missing or unreadable")
    if image.shape!=(480,640,3):raise ValueError("RGB image must retain original 640x480 coordinates")
    masks={}
    for key,name in MASK_FILES.items():
        mask=cv2.imread(str(root/name),cv2.IMREAD_UNCHANGED)
        if mask is None:raise ValueError(f"{name} missing or unreadable")
        if mask.shape!=(480,640) or mask.dtype!=np.uint8:raise ValueError(f"{name} must be 640x480 mono8")
        values=np.unique(mask)
        if not np.isin(values,[0,255]).all():raise ValueError(f"{name} must contain only 0 or 255")
        masks[key]=mask
    if np.count_nonzero(masks["road"])==0:raise ValueError("road_mask.png must contain a nonempty road region")
    # Boundary strokes may straddle the road polygon edge; allow a 7 px annotation margin.
    allowed=cv2.dilate(masks["road"],np.ones((15,15),np.uint8))>0
    for key in ("white","yellow"):
        if np.any((masks[key]>0)&~allowed):raise ValueError(f"{key} lane pixels must lie on or next to road mask")
    return metadata,image,masks


def evaluate_manual_sample(sample_directory,debug_directory=None):
    metadata,image,masks=load_manual_sample(sample_directory);camera=metadata["camera_info"];extrinsic=metadata["camera_extrinsic"]
    geometry=CameraGeometry(camera["k"],(extrinsic["camera_x_m"],extrinsic["camera_y_m"],extrinsic["camera_z_m"]),(extrinsic["camera_mount_roll_deg"],extrinsic["camera_mount_pitch_deg"],extrinsic["camera_mount_yaw_deg"]),distortion_coeffs=camera["d"],distortion_model=camera["distortion_model"])
    bev=BevTransform(geometry,.3,8.,2.,2.,.02);road,white,yellow=(bev.warp(masks[key]) for key in ("road","white","yellow"))
    if any(value is None for value in (road,white,yellow)):raise ValueError("singular or invalid BEV transform")
    left,right=extract_boundary(yellow,bev),extract_boundary(white,bev);rows,cols=np.nonzero(road>0);road_points=np.empty((0,2))
    if len(rows):
        unique=np.unique(rows);road_points=np.array([bev.bev_to_ground(float(np.median(cols[rows==row])),row) for row in unique])
    path,mode=generate_path(left,right,road_points,.8);confidence={1:1.,2:.65,3:.65,4:.5}.get(mode,0.);valid=validate_path(path,confidence,max_forward_m=8.)
    speed=target_speed(path,mode,valid=valid);steering=steering_angle_deg(path,speed,.3,.5,2.,.5,25.) if valid else 0.
    outside=0
    for x,y in path:
        col,row=bev.ground_to_bev(x,y);ri,ci=int(round(row)),int(round(col));outside+=not (0<=ri<road.shape[0] and 0<=ci<road.shape[1] and road[ri,ci]>0)
    finite=bool(np.isfinite(path).all() and np.isfinite([speed,steering,confidence]).all())
    report={"status":"PASS" if valid and finite and outside==0 else "FAIL","failure_reason":"" if valid and finite and outside==0 else ("path_outside_road" if outside else "invalid_or_nonfinite_path"),"sample":str(Path(sample_directory).expanduser()),"frame_id":metadata["frame_id"],"timestamp":float(metadata["timestamp"]),"path_valid":valid,"path_mode":int(mode),"path_confidence":float(confidence),"pose_count":int(len(path)),"path_x_min":None if not len(path) else float(path[:,0].min()),"path_x_max":None if not len(path) else float(path[:,0].max()),"path_y_min":None if not len(path) else float(path[:,1].min()),"path_y_max":None if not len(path) else float(path[:,1].max()),"max_curvature":float(max_curvature(path)),"target_speed_mps":float(speed),"target_steering_deg":float(steering),"finite":finite,"road_mask_outside_path_points":int(outside),"valid_pixel_count":{key:int(np.count_nonzero(value)) for key,value in masks.items()}}
    if debug_directory is not None:write_debug_images(debug_directory,image,masks,road,white,yellow,path,bev,report)
    return report


def write_debug_images(directory,image,masks,road,white,yellow,path,bev,report):
    target=validate_output_path(directory);target.mkdir(parents=True,exist_ok=True)
    overlay=image.copy();overlay[masks["road"]>0]=(overlay[masks["road"]>0]*.55+np.array([0,100,0])).clip(0,255);overlay[masks["white"]>0]=(255,255,255);overlay[masks["yellow"]>0]=(0,255,255)
    cv2.imwrite(str(target/"01_raw_mask_overlay.png"),overlay)
    map_x,map_y=cv2.initUndistortRectifyMap(bev.geometry.k,bev.geometry.distortion,None,bev.geometry.k,(image.shape[1],image.shape[0]),cv2.CV_32FC1)
    undistorted={key:cv2.remap(value,map_x,map_y,cv2.INTER_NEAREST) for key,value in masks.items()}
    cv2.imwrite(str(target/"02_undistorted_road.png"),undistorted["road"]);cv2.imwrite(str(target/"03_undistorted_white.png"),undistorted["white"]);cv2.imwrite(str(target/"04_undistorted_yellow.png"),undistorted["yellow"])
    cv2.imwrite(str(target/"05_bev_road.png"),road);cv2.imwrite(str(target/"06_bev_white.png"),white);cv2.imwrite(str(target/"07_bev_yellow.png"),yellow)
    canvas=cv2.cvtColor(road,cv2.COLOR_GRAY2BGR)
    for x,y in path:
        col,row=bev.ground_to_bev(x,y);cv2.circle(canvas,(int(round(col)),int(round(row))),2,(0,0,255),-1)
    if len(path):
        lookahead=.5;target_point=path[np.argmin(np.abs(np.linalg.norm(path,axis=1)-lookahead))];col,row=bev.ground_to_bev(*target_point);cv2.circle(canvas,(int(round(col)),int(round(row))),6,(255,0,255),2)
    cv2.putText(canvas,f"mode={report['path_mode']} conf={report['path_confidence']:.2f}",(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,0),1)
    cv2.putText(canvas,f"speed={report['target_speed_mps']:.2f} steer={report['target_steering_deg']:.2f}",(8,40),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,0),1)
    cv2.imwrite(str(target/"08_bev_boundaries_path_control.png"),canvas)

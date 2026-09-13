"""One-shot ROS CLI for physical D456 geometry validation."""
import argparse
from pathlib import Path
import time
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from .geometry_validation import camera_info_dict,evaluate_reference_points,pitch_diagnostics,pitch_projection_diagnostics,save_report,validate_camera_info,validate_reference


class CameraInfoCollector(Node):
    def __init__(self):
        super().__init__("camera_geometry_validation");self.messages=[]
        self.create_subscription(CameraInfo,"/camera/camera_info",self.messages.append,10)


def main(args=None):
    parser=argparse.ArgumentParser(description="Validate D456 CameraInfo and surveyed pixel/ground points")
    default_reference=Path(get_package_share_directory("camera_navigation"))/"config"/"camera_geometry_reference.yaml"
    parser.add_argument("--reference",default=str(default_reference));parser.add_argument("--output",default="~/.config/camera_navigation/camera_geometry_validation.yaml");parser.add_argument("--timeout",type=float,default=5.);parser.add_argument("--observe",type=float,default=1.)
    cli=parser.parse_args(rclpy.utilities.remove_ros_args(args=args)[1:]);rclpy.init(args=args);node=CameraInfoCollector();deadline=time.monotonic()+cli.timeout
    while rclpy.ok() and not node.messages and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not node.messages:
        node.destroy_node();rclpy.shutdown();raise RuntimeError("no /camera/camera_info received before timeout")
    observe_deadline=time.monotonic()+cli.observe
    while rclpy.ok() and time.monotonic()<observe_deadline:rclpy.spin_once(node,timeout_sec=.05)
    infos=[camera_info_dict(msg) for msg in node.messages]; first=infos[0]; camera_ok,reason=validate_camera_info(first)
    if camera_ok:
        for item in infos[1:]:
            camera_ok,reason=validate_camera_info(item,initial_frame_id=first["frame_id"])
            if not camera_ok:break
    with Path(cli.reference).open(encoding="utf-8") as stream:reference_document=yaml.safe_load(stream) or {}
    pitch_pixel=(first["k"][2],min(first["height"]-1.,first["k"][5]+160.))
    reference=reference_document.get("camera_geometry_reference",{}); report={"status":"FAIL","camera_info_valid":camera_ok,"camera_info_reason":reason,"camera_info":first,"camera_info_message_count":len(infos),"estimated_hz":None,"pitch_diagnostics":pitch_diagnostics(),"pitch_projection_diagnostics":pitch_projection_diagnostics(first["k"],pixel=pitch_pixel,distortion_coeffs=first["d"],distortion_model=first["distortion_model"])}
    stamps=[item["timestamp"] for item in infos]
    if len(stamps)>1 and stamps[-1]>stamps[0]:report["estimated_hz"]=(len(stamps)-1)/(stamps[-1]-stamps[0])
    reference_ok,reference_reason=validate_reference(reference,first)
    if not reference_ok:camera_ok=False;report["camera_info_valid"]=False;report["camera_info_reason"]=reference_reason
    if camera_ok:
        report.update(evaluate_reference_points(first,reference));report["camera_info_valid"]=True;report["camera_info_reason"]="ok"
    destination=save_report(report,cli.output);node.get_logger().info(f"geometry validation {report['status']}: {destination}")
    node.destroy_node();rclpy.shutdown()

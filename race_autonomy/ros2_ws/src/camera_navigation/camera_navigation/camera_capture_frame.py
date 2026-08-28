"""One-shot lossless D456 raw-frame and CameraInfo capture utility."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import cv2
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo,Image
from .geometry_validation import camera_info_dict,validate_output_path

class Capture(Node):
    def __init__(self):
        super().__init__("camera_capture_frame");self.image=None;self.info=None;self.bridge=CvBridge()
        self.create_subscription(Image,"/camera/image_raw",lambda msg:setattr(self,"image",msg),10);self.create_subscription(CameraInfo,"/camera/camera_info",lambda msg:setattr(self,"info",msg),10)

def main(args=None):
    parser=argparse.ArgumentParser();parser.add_argument("--output",default="~/.config/camera_navigation/manual_samples/sample_0001");parser.add_argument("--timeout",type=float,default=10.)
    cli=parser.parse_args(rclpy.utilities.remove_ros_args(args=args)[1:]);output=validate_output_path(cli.output);rclpy.init(args=args);node=Capture();deadline=node.get_clock().now().nanoseconds*1e-9+cli.timeout
    while rclpy.ok() and (node.image is None or node.info is None) and node.get_clock().now().nanoseconds*1e-9<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if node.image is None or node.info is None:raise RuntimeError("image_raw and matching CameraInfo were not received")
    msg=node.image
    if (msg.width,msg.height)!=(640,480):raise ValueError("capture requires original 640x480 image")
    if msg.encoding.lower() not in ("rgb8","bgr8"):raise ValueError(f"unsupported raw RGB encoding: {msg.encoding}")
    info_stamp=node.info.header.stamp.sec+node.info.header.stamp.nanosec*1e-9;image_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if node.info.header.frame_id!=msg.header.frame_id or abs(info_stamp-image_stamp)>.05:raise ValueError("Image and CameraInfo frame/timestamp do not correspond")
    bgr=node.bridge.imgmsg_to_cv2(msg,"bgr8");output.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(output/"image_raw.png"),bgr):raise RuntimeError("PNG write failed")
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;metadata={"image_topic":"/camera/image_raw","timestamp":stamp,"frame_id":msg.header.frame_id,"width":msg.width,"height":msg.height,"encoding":msg.encoding,"png_color_contract":"lossless PNG; OpenCV BGR storage representing original RGB colors","camera_info":camera_info_dict(node.info),"camera_extrinsic":{"camera_x_m":.38,"camera_y_m":0.,"camera_z_m":.96,"camera_mount_roll_deg":0.,"camera_mount_pitch_deg":-5.,"camera_mount_yaw_deg":0.},"capture_time_utc":datetime.now(timezone.utc).isoformat()}
    with (output/"metadata.yaml").open("w",encoding="utf-8") as stream:yaml.safe_dump(metadata,stream,sort_keys=False)
    node.get_logger().info(f"captured lossless sample frame: {output}");node.destroy_node();rclpy.shutdown()

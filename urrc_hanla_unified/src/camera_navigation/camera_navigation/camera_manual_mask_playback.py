"""Temporary external-mask ROS publisher for one validated manual sample."""
import argparse
import time
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from .manual_sample import load_manual_sample

def build_mask_messages(bridge,metadata,masks,stamp):
    messages={}
    for key in ("road","white","yellow"):
        message=bridge.cv2_to_imgmsg(masks[key],"mono8");message.header.stamp=stamp;message.header.frame_id=metadata["frame_id"];messages[key]=message
    return messages

class ManualPlayback(Node):
    def __init__(self,metadata,masks):
        super().__init__("camera_manual_mask_playback");self.metadata=metadata;self.masks=masks;self.bridge=CvBridge()
        self.publishers={key:self.create_publisher(Image,f"/camera/{topic}",10) for key,topic in (("road","road_mask"),("white","white_line_mask"),("yellow","yellow_line_mask"))};self.valid=self.create_publisher(Bool,"/camera/perception_valid",10)
    def publish_frame(self):
        stamp=self.get_clock().now().to_msg()
        messages=build_mask_messages(self.bridge,self.metadata,self.masks,stamp)
        for key,publisher in self.publishers.items():publisher.publish(messages[key])
        self.valid.publish(Bool(data=True))

def main(args=None):
    parser=argparse.ArgumentParser();parser.add_argument("--sample",required=True);parser.add_argument("--duration",type=float,default=5.);parser.add_argument("--rate",type=float,default=10.);parser.add_argument("--once",action="store_true")
    cli=parser.parse_args(rclpy.utilities.remove_ros_args(args=args)[1:]);metadata,_,masks=load_manual_sample(cli.sample);rclpy.init(args=args);node=ManualPlayback(metadata,masks)
    if cli.once:node.publish_frame();rclpy.spin_once(node,timeout_sec=.1)
    else:
        deadline=time.monotonic()+cli.duration;period=1./cli.rate
        while rclpy.ok() and time.monotonic()<deadline:node.publish_frame();rclpy.spin_once(node,timeout_sec=period)
    node.destroy_node();rclpy.shutdown()

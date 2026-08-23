#!/usr/bin/env python3
"""Publish a video file as a ROS camera stream for repeatable inference tests."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher_node")
        defaults = {"video_path":"/home/parkjinwoo/Downloads/asdf.mp4", "width":640,
                    "height":480, "fps":15.0, "loop":True,
                    "frame_id":"video_camera_optical_frame"}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        self.cap = cv2.VideoCapture(str(self.p("video_path")))
        if not self.cap.isOpened(): raise RuntimeError(f"cannot open video: {self.p('video_path')}")
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.create_timer(1.0/max(1.0,float(self.p("fps"))), self.tick)

    def p(self, name): return self.get_parameter(name).value

    def tick(self):
        ok, frame = self.cap.read()
        if not ok and bool(self.p("loop")):
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0); ok, frame = self.cap.read()
        if not ok: return
        width, height = int(self.p("width")), int(self.p("height"))
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        stamp = self.get_clock().now().to_msg(); frame_id = str(self.p("frame_id"))
        image = Image(); image.header.stamp=stamp; image.header.frame_id=frame_id
        image.height=height; image.width=width; image.encoding="bgr8"; image.step=width*3
        image.data=frame.tobytes(); self.image_pub.publish(image)
        info=CameraInfo(); info.header=image.header; info.height=height; info.width=width
        info.distortion_model="plumb_bob"; info.d=[0.0]*5
        focal=float(width); info.k=[focal,0.0,width/2.0,0.0,focal,height/2.0,0.0,0.0,1.0]
        info.r=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        info.p=[focal,0.0,width/2.0,0.0,0.0,focal,height/2.0,0.0,0.0,0.0,1.0,0.0]
        self.info_pub.publish(info)

    def destroy_node(self):
        self.cap.release(); return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=VideoPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

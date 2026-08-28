#!/usr/bin/env python3
"""Estimate stop-line distance by calibrated monocular ground projection."""
import json, math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, String
from camera_navigation.camera_geometry import CameraGeometry

def stop_box_distance_m(box, geometry, camera_x_m, vertical_ratio=0.5):
    if box is None or len(box)!=4:return None
    x1,y1,x2,y2=(float(v) for v in box)
    if not all(math.isfinite(v) for v in (x1,y1,x2,y2)) or x2<=x1 or y2<=y1:return None
    ratio=min(1.,max(0.,float(vertical_ratio)))
    point=geometry.pixel_to_ground((x1+x2)*.5,y1+(y2-y1)*ratio)
    if point is None:return None
    distance=float(point[0])-float(camera_x_m)
    return distance if math.isfinite(distance) and distance>0. else None

class StopLineGroundNode(Node):
    def __init__(self):
        super().__init__("stop_line_ground_node")
        defaults={"max_detection_age_sec":.35,"camera_x_m":.245,"camera_y_m":0.,"camera_z_m":.85,"camera_mount_roll_deg":0.,"camera_mount_pitch_deg":-5.,"camera_mount_yaw_deg":0.,"box_vertical_ratio":.5,"maximum_distance_m":30.}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.info=None;self.box=None;self.box_time=None
        self.distance_pub=self.create_publisher(Float32,"/perception/stop_line_distance_m",10);self.valid_pub=self.create_publisher(Bool,"/perception/stop_line_distance_valid",10)
        self.status_pub=self.create_publisher(String,"/perception/stop_line_ground_status",10);self.debug_pub=self.create_publisher(String,"/perception/stop_line_ground_debug_json",10)
        self.create_subscription(CameraInfo,"/camera/camera_info",lambda msg:setattr(self,"info",msg),10);self.create_subscription(String,"/perception/detections_json",self.on_detections,10);self.create_timer(.05,self.publish_distance)
    def p(self,name):return self.get_parameter(name).value
    def on_detections(self,msg):
        try:
            items=json.loads(msg.data).get("detections",[]);stops=[i for i in items if str(i.get("class_name","")).lower().replace("_","") in {"stop","stopline"} and len(i.get("xyxy",[]))==4]
            self.box=max(stops,key=lambda i:float(i.get("confidence",0.)))["xyxy"] if stops else None;self.box_time=time.monotonic()
        except (ValueError,TypeError,KeyError,json.JSONDecodeError):self.box=None;self.box_time=time.monotonic()
    def publish_distance(self):
        distance=None;status="no_camera_info" if self.info is None else "no_fresh_stop_detection"
        if self.info is not None and self.box is not None and self.box_time is not None and time.monotonic()-self.box_time<=float(self.p("max_detection_age_sec")):
            try:
                g=CameraGeometry(self.info.k,(self.p("camera_x_m"),self.p("camera_y_m"),self.p("camera_z_m")),(self.p("camera_mount_roll_deg"),self.p("camera_mount_pitch_deg"),self.p("camera_mount_yaw_deg")),max_distance_m=self.p("maximum_distance_m"),distortion_coeffs=self.info.d,distortion_model=self.info.distortion_model or "plumb_bob")
                distance=stop_box_distance_m(self.box,g,self.p("camera_x_m"),self.p("box_vertical_ratio"));status="ok" if distance is not None else "ground_projection_invalid"
            except (TypeError,ValueError):status="camera_geometry_invalid"
        valid=distance is not None;self.valid_pub.publish(Bool(data=valid));self.status_pub.publish(String(data=status));self.debug_pub.publish(String(data=json.dumps({"valid":valid,"status":status,"distance_m":distance,"bbox_xyxy":self.box,"method":"calibrated_monocular_ground_projection"})))
        if valid:self.distance_pub.publish(Float32(data=distance))

def main(args=None):
    rclpy.init(args=args);node=StopLineGroundNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=="__main__":main()

"""Section-5 LiDAR local planner using Ackermann trajectory candidates."""

import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

from .ackermann_local_planner import choose_trajectory, pure_pursuit_steering


class AckermannLidarPlannerNode(Node):
    def __init__(self):
        super().__init__("ackermann_lidar_planner")
        defaults={
            "scan_topic":"/scan", "camera_path_topic":"/camera/path",
            "camera_path_valid_topic":"/camera/path_valid",
            "camera_wheel_topic":"/camera_wheel", "drive_mode_topic":"/drive_mode",
            "avoidance_section":"5", "wheelbase_m":.73,
            "vehicle_length_m":1.30, "vehicle_width_m":.77,
            "safety_margin_m":.15, "horizon_m":2.5, "trajectory_step_m":.1,
            "candidate_step_deg":3.0, "maximum_steering_deg":27.0,
            "lookahead_m":1.0, "steering_rate_limit_deg_s":90.0,
            "avoid_distance_m":2.5, "clear_distance_m":3.0,
            "clear_confirm_sec":.8, "hard_stop_distance_m":.45,
            "front_half_angle_deg":18.0, "scan_timeout_sec":.3,
            "camera_timeout_sec":.5, "path_timeout_sec":.5,
            "avoid_drive_stage":1, "publish_hz":20.0,
            "laser_x_m":0.0, "laser_y_m":0.0, "laser_yaw_deg":0.0,
        }
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.scan=None;self.scan_time=None;self.camera_path=[];self.path_time=None
        self.camera_path_valid=False;self.camera_wheel=0;self.camera_time=None
        self.drive_mode="";self.avoiding=False;self.clear_since=None
        self.last_wheel=0.0;self.last_tick=time.monotonic()
        self.path_pub=self.create_publisher(Path,"/avoidance/local_path",10)
        self.wheel_pub=self.create_publisher(Int32,"/lidar_wheel",10)
        self.drive_pub=self.create_publisher(Float32,"/lidar_drive",10)
        self.stop_pub=self.create_publisher(Bool,"/lidar_stop",10)
        self.active_pub=self.create_publisher(Bool,"/avoidance/active",10)
        self.state_pub=self.create_publisher(String,"/avoidance/state",10)
        self.create_subscription(LaserScan,self.p("scan_topic"),self.on_scan,10)
        self.create_subscription(Path,self.p("camera_path_topic"),self.on_path,10)
        self.create_subscription(Bool,self.p("camera_path_valid_topic"),
            lambda m:setattr(self,"camera_path_valid",bool(m.data)),10)
        self.create_subscription(Int32,self.p("camera_wheel_topic"),self.on_wheel,10)
        self.create_subscription(String,self.p("drive_mode_topic"),
            lambda m:setattr(self,"drive_mode",str(m.data).strip()),10)
        self.create_timer(1.0/max(1.0,float(self.p("publish_hz"))),self.tick)

    def p(self,name):return self.get_parameter(name).value
    def on_scan(self,msg):self.scan=msg;self.scan_time=time.monotonic()
    def on_wheel(self,msg):
        self.camera_wheel=int(msg.data);self.camera_time=time.monotonic()
    def on_path(self,msg):
        self.camera_path=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.path_time=time.monotonic()

    def obstacle_points(self):
        yaw0=math.radians(float(self.p("laser_yaw_deg")))
        c,s=math.cos(yaw0),math.sin(yaw0);points=[]
        for index,distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance) or not self.scan.range_min<=distance<=self.scan.range_max:continue
            angle=self.scan.angle_min+index*self.scan.angle_increment
            lx,ly=distance*math.cos(angle),distance*math.sin(angle)
            points.append((float(self.p("laser_x_m"))+c*lx-s*ly,
                           float(self.p("laser_y_m"))+s*lx+c*ly))
        return np.asarray(points,dtype=float).reshape((-1,2))

    def publish(self,wheel,stage,stop,state,path=None,details=None):
        self.wheel_pub.publish(Int32(data=int(round(wheel))))
        self.drive_pub.publish(Float32(data=float(stage)))
        self.stop_pub.publish(Bool(data=bool(stop)))
        payload={"state":state};payload.update(details or {})
        self.state_pub.publish(String(data=json.dumps(payload,separators=(",",":"))))
        message=Path();message.header.stamp=self.get_clock().now().to_msg();message.header.frame_id="base_link"
        for x,y,*_ in ([] if path is None else path):
            pose=PoseStamped();pose.header=message.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y);pose.pose.orientation.w=1.;message.poses.append(pose)
        self.path_pub.publish(message)

    def limited(self,target,now):
        dt=max(0.0,now-self.last_tick);self.last_tick=now
        change=float(self.p("steering_rate_limit_deg_s"))*dt
        self.last_wheel=max(self.last_wheel-change,min(float(target),self.last_wheel+change))
        return self.last_wheel

    def tick(self):
        now=time.monotonic();active=self.drive_mode==str(self.p("avoidance_section"))
        self.active_pub.publish(Bool(data=active))
        if not active:
            self.avoiding=False;self.clear_since=None
            wheel=self.limited(self.camera_wheel,now)
            return self.publish(wheel,1,False,"INACTIVE_CAMERA_PASS")
        if (self.scan is None or self.scan_time is None or now-self.scan_time>float(self.p("scan_timeout_sec")) or
            self.camera_time is None or now-self.camera_time>float(self.p("camera_timeout_sec")) or
            not self.camera_path_valid or self.path_time is None or now-self.path_time>float(self.p("path_timeout_sec"))):
            return self.publish(self.limited(0.0,now),0,True,"STOP_INPUT_STALE")
        obstacles=self.obstacle_points();front_angle=math.radians(float(self.p("front_half_angle_deg")))
        angles=np.abs(np.arctan2(obstacles[:,1],obstacles[:,0])) if len(obstacles) else np.asarray([])
        front=obstacles[(obstacles[:,0]>0)&(angles<=front_angle)] if len(obstacles) else obstacles
        front_distance=float(np.min(np.hypot(front[:,0],front[:,1]))) if len(front) else math.inf
        if front_distance<=float(self.p("hard_stop_distance_m")):
            return self.publish(self.limited(0.0,now),0,True,"STOP_OBSTACLE_TOO_CLOSE")
        if front_distance<float(self.p("avoid_distance_m")):self.avoiding=True;self.clear_since=None
        elif self.avoiding and front_distance>=float(self.p("clear_distance_m")):
            if self.clear_since is None:self.clear_since=now
            elif now-self.clear_since>=float(self.p("clear_confirm_sec")):self.avoiding=False;self.clear_since=None
        else:self.clear_since=None
        if not self.avoiding:
            return self.publish(self.limited(self.camera_wheel,now),1,False,"CAMERA_FOLLOW")
        maximum=float(self.p("maximum_steering_deg"));step=float(self.p("candidate_step_deg"))
        candidates=np.arange(-maximum,maximum+.5*step,step)
        best,diagnostics=choose_trajectory(candidates,obstacles,self.camera_path,
            float(self.p("wheelbase_m")),float(self.p("horizon_m")),float(self.p("trajectory_step_m")),
            float(self.p("vehicle_length_m")),float(self.p("vehicle_width_m")),float(self.p("safety_margin_m")),self.last_wheel)
        if best is None:
            return self.publish(self.limited(0.0,now),0,True,"STOP_NO_SAFE_TRAJECTORY",details={"candidate_count":len(diagnostics)})
        _,candidate_steer,path,clearance,error=best
        target=pure_pursuit_steering(path,float(self.p("lookahead_m")),float(self.p("wheelbase_m")),maximum)
        wheel=self.limited(target,now)
        return self.publish(wheel,int(self.p("avoid_drive_stage")),False,"ACKERMANN_AVOID",path,
            {"candidate_steering_deg":candidate_steer,"pure_pursuit_steering_deg":target,"limited_steering_deg":wheel,"clearance_m":clearance,"camera_path_error_m":error})


def main(args=None):
    rclpy.init(args=args);node=AckermannLidarPlannerNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

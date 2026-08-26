"""Path to physical target speed and steering; no actuator publishers."""
import time
import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8
from .pure_pursuit import rate_limit, steering_angle_deg
from .speed_planner import target_speed


class CameraPathController(Node):
    def __init__(self):
        super().__init__("camera_path_controller_node"); self.path=np.empty((0,2)); self.valid=False; self.confidence=0.; self.mode=0; self.speed=0.; self.steer=0.; self.last=time.monotonic()
        defaults={"wheelbase_m":.78,"min_lookahead_m":.5,"max_lookahead_m":2.,"lookahead_speed_gain":.5,"max_steering_deg":27.,"steering_rate_limit_deg_s":60.,"normal_speed_mps":.526,"turn_speed_mps":.229,"single_boundary_speed_mps":.526,"road_only_speed_mps":.229,"max_lateral_accel_mps2":1.,"accel_limit_mps2":.5,"decel_limit_mps2":1.}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.create_subscription(Path,"/camera/path",self.on_path,10); self.create_subscription(Bool,"/camera/path_valid",lambda m:setattr(self,"valid",m.data),10); self.create_subscription(Float32,"/camera/path_confidence",lambda m:setattr(self,"confidence",m.data),10); self.create_subscription(Int8,"/camera/path_mode",lambda m:setattr(self,"mode",m.data),10)
        self.speed_pub=self.create_publisher(Float32,"/camera/target_speed_mps",10); self.steer_pub=self.create_publisher(Float32,"/camera/target_steering_deg",10); self.create_timer(.05,self.control)
    def p(self,n):return self.get_parameter(n).value
    def on_path(self,m):self.path=np.array([[p.pose.position.x,p.pose.position.y] for p in m.poses])
    def control(self):
        now=time.monotonic();dt=max(now-self.last,1e-3);self.last=now
        desired=target_speed(self.path,self.mode,self.valid and self.confidence>0.,normal=self.p("normal_speed_mps"),turn=self.p("turn_speed_mps"),single=self.p("single_boundary_speed_mps"),road=self.p("road_only_speed_mps"),max_lateral_accel=self.p("max_lateral_accel_mps2")); limit=self.p("accel_limit_mps2") if desired>self.speed else self.p("decel_limit_mps2");self.speed=0. if desired<=0. else float(np.clip(desired,self.speed-limit*dt,self.speed+limit*dt))
        target=steering_angle_deg(self.path,self.speed,self.p("wheelbase_m"),self.p("min_lookahead_m"),self.p("max_lookahead_m"),self.p("lookahead_speed_gain"),self.p("max_steering_deg")) if desired>0 else 0.;self.steer=rate_limit(target,self.steer,self.p("steering_rate_limit_deg_s"),dt) if desired>0 else 0.
        self.speed_pub.publish(Float32(data=self.speed));self.steer_pub.publish(Float32(data=self.steer))

def main():
    rclpy.init(); node=CameraPathController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try:
            if rclpy.ok(): rclpy.shutdown()
        except KeyboardInterrupt: pass

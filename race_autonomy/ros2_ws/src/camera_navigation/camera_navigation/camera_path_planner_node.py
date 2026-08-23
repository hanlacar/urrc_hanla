"""Synchronized camera masks to metric base_link Path ROS adapter."""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Header, Int8, String

from .bev_transform import BevTransform
from .boundary_extractor import extract_boundary
from .drivable_corridor import clearance_corridor_path
from .camera_geometry import CameraGeometry
from .mock_scenario_generator import SCENARIOS, MASK_TOPICS, external_mask_topics, make_masks
from .path_generator import INVALID, LANE_REACQUIRE, OBSTACLE_CORRIDOR, TURN_TEMPLATE, blend_reacquire, generate_path, path_inside_bounds
from .path_validator import MaskMeta, validate_mask_set, validate_path
from .turn_path_generator import TurnState, TurnStateMachine, bezier_turn
from .ros_image import image_to_mono8


class CameraPathPlanner(Node):
    MASK_TOPICS=MASK_TOPICS

    def __init__(self):
        super().__init__("camera_path_planner_node")
        self.info=None; self.frame=None; self.frame_changed=False
        self.masks={}; self.mask_meta={}; self.last_accepted_stamp=None; self.perception=False
        self.imu_valid=False; self.pitch=0.; self.roll=0.; self.yaw=0.; self.turn_direction=0; self.section=1
        self.previous=None; self.previous_time=None
        self.turn_sm=TurnStateMachine(); self.turn_template=None
        defaults={"input_mode":"external","mock_scenario":"STRAIGHT_BOTH","base_frame_id":"base_link","camera_optical_frame_id":"","camera_x_m":.245,"camera_y_m":.004,"camera_z_m":.845,"camera_mount_roll_deg":0.,"camera_mount_pitch_deg":0.,"camera_mount_yaw_deg":0.,"vehicle_width_m":.77,"vehicle_length_m":1.26,"front_overhang_m":.27,"rear_overhang_m":.26,"mask_sync_tolerance_sec":.05,"mask_stale_timeout_sec":.2,"temporal_hold_timeout_sec":.3,"turn_progress_timeout_sec":3.,"min_turn_radius_m":1.2,"bev_forward_min_m":.3,"bev_forward_max_m":8.,"bev_left_m":2.,"bev_right_m":2.,"bev_resolution_m_per_pixel":.02,"lane_width_m":.8,"yellow_path_corridor_m":.25,"obstacle_safety_margin_m":.15}
        for key,value in defaults.items(): self.declare_parameter(key,value)
        self.turn_sm.progress_timeout_s=self.p("turn_progress_timeout_sec")
        self.input_mode=self.p("input_mode")
        external_topics=external_mask_topics(self.input_mode)
        self.create_subscription(CameraInfo,"/camera/camera_info",self.on_info,10)
        if self.input_mode == "external":
            for key,topic in external_topics:
                self.create_subscription(Image,topic,lambda msg,k=key:self.on_mask(k,msg),qos_profile_sensor_data)
            self.create_subscription(Bool,"/camera/perception_valid",lambda msg:setattr(self,"perception",msg.data),10)
        else:
            synthetic=CameraInfo(); synthetic.header.frame_id="mock_rgb_optical_frame"; synthetic.width=640; synthetic.height=480; synthetic.distortion_model="plumb_bob"; synthetic.d=[0.]*5; synthetic.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]
            self.on_info(synthetic); self.perception=True
        self.create_subscription(Bool,"/imu_valid",lambda msg:setattr(self,"imu_valid",msg.data),10)
        self.create_subscription(Float32,"/imu_pitch",lambda msg:setattr(self,"pitch",msg.data),10)
        self.create_subscription(Float32,"/imu_roll",lambda msg:setattr(self,"roll",msg.data),10)
        self.create_subscription(Float32,"/imu_yaw",lambda msg:setattr(self,"yaw",msg.data),10)
        self.create_subscription(Int8,"/mission/turn_direction",lambda msg:setattr(self,"turn_direction",msg.data),10)
        self.create_subscription(Int8,"/mission/active_section",lambda msg:setattr(self,"section",msg.data),10)
        self.path_pub=self.create_publisher(Path,"/camera/path",10); self.valid_pub=self.create_publisher(Bool,"/camera/path_valid",10)
        self.conf_pub=self.create_publisher(Float32,"/camera/path_confidence",10); self.mode_pub=self.create_publisher(Int8,"/camera/path_mode",10)
        self.status_pub=self.create_publisher(String,"/camera/path_status",10)
        self.yellow_ahead_pub=self.create_publisher(Float32,"/camera/yellow_line_ahead_m",10)
        self.yellow_ahead_valid_pub=self.create_publisher(Bool,"/camera/yellow_line_ahead_valid",10)
        self.create_timer(.05,self.tick)

    def p(self,name): return self.get_parameter(name).value
    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def on_info(self,msg):
        incoming=self.p("camera_optical_frame_id") or msg.header.frame_id
        if self.frame is not None and incoming != self.frame: self.frame_changed=True
        self.frame=incoming; self.info=msg

    def on_mask(self,key,msg):
        self.masks[key]=image_to_mono8(msg)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.mask_meta[key]=MaskMeta(stamp,msg.header.frame_id,msg.width,msg.height,msg.encoding)

    def _make_mock(self):
        scenario=self.p("mock_scenario")
        if scenario not in SCENARIOS: raise ValueError(f"invalid mock_scenario: {scenario}")
        if scenario == "STALE_INPUT": return False
        road,white,yellow=make_masks(scenario,self.info.width,self.info.height)
        self.masks=dict(zip(("road","white","yellow"),(road,white,yellow)))
        stamp=self.now(); meta=MaskMeta(stamp,self.frame,self.info.width,self.info.height,"mono8")
        self.mask_meta={key:meta for key in self.masks}; return True

    def _header(self): return Header(stamp=self.get_clock().now().to_msg(),frame_id=self.p("base_frame_id"))
    def publish_invalid(self, reason="invalid"):
        self.path_pub.publish(Path(header=self._header())); self.valid_pub.publish(Bool(data=False)); self.conf_pub.publish(Float32(data=0.)); self.mode_pub.publish(Int8(data=INVALID)); self.status_pub.publish(String(data=reason)); self.yellow_ahead_valid_pub.publish(Bool(data=False))

    def tick(self):
        try:
            if self.info is None:return self.publish_invalid("missing_camera_info")
            if self.frame_changed:return self.publish_invalid("camera_frame_changed")
            if not self.perception:return self.publish_invalid("perception_invalid")
            if self.input_mode == "mock" and not self._make_mock(): return self.publish_invalid("mock_input_stale")
            metas=[self.mask_meta[k] for k in ("road","white","yellow")] if len(self.mask_meta)==3 else []
            ok,_=validate_mask_set(metas,(self.info.width,self.info.height),self.now(),self.p("mask_sync_tolerance_sec"),self.p("mask_stale_timeout_sec"),self.last_accepted_stamp)
            if not ok:return self.publish_invalid(f"mask_validation:{_}")
            self.last_accepted_stamp=max(m.stamp for m in metas)
            geometry=CameraGeometry(self.info.k,(self.p("camera_x_m"),self.p("camera_y_m"),self.p("camera_z_m")),(self.p("camera_mount_roll_deg"),self.p("camera_mount_pitch_deg"),self.p("camera_mount_yaw_deg")),distortion_coeffs=self.info.d,distortion_model=self.info.distortion_model or "plumb_bob")
            bev=BevTransform(geometry,self.p("bev_forward_min_m"),self.p("bev_forward_max_m"),self.p("bev_left_m"),self.p("bev_right_m"),self.p("bev_resolution_m_per_pixel"))
            road,white,yellow=(bev.warp(self.masks[key]) for key in ("road","white","yellow"))
            if any(mask is None for mask in (road,white,yellow)):return self.publish_invalid("bev_warp_failed")
            left,right=extract_boundary(yellow,bev),extract_boundary(white,bev)
            rows,cols=np.nonzero(road>0); road_points=np.empty((0,2))
            if len(rows)>20:
                unique=np.unique(rows); road_points=np.array([bev.bev_to_ground(float(np.median(cols[rows==row])),row) for row in unique])
            age=np.inf if self.previous_time is None else self.now()-self.previous_time
            path,mode=generate_path(left,right,road_points,self.p("lane_width_m"),self.previous,age,self.p("temporal_hold_timeout_sec"))
            if self.section == 5:
                corridor = clearance_corridor_path(
                    road, bev, self.p("vehicle_width_m"),
                    self.p("obstacle_safety_margin_m"))
                if len(corridor) >= 2:
                    path,mode=corridor,OBSTACLE_CORRIDOR
                else:
                    return self.publish_invalid("s_curve_no_vehicle_clearance_corridor")
            road_width_ratio=(cols.max()-cols.min()+1)/road.shape[1] if len(cols) else 0.
            intersection=("INTERSECTION" in self.p("mock_scenario")) if self.input_mode=="mock" else road_width_ratio>.7
            lane_visible=len(left)>=2 or len(right)>=2
            state=self.turn_sm.update(self.turn_direction,self.yaw,self.imu_valid,intersection=intersection,exit_visible=lane_visible,lane_visible=lane_visible,now=self.now())
            if state in (TurnState.TURN_ENTER,TurnState.TURN_EXECUTE,TurnState.EXIT_SEARCH):
                candidate=bezier_turn(self.turn_direction,min_turn_radius_m=self.p("min_turn_radius_m"))
                in_bounds=path_inside_bounds(candidate,0.,self.p("bev_forward_max_m"),self.p("bev_left_m"),self.p("bev_right_m"))
                inside_road=[]
                for x,y in candidate:
                    if x < self.p("bev_forward_min_m"):continue
                    col,row=bev.ground_to_bev(x,y); ri,ci=int(round(row)),int(round(col)); inside_road.append(0<=ri<road.shape[0] and 0<=ci<road.shape[1] and road[ri,ci]>0)
                if not in_bounds or (inside_road and not all(inside_road)):
                    self.turn_sm.state=TurnState.ABORT; return self.publish_invalid("turn_path_outside_drivable_area")
                path,mode=candidate,TURN_TEMPLATE; self.turn_template=candidate
            elif state == TurnState.LANE_REACQUIRE and self.turn_template is not None and len(path)>=2:
                path,mode=blend_reacquire(self.turn_template,path,.5),LANE_REACQUIRE
            elif state == TurnState.ABORT:return self.publish_invalid("turn_state_aborted")
            confidence={1:1.,2:.65,3:.65,4:.5,5:.3,OBSTACLE_CORRIDOR:.7}.get(mode,0.); valid=validate_path(path,confidence,max_forward_m=self.p("bev_forward_max_m"))
            if mode==TURN_TEMPLATE:confidence=.7*self.turn_sm.confidence;valid=validate_path(path,confidence,max_forward_m=self.p("bev_forward_max_m"))
            elif mode==LANE_REACQUIRE:confidence=.6;valid=validate_path(path,confidence,max_forward_m=self.p("bev_forward_max_m"))
            if valid and mode != 5:self.previous=path; self.previous_time=self.now()
            yellow_ahead=[]
            if valid and len(left)>=2 and len(path)>=2:
                order=np.argsort(path[:,0]); px,py=path[order,0],path[order,1]
                usable=left[(left[:,0]>=px.min())&(left[:,0]<=px.max())]
                if len(usable):
                    lateral_error=np.abs(usable[:,1]-np.interp(usable[:,0],px,py))
                    yellow_ahead=usable[lateral_error<=self.p("yellow_path_corridor_m"),0]
            yellow_valid=len(yellow_ahead)>0
            if yellow_valid:self.yellow_ahead_pub.publish(Float32(data=float(np.min(yellow_ahead))))
            self.yellow_ahead_valid_pub.publish(Bool(data=yellow_valid))
            message=Path(header=self._header())
            for x,y in path:
                pose=PoseStamped(header=message.header); pose.pose.position.x=float(x); pose.pose.position.y=float(y); pose.pose.orientation.w=1.; message.poses.append(pose)
            self.path_pub.publish(message); self.valid_pub.publish(Bool(data=valid)); self.conf_pub.publish(Float32(data=confidence)); self.mode_pub.publish(Int8(data=mode))
            self.status_pub.publish(String(data="ok" if valid else "generated_path_invalid"))
        except (KeyError,TypeError,ValueError) as error:
            self.publish_invalid(f"planner_exception:{type(error).__name__}:{error}")


def main():
    rclpy.init(); node=CameraPathPlanner()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():rclpy.shutdown()

import time
import json
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy,qos_profile_sensor_data
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo,Image
from std_msgs.msg import Bool,Float32,Int8,String
from camera_navigation.camera_geometry import CameraGeometry
from camera_navigation.bev_roi import lateral_extent_m
from .class_mapper import SemanticClassMapper
from .image_contract import LatestFrameBuffer,validate_image_contract
from .inference_backend import UltralyticsSegmentationBackend
from .inference_diagnostics import LatencyTracker
from .mask_postprocessor import (exclude_navigation_image_bottom,filter_lane_components,has_navigation_mask,
                                 restore_road_surface_words,
                                 validate_output_mask)
from .model_manifest import load_manifest
from .ros_image import bgr8_to_image,image_to_bgr8,mono8_to_image
from .stop_detection import contains_stop

class CameraYoloInferenceNode(Node):
    def __init__(self,backend=None):
        super().__init__("camera_yolo_inference_node");self.frames=LatestFrameBuffer();self.camera_info=None;self.busy=False;self.latencies=LatencyTracker();self.ready=False;self.failure="not_initialized";self.latest_visualization=None;self.last_visualized_stamp=None;self.cached_visualization_msg=None
        self.navigation_path=np.empty((0,2));self.navigation_path_time=None;self.path_valid=False;self.path_confidence=0.;self.path_mode=0;self.target_speed=0.;self.target_steering=0.;self.active_section=1;self.turn_direction=0
        self.input_frame_times=deque(maxlen=240);self.output_frame_times=deque(maxlen=120)
        self.input_callbacks=MutuallyExclusiveCallbackGroup();self.inference_callbacks=MutuallyExclusiveCallbackGroup();self.visualization_callbacks=MutuallyExclusiveCallbackGroup()
        # use_sim_time is declared by rclpy itself and must not be redeclared.
        defaults={"segmentation_model_path":"","class_manifest_path":"","device":"cpu","input_width":640,"input_height":480,"inference_fps":40.0,"detections_image_fps":30.0,"confidence_threshold":.25,"mask_threshold":.5,"road_mask_threshold":.30,"lane_mask_threshold":.45,"road_confidence_threshold":.25,"lane_confidence_threshold":.50,"lane_minimum_component_area":80,"lane_maximum_horizontal_ratio":4.0,"lane_minimum_horizontal_width":80,"navigation_bottom_exclusion_ratio":0.,"max_image_age_sec":.2,"max_inference_latency_ms":40.,"input_image_topic":"/camera/image_raw","input_camera_info_topic":"/camera/camera_info","expected_image_width":640,"expected_image_height":480,"stop_detected_topic":"/perception/stop_detected","stop_confidence_threshold":.25,"traffic20_confidence_threshold":.25,"publish_diagnostics":True,"require_cuda":False,"camera_x_m":.245,"camera_y_m":0.,"camera_z_m":.85,"camera_mount_roll_deg":0.,"camera_mount_pitch_deg":-5.,"camera_mount_yaw_deg":0.,"path_overlay_timeout_sec":.5,"bev_forward_min_m":.3,"bev_forward_max_m":6.,"bev_normal_lateral_m":1.2,"bev_turn_lateral_m":1.5,"bev_s_curve_lateral_m":1.5,"bev_intersection_lateral_m":1.,"min_lookahead_m":.5,"max_lookahead_m":2.,"lookahead_speed_gain":.5}
        for key,value in defaults.items():self.declare_parameter(key,value)
        output_qos=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,depth=1,reliability=QoSReliabilityPolicy.RELIABLE)
        self.mask_pubs={role:self.create_publisher(Image,f"/camera/{topic}",qos_profile_sensor_data) for role,topic in (("road","road_mask"),("white_line","white_line_mask"),("yellow_line","yellow_line_mask"))}
        # Debug video must never back-pressure inference when RQT is slow.
        self.detections_image_pub=self.create_publisher(Image,"/perception/detections_image",qos_profile_sensor_data)
        self.detections_pub=self.create_publisher(String,"/perception/detections_json",output_qos)
        self.stop_pub=self.create_publisher(Bool,self.p("stop_detected_topic"),output_qos)
        self.traffic20_pub=self.create_publisher(Bool,"/perception/traffic20_detected",output_qos)
        self.valid_pub=self.create_publisher(Bool,"/camera/perception_valid",output_qos);self.latency_pub=self.create_publisher(Float32,"/camera/inference_latency_ms",output_qos);self.status_pub=self.create_publisher(String,"/camera/inference_status",output_qos)
        self.input_fps_pub=self.create_publisher(Float32,"/camera/input_fps",output_qos);self.output_fps_pub=self.create_publisher(Float32,"/camera/output_fps",output_qos)
        self.create_subscription(Image,self.p("input_image_topic"),self.on_image,qos_profile_sensor_data,callback_group=self.input_callbacks);self.create_subscription(CameraInfo,self.p("input_camera_info_topic"),lambda msg:setattr(self,"camera_info",msg),qos_profile_sensor_data,callback_group=self.input_callbacks)
        self.create_subscription(Path,"/camera/path",self.on_path,10)
        self.create_subscription(Bool,"/camera/path_valid",lambda m:setattr(self,"path_valid",m.data),10)
        self.create_subscription(Float32,"/camera/path_confidence",lambda m:setattr(self,"path_confidence",m.data),10)
        self.create_subscription(Int8,"/camera/path_mode",lambda m:setattr(self,"path_mode",m.data),10)
        self.create_subscription(Float32,"/camera/target_speed_mps",lambda m:setattr(self,"target_speed",m.data),10)
        self.create_subscription(Float32,"/camera/target_steering_deg",lambda m:setattr(self,"target_steering",m.data),10)
        self.create_subscription(Int8,"/mission/active_section",lambda m:setattr(self,"active_section",int(m.data)),10)
        self.create_subscription(Int8,"/mission/turn_direction",lambda m:setattr(self,"turn_direction",int(m.data)),10)
        try:
            manifest=load_manifest(self.p("class_manifest_path"));self.mapper=SemanticClassMapper(manifest)
            input_shape=(int(self.p("input_height")),int(self.p("input_width")))
            self.backend=backend or UltralyticsSegmentationBackend(self.p("segmentation_model_path"),self.p("device"),input_shape,self.p("confidence_threshold"),self.p("require_cuda"));self.backend.load_model();self.backend.warmup();self.model_names=self.backend.get_model_names();self.mapper.resolve_model_classes(self.model_names)
            self.role_class_ids={role:self.mapper.class_ids_for_role(role) for role in ("road","white_line","yellow_line")}
            self.inference_role_class_ids=dict(self.role_class_ids)
            self.inference_role_class_ids["words"]=self.mapper.class_ids_for_role("words")
            self.navigation_class_ids=set().union(*map(set,self.role_class_ids.values()))
            self.ready=True;self.failure="ok"
        except Exception as error:self.failure=f"initialization_failed: {error}"
        inference_fps=max(1.0,float(self.p("inference_fps")))
        self.inference_period=1.0/inference_fps
        self.last_inference_time=-float("inf")
        self.last_detections_image_time=-float("inf")
        # 5 ms divides the 25 ms / 40 Hz inference period exactly and avoids
        # waking Python 1000 times per second while waiting for a new frame.
        poll_period=min(.005,self.inference_period/4.0)
        self.create_timer(poll_period,self.process_latest,callback_group=self.inference_callbacks)
        self.create_timer(1.,self.publish_health,callback_group=self.input_callbacks)
        visualization_fps=float(self.p("detections_image_fps"))
        if visualization_fps>0.0:
            self.visualization_period=1.0/visualization_fps
            # A timer running exactly at 30 Hz permanently loses a cycle when
            # inference owns the Python thread at its deadline. Poll cheaply
            # and apply the 30 Hz limit ourselves so the next available slot
            # is used instead of waiting another complete display period.
            self.create_timer(min(.005,self.visualization_period/4.0),self.publish_latest_visualization,callback_group=self.visualization_callbacks)
    def p(self,name):return self.get_parameter(name).value
    def on_image(self,image):
        self.input_frame_times.append(time.monotonic());self.frames.push(image)
    def on_path(self,msg):
        self.navigation_path=np.asarray([(p.pose.position.x,p.pose.position.y) for p in msg.poses],dtype=float).reshape((-1,2))
        self.navigation_path_time=time.monotonic()
    @staticmethod
    def measured_fps(frame_times):
        if len(frame_times)<2:return 0.0
        duration=frame_times[-1]-frame_times[0]
        return (len(frame_times)-1)/duration if duration>0.0 else 0.0
    def publish_status(self,text):
        if self.p("publish_diagnostics"):self.status_pub.publish(String(data=text))
    def publish_health(self):
        self.input_fps_pub.publish(Float32(data=float(self.measured_fps(self.input_frame_times))))
        self.output_fps_pub.publish(Float32(data=float(self.measured_fps(self.output_frame_times))))
        if not self.ready:
            self.valid_pub.publish(Bool(data=False));self.stop_pub.publish(Bool(data=False));self.traffic20_pub.publish(Bool(data=False));self.publish_status(self.failure)
    def publish_invalid(self,image,reason):
        self.valid_pub.publish(Bool(data=False));self.stop_pub.publish(Bool(data=False));self.traffic20_pub.publish(Bool(data=False));self.publish_status(reason)
        if image is not None:
            zero=np.zeros((image.height,image.width),np.uint8)
            for publisher in self.mask_pubs.values():
                publisher.publish(mono8_to_image(zero,image.header))
    def render_detections(self,bgr,instances,masks):
        output=bgr.copy();names=self.model_names
        colors=((0,180,0),(255,255,255),(0,220,255),(0,0,255),(0,255,255),(0,255,0),(255,128,0),(255,0,255),(0,128,255),(255,0,0),(128,255,255),(180,180,180))
        role_colors={"road":(0,180,0),"white_line":(255,255,255),"yellow_line":(0,220,255)}
        for role,mask in masks.items():
            selected=mask>0
            if np.any(selected):output[selected]=role_colors[role]
        # Draw the segmentation shape of every non-navigation object over the
        # complete camera frame.  Blend with the source image so small traffic
        # lights and stop markings remain visually identifiable in RQT.
        for instance in instances:
            class_id=int(instance["class_id"]);mask=instance.get("mask")
            if mask is None:continue
            selected=np.asarray(mask)>0
            if not np.any(selected):continue
            color=np.asarray(colors[class_id%len(colors)],dtype=np.float32)
            source=output[selected].astype(np.float32)
            output[selected]=np.clip(source*.45+color*.55,0,255).astype(np.uint8)
        for instance in instances:
            class_id=int(instance["class_id"]);color=colors[class_id%len(colors)];name=names.get(class_id,str(class_id)) if isinstance(names,dict) else names[class_id]
            if class_id in self.navigation_class_ids:
                continue
            box=instance.get("xyxy",());
            if len(box)!=4:continue
            x1,y1,x2,y2=(int(v) for v in box);cv2.rectangle(output,(x1,y1),(x2,y2),color,1)
            cv2.putText(output,f"{name} {instance.get('confidence',0.):.2f}",(max(0,x1),max(20,y1)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        fps_text=f"INPUT {self.measured_fps(self.input_frame_times):.1f} FPS  OUTPUT {self.measured_fps(self.output_frame_times):.1f} FPS"
        cv2.rectangle(output,(5,5),(405,36),(0,0,0),-1)
        cv2.putText(output,fps_text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.58,(0,255,0),2,cv2.LINE_AA)
        self.render_path_overlay(output)
        return output
    def render_path_overlay(self,output):
        info=self.camera_info;path=self.navigation_path.copy()
        fresh=(self.navigation_path_time is not None and time.monotonic()-self.navigation_path_time<=float(self.p("path_overlay_timeout_sec")))
        if info is None:return
        try:
            geometry=CameraGeometry(info.k,(self.p("camera_x_m"),self.p("camera_y_m"),self.p("camera_z_m")),(self.p("camera_mount_roll_deg"),self.p("camera_mount_pitch_deg"),self.p("camera_mount_yaw_deg")),distortion_coeffs=info.d,distortion_model=info.distortion_model or "plumb_bob")
            lateral=lateral_extent_m(self.active_section,self.turn_direction,self.p("bev_normal_lateral_m"),self.p("bev_turn_lateral_m"),self.p("bev_s_curve_lateral_m"),self.p("bev_intersection_lateral_m"))
            def project(points):
                pixels=[]
                for x,y in points:
                    uv=geometry.ground_to_pixel(float(x),float(y))
                    if uv is not None and np.isfinite(uv).all():
                        u,v=(int(round(value)) for value in uv)
                        if -1000<u<output.shape[1]+1000 and -1000<v<output.shape[0]+1000:pixels.append((u,v))
                return pixels
            xs=np.linspace(float(self.p("bev_forward_min_m")),float(self.p("bev_forward_max_m")),32)
            for side in (-lateral,lateral):
                pixels=project([(x,side) for x in xs])
                if len(pixels)>1:cv2.polylines(output,[np.asarray(pixels,np.int32)],False,(255,0,255),1,cv2.LINE_AA)
            if fresh and self.path_valid and len(path)>=2:
                pixels=project(path)
                if len(pixels)>1:cv2.polylines(output,[np.asarray(pixels,np.int32)],False,(0,0,255),3,cv2.LINE_AA)
                lookahead=float(np.clip(self.p("min_lookahead_m")+self.p("lookahead_speed_gain")*max(0.,self.target_speed),self.p("min_lookahead_m"),self.p("max_lookahead_m")))
                target=path[np.argmin(np.abs(np.linalg.norm(path,axis=1)-lookahead))]
                target_pixel=project([target])
                if target_pixel:cv2.circle(output,target_pixel[0],7,(255,0,0),-1,cv2.LINE_AA)
            state="VALID" if fresh and self.path_valid else "INVALID/STALE"
            text=f"SEC {self.active_section} PATH {state} conf={self.path_confidence:.2f} speed={self.target_speed:.2f} steer={self.target_steering:+.1f}"
            cv2.rectangle(output,(5,40),(min(output.shape[1]-5,635),65),(0,0,0),-1)
            cv2.putText(output,text,(10,58),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1,cv2.LINE_AA)
        except (TypeError,ValueError,IndexError):
            cv2.putText(output,"PATH OVERLAY ERROR",(10,58),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,0,255),1,cv2.LINE_AA)
    def publish_detections(self,instances,image):
        names=self.model_names;detections=[]
        for item in instances:
            class_id=int(item["class_id"]);name=names.get(class_id,str(class_id)) if isinstance(names,dict) else names[class_id]
            detections.append({"class_id":class_id,"class_name":str(name),"confidence":round(float(item.get("confidence",0.)),4),"xyxy":[round(float(v),1) for v in item.get("xyxy",[])]})
        self.detections_pub.publish(String(data=json.dumps({"stamp":{"sec":image.header.stamp.sec,"nanosec":image.header.stamp.nanosec},"frame_id":image.header.frame_id,"detections":detections})))
    def publish_latest_visualization(self):
        now=time.monotonic()
        if now-self.last_detections_image_time<self.visualization_period:return
        item=self.latest_visualization
        if item is None:return
        bgr,instances,masks,header=item;stamp=(header.stamp.sec,header.stamp.nanosec)
        if stamp!=self.last_visualized_stamp:
            self.cached_visualization_msg=bgr8_to_image(self.render_detections(bgr,instances,masks),header)
            self.last_visualized_stamp=stamp
        # Publish the latest rendered frame at the configured display cadence.
        # Repeating a frame is intentional when inference is below 30 Hz; the
        # true perception rate remains available on /camera/output_fps.
        if self.cached_visualization_msg is not None:
            self.detections_image_pub.publish(self.cached_visualization_msg)
            self.last_detections_image_time=now
    def process_latest(self):
        if self.busy:return
        inference_now=time.monotonic()
        if inference_now-self.last_inference_time<self.inference_period:return
        image=self.frames.take()
        if image is None:return
        self.last_inference_time=inference_now
        if not self.ready:return self.publish_invalid(image,self.failure)
        now=self.get_clock().now().nanoseconds*1e-9;valid,reason=validate_image_contract(image,self.camera_info,now,self.p("max_image_age_sec"),self.p("expected_image_width"),self.p("expected_image_height"))
        if not valid:return self.publish_invalid(image,reason)
        self.busy=True;started=time.perf_counter()
        try:
            bgr=image_to_bgr8(image)
            role_confidences={"road":self.p("road_confidence_threshold"),"white_line":self.p("lane_confidence_threshold"),"yellow_line":self.p("lane_confidence_threshold")}
            role_mask_thresholds={"road":self.p("road_mask_threshold"),"white_line":self.p("lane_mask_threshold"),"yellow_line":self.p("lane_mask_threshold")}
            instances,masks=self.backend.infer_navigation(bgr,self.inference_role_class_ids,self.p("mask_threshold"),role_confidences,role_mask_thresholds)
            words_mask=masks.pop("words",np.zeros_like(masks["road"]))
            masks["road"]=restore_road_surface_words(
                masks["road"],words_mask,proximity_pixels=20)
            masks=exclude_navigation_image_bottom(
                masks,self.p("navigation_bottom_exclusion_ratio"))
            for role in ("white_line","yellow_line"):
                masks[role]=filter_lane_components(masks[role],self.p("lane_minimum_component_area"),self.p("lane_maximum_horizontal_ratio"),self.p("lane_minimum_horizontal_width"))
            self.stop_pub.publish(Bool(data=contains_stop(instances,self.model_names,self.p("stop_confidence_threshold"))))
            names=self.model_names
            def class_name(item):
                class_id=int(item["class_id"])
                return str(names.get(class_id,class_id) if isinstance(names,dict) else names[class_id]).strip().lower().replace("_","").replace("-","")
            traffic20=any(class_name(item) in {"traffic20","speed20"} and float(item.get("confidence",0.0))>=self.p("traffic20_confidence_threshold") for item in instances)
            self.traffic20_pub.publish(Bool(data=traffic20))
            self.publish_detections(instances,image)
            latency=(time.perf_counter()-started)*1000.;self.latencies.add(latency)
            self.latency_pub.publish(Float32(data=float(latency)))
            # RQT is diagnostic output, not a driving-validity signal.  Keep
            # it live even when this frame contains no usable navigation mask;
            # perception_valid and the zero masks below still force a safe stop.
            self.output_frame_times.append(time.monotonic())
            self.latest_visualization=(bgr,instances,masks,image.header)
            for role in ("road","white_line","yellow_line"):
                if not validate_output_mask(masks[role],(image.height,image.width),allow_empty=True):raise ValueError(f"invalid_{role}_mask")
            if not has_navigation_mask(masks):raise ValueError("empty_navigation_masks: road/white_line/yellow_line all absent")
            stamp_age=self.get_clock().now().nanoseconds*1e-9-(image.header.stamp.sec+image.header.stamp.nanosec*1e-9)
            if latency>self.p("max_inference_latency_ms"):raise ValueError("inference_latency_limit")
            if stamp_age>self.p("max_image_age_sec"):raise ValueError("image_age_limit")
            for role,publisher in self.mask_pubs.items():
                publisher.publish(mono8_to_image(masks[role],image.header))
            self.valid_pub.publish(Bool(data=True));self.publish_status("ok")
        except Exception as error:self.publish_invalid(image,f"inference_failed: {error}")
        finally:self.busy=False

def main():
    rclpy.init();node=CameraYoloInferenceNode()
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown()
        if rclpy.ok():rclpy.shutdown()

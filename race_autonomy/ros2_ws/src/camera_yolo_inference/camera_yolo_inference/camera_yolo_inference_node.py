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
from sensor_msgs.msg import CameraInfo,Image
from std_msgs.msg import Bool,Float32,String
from .class_mapper import SemanticClassMapper
from .image_contract import LatestFrameBuffer,validate_image_contract
from .inference_backend import UltralyticsSegmentationBackend
from .inference_diagnostics import LatencyTracker
from .mask_postprocessor import has_navigation_mask,validate_output_mask
from .model_manifest import load_manifest
from .ros_image import bgr8_to_image,image_to_bgr8,mono8_to_image
from .stop_detection import contains_stop

class CameraYoloInferenceNode(Node):
    def __init__(self,backend=None):
        super().__init__("camera_yolo_inference_node");self.frames=LatestFrameBuffer();self.camera_info=None;self.busy=False;self.latencies=LatencyTracker();self.ready=False;self.failure="not_initialized";self.latest_visualization=None;self.last_visualized_stamp=None
        self.input_frame_times=deque(maxlen=240);self.output_frame_times=deque(maxlen=120)
        self.input_callbacks=MutuallyExclusiveCallbackGroup();self.inference_callbacks=MutuallyExclusiveCallbackGroup();self.visualization_callbacks=MutuallyExclusiveCallbackGroup()
        # use_sim_time is declared by rclpy itself and must not be redeclared.
        defaults={"segmentation_model_path":"","class_manifest_path":"","device":"cpu","input_width":640,"input_height":480,"inference_fps":40.0,"detections_image_fps":30.0,"confidence_threshold":.25,"mask_threshold":.5,"max_image_age_sec":.2,"max_inference_latency_ms":40.,"input_image_topic":"/camera/image_raw","input_camera_info_topic":"/camera/camera_info","expected_image_width":640,"expected_image_height":480,"stop_detected_topic":"/perception/stop_detected","stop_confidence_threshold":.25,"traffic20_confidence_threshold":.25,"publish_diagnostics":True,"require_cuda":False}
        for key,value in defaults.items():self.declare_parameter(key,value)
        output_qos=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,depth=1,reliability=QoSReliabilityPolicy.RELIABLE)
        self.mask_pubs={role:self.create_publisher(Image,f"/camera/{topic}",qos_profile_sensor_data) for role,topic in (("road","road_mask"),("white_line","white_line_mask"),("yellow_line","yellow_line_mask"))}
        self.detections_image_pub=self.create_publisher(Image,"/perception/detections_image",output_qos)
        self.detections_pub=self.create_publisher(String,"/perception/detections_json",output_qos)
        self.stop_pub=self.create_publisher(Bool,self.p("stop_detected_topic"),output_qos)
        self.traffic20_pub=self.create_publisher(Bool,"/perception/traffic20_detected",output_qos)
        self.valid_pub=self.create_publisher(Bool,"/camera/perception_valid",output_qos);self.latency_pub=self.create_publisher(Float32,"/camera/inference_latency_ms",output_qos);self.status_pub=self.create_publisher(String,"/camera/inference_status",output_qos)
        self.input_fps_pub=self.create_publisher(Float32,"/camera/input_fps",output_qos);self.output_fps_pub=self.create_publisher(Float32,"/camera/output_fps",output_qos)
        self.create_subscription(Image,self.p("input_image_topic"),self.on_image,qos_profile_sensor_data,callback_group=self.input_callbacks);self.create_subscription(CameraInfo,self.p("input_camera_info_topic"),lambda msg:setattr(self,"camera_info",msg),qos_profile_sensor_data,callback_group=self.input_callbacks)
        try:
            manifest=load_manifest(self.p("class_manifest_path"));self.mapper=SemanticClassMapper(manifest)
            input_shape=(int(self.p("input_height")),int(self.p("input_width")))
            self.backend=backend or UltralyticsSegmentationBackend(self.p("segmentation_model_path"),self.p("device"),input_shape,self.p("confidence_threshold"),self.p("require_cuda"));self.backend.load_model();self.mapper.resolve_model_classes(self.backend.get_model_names());self.backend.warmup();self.ready=True;self.failure="ok"
        except Exception as error:self.failure=f"initialization_failed: {error}"
        inference_fps=max(1.0,float(self.p("inference_fps")))
        self.inference_period=1.0/inference_fps
        self.last_inference_time=-float("inf")
        self.last_detections_image_time=-float("inf")
        # A 5 ms polling timer quantizes a 16.67 ms target period to roughly
        # 20 ms (50 Hz). Poll at 1 ms so a 60 Hz camera frame is handled with
        # negligible scheduler delay while LatestFrameBuffer still drops lag.
        self.create_timer(.001,self.process_latest,callback_group=self.inference_callbacks)
        self.create_timer(1.,self.publish_health,callback_group=self.input_callbacks)
        visualization_fps=float(self.p("detections_image_fps"))
        if visualization_fps>0.0:self.create_timer(1.0/visualization_fps,self.publish_latest_visualization,callback_group=self.visualization_callbacks)
    def p(self,name):return self.get_parameter(name).value
    def on_image(self,image):
        self.input_frame_times.append(time.monotonic());self.frames.push(image)
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
        output=bgr.copy();overlay=bgr.copy();names=self.backend.get_model_names()
        colors=((0,180,0),(255,255,255),(0,220,255),(0,0,255),(0,255,255),(0,255,0),(255,128,0),(255,0,255),(0,128,255),(255,0,0),(128,255,255),(180,180,180))
        role_colors={"road":(0,180,0),"white_line":(255,255,255),"yellow_line":(0,220,255)}
        for role,mask in masks.items():overlay[mask>0]=role_colors[role]
        output=cv2.addWeighted(output,.60,overlay,.40,0.)
        for instance in instances:
            class_id=int(instance["class_id"]);color=colors[class_id%len(colors)];name=names.get(class_id,str(class_id)) if isinstance(names,dict) else names[class_id]
            box=instance.get("xyxy",());
            if len(box)!=4:continue
            x1,y1,x2,y2=(int(v) for v in box);cv2.rectangle(output,(x1,y1),(x2,y2),color,1)
            cv2.putText(output,f"{name} {instance.get('confidence',0.):.2f}",(max(0,x1),max(20,y1)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        fps_text=f"INPUT {self.measured_fps(self.input_frame_times):.1f} FPS  OUTPUT {self.measured_fps(self.output_frame_times):.1f} FPS"
        cv2.rectangle(output,(5,5),(405,36),(0,0,0),-1)
        cv2.putText(output,fps_text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.58,(0,255,0),2,cv2.LINE_AA)
        return output
    def publish_detections(self,instances,image):
        names=self.backend.get_model_names();detections=[]
        for item in instances:
            class_id=int(item["class_id"]);name=names.get(class_id,str(class_id)) if isinstance(names,dict) else names[class_id]
            detections.append({"class_id":class_id,"class_name":str(name),"confidence":round(float(item.get("confidence",0.)),4),"xyxy":[round(float(v),1) for v in item.get("xyxy",[])]})
        self.detections_pub.publish(String(data=json.dumps({"stamp":{"sec":image.header.stamp.sec,"nanosec":image.header.stamp.nanosec},"frame_id":image.header.frame_id,"detections":detections})))
    def publish_latest_visualization(self):
        item=self.latest_visualization
        if item is None:return
        bgr,instances,masks,header=item;stamp=(header.stamp.sec,header.stamp.nanosec)
        if stamp==self.last_visualized_stamp:return
        self.detections_image_pub.publish(bgr8_to_image(self.render_detections(bgr,instances,masks),header));self.last_visualized_stamp=stamp
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
            role_class_ids={role:self.mapper.class_ids_for_role(role) for role in ("road","white_line","yellow_line")}
            instances,masks=self.backend.infer_navigation(bgr,role_class_ids,self.p("mask_threshold"))
            self.stop_pub.publish(Bool(data=contains_stop(instances,self.backend.get_model_names(),self.p("stop_confidence_threshold"))))
            names=self.backend.get_model_names()
            def class_name(item):
                class_id=int(item["class_id"])
                return str(names.get(class_id,class_id) if isinstance(names,dict) else names[class_id]).strip().lower().replace("_","").replace("-","")
            traffic20=any(class_name(item) in {"traffic20","speed20"} and float(item.get("confidence",0.0))>=self.p("traffic20_confidence_threshold") for item in instances)
            self.traffic20_pub.publish(Bool(data=traffic20))
            self.publish_detections(instances,image)
            for role in ("road","white_line","yellow_line"):
                if not validate_output_mask(masks[role],(image.height,image.width),allow_empty=True):raise ValueError(f"invalid_{role}_mask")
            if not has_navigation_mask(masks):raise ValueError("empty_navigation_masks: road/white_line/yellow_line all absent")
            latency=(time.perf_counter()-started)*1000.;self.latencies.add(latency)
            stamp_age=self.get_clock().now().nanoseconds*1e-9-(image.header.stamp.sec+image.header.stamp.nanosec*1e-9)
            if latency>self.p("max_inference_latency_ms"):raise ValueError("inference_latency_limit")
            if stamp_age>self.p("max_image_age_sec"):raise ValueError("image_age_limit")
            for role,publisher in self.mask_pubs.items():
                publisher.publish(mono8_to_image(masks[role],image.header))
            self.output_frame_times.append(time.monotonic())
            self.valid_pub.publish(Bool(data=True));self.latency_pub.publish(Float32(data=float(latency)));self.publish_status("ok")
            # RQT rendering runs in an independent callback group and consumes
            # only the latest result, so it cannot throttle perception.
            self.latest_visualization=(bgr,instances,masks,image.header)
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

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS=(("segmentation_model_path",""),("class_manifest_path",""),("device","cuda:0"),("require_cuda","true"),("input_width","640"),("input_height","480"),("inference_fps","40.0"),("detections_image_fps","30.0"),("confidence_threshold","0.25"),("mask_threshold","0.5"),("max_image_age_sec","0.2"),("max_inference_latency_ms","40.0"),("input_image_topic","/camera/image_raw"),("input_camera_info_topic","/camera/camera_info"),("expected_image_width","640"),("expected_image_height","480"),("publish_diagnostics","true"),("use_sim_time","false"))
def generate_launch_description():
    share=Path(get_package_share_directory("camera_yolo_inference"));config=str(share/"config"/"yolo_inference.yaml");manifest=str(share/"config"/"class_manifest.yaml");model=str(share/"models"/"hanla_yolo11n_seg_0811_best.engine")
    declarations=[DeclareLaunchArgument(name,default_value=(manifest if name=="class_manifest_path" else model if name=="segmentation_model_path" else default)) for name,default in ARGUMENTS]
    declarations.append(DeclareLaunchArgument("python_executable",default_value="/home/parkjinwoo/urrc_hanla/race_autonomy/ros2_ws/.yolo_runtime/bin/python"))
    declarations.append(DeclareLaunchArgument("launch_rqt",default_value="true"))
    params={name:LaunchConfiguration(name) for name,_ in ARGUMENTS}
    return LaunchDescription(declarations+[
        Node(package="camera_yolo_inference",executable="camera_yolo_inference_node",prefix=LaunchConfiguration("python_executable"),parameters=[config,params],output="screen"),
        Node(package="camera_yolo_inference",executable="stop_line_ground_node",output="screen"),
        TimerAction(period=2.0, actions=[
            Node(package="rqt_image_view",executable="rqt_image_view",name="camera_input_view",arguments=["/camera/image_raw"],condition=IfCondition(LaunchConfiguration("launch_rqt")),output="screen"),
        ]),
        TimerAction(period=7.0, actions=[
            Node(package="rqt_image_view",executable="rqt_image_view",name="yolo_output_view",arguments=["/perception/detections_image"],condition=IfCondition(LaunchConfiguration("launch_rqt")),output="screen"),
        ]),
    ])

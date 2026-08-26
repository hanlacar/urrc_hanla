from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    yolo=Path(get_package_share_directory("camera_yolo_inference"));nav=Path(get_package_share_directory("camera_navigation"));camera=Path(get_package_share_directory("camera_bringup"))
    args={name:LaunchConfiguration(name) for name in ("segmentation_model_path","class_manifest_path","device","require_cuda","input_width","input_height","inference_fps","detections_image_fps","confidence_threshold","mask_threshold","max_image_age_sec","max_inference_latency_ms","input_image_topic","input_camera_info_topic","expected_image_width","expected_image_height","publish_diagnostics","use_sim_time","launch_rqt")}
    defaults={"segmentation_model_path":str(yolo/"models"/"hanla_yolo11n_seg_0811_best.engine"),"class_manifest_path":str(yolo/"config"/"class_manifest.yaml"),"device":"cuda:0","require_cuda":"true","input_width":"640","input_height":"480","inference_fps":"40.0","detections_image_fps":"30.0","confidence_threshold":"0.25","mask_threshold":"0.5","max_image_age_sec":"0.2","max_inference_latency_ms":"100.0","input_image_topic":"/camera/image_raw","input_camera_info_topic":"/camera/camera_info","expected_image_width":"640","expected_image_height":"480","publish_diagnostics":"true","use_sim_time":"false","launch_rqt":"true","launch_camera":"false","color_width":"640","color_height":"480","color_fps":"60"}
    declarations=[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()]
    inference=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(yolo/"launch"/"yolo_inference.launch.py")),launch_arguments=args.items())
    camera_node=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera/"launch"/"d456_bringup.launch.py")),launch_arguments={name:LaunchConfiguration(name) for name in ("color_width","color_height","color_fps")}.items(),condition=IfCondition(LaunchConfiguration("launch_camera")))
    nav_config=str(nav/"config"/"camera_navigation.yaml")
    planner=Node(package="camera_navigation",executable="camera_path_planner_node",parameters=[nav_config,{"input_mode":"external","use_sim_time":args["use_sim_time"]}],output="screen")
    controller=Node(package="camera_navigation",executable="camera_path_controller_node",parameters=[nav_config,{"use_sim_time":args["use_sim_time"]}],output="screen")
    return LaunchDescription(declarations+[camera_node,inference,planner,controller])

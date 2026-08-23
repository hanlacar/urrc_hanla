from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory("camera_yolo_inference"))
    video=LaunchConfiguration("video_path")
    return LaunchDescription([
        DeclareLaunchArgument("video_path",default_value="/home/parkjinwoo/Downloads/asdf.mp4"),
        DeclareLaunchArgument("fps",default_value="15.0"),
        LogInfo(msg="Video CUDA test: output image is /perception/detections_image"),
        Node(package="camera_yolo_inference",executable="video_publisher_node",
             parameters=[{"video_path":video,"fps":LaunchConfiguration("fps"),
                          "width":640,"height":480}],output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch"/"yolo_inference.launch.py")),
            launch_arguments={"device":"cuda:0","require_cuda":"true","input_width":"640","input_height":"480",
                              "inference_fps":"40.0","detections_image_fps":"30.0",
                              "expected_image_width":"640","expected_image_height":"480","max_image_age_sec":"1.0",
                              "max_inference_latency_ms":"200.0"}.items()),
    ])

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    share=Path(get_package_share_directory("camera_yolo_inference"))
    navigation=Path(get_package_share_directory("camera_navigation"))
    control=Path(get_package_share_directory("race_control"))
    perception=Path(get_package_share_directory("race_perception"))
    video=LaunchConfiguration("video_path")
    return LaunchDescription([
        DeclareLaunchArgument(
            "video_path",
            default_value=(
                "/home/parkjinwoo/urrc_hanla/recordings/raw/"
                "20260829_170657.mp4"
            ),
        ),
        DeclareLaunchArgument("fps",default_value="60.0"),
        DeclareLaunchArgument("loop",default_value="false"),
        DeclareLaunchArgument("initial_section",default_value="5"),
        DeclareLaunchArgument("maximum_drive_stage",default_value="1"),
        DeclareLaunchArgument("launch_rqt",default_value="true"),
        LogInfo(msg=("Video CUDA path-generation test: YOLO labels + camera path + "
                     "Pure Pursuit overlay on /perception/detections_image; "
                     "path stability on /camera/path_accuracy")),
        Node(package="camera_yolo_inference",executable="video_publisher_node",
             parameters=[{"video_path":video,"fps":LaunchConfiguration("fps"),
                          "loop":LaunchConfiguration("loop"),
                          "width":640,"height":480}],
             remappings=[("/camera/image_raw","/video_test/image_raw"),
                         ("/camera/camera_info","/video_test/camera_info")],
             output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch"/"yolo_inference.launch.py")),
            launch_arguments={"device":"cuda:0","require_cuda":"true","input_width":"640","input_height":"480",
                              "inference_fps":"40.0","detections_image_fps":"30.0",
                              "navigation_bottom_exclusion_ratio":"0.0",
                              "input_image_topic":"/video_test/image_raw",
                              "input_camera_info_topic":"/video_test/camera_info",
                              "expected_image_width":"640","expected_image_height":"480","max_image_age_sec":"1.0",
                              "max_inference_latency_ms":"200.0",
                              "launch_rqt":LaunchConfiguration("launch_rqt")}.items()),
        Node(package="race_perception",executable="traffic_light_color",
             name="traffic_light_color",
             parameters=[str(perception/"config"/"traffic_light_color.yaml"),
                         {"final_image_topic":"/video_test/image_raw"}],
             output="screen"),
        Node(package="camera_navigation",executable="camera_path_planner_node",
             parameters=[str(navigation/"config"/"camera_navigation.yaml"),
                         {"input_mode":"external"}],
             remappings=[("/camera/camera_info","/video_test/camera_info")],
             output="screen"),
        Node(package="camera_navigation",executable="camera_path_controller_node",
             parameters=[str(navigation/"config"/"camera_navigation.yaml")],
             output="screen"),
        Node(package="rqt_image_view",executable="rqt_image_view",
             name="camera_path_debug_view",arguments=["/camera/path_debug_image"],
             condition=IfCondition(LaunchConfiguration("launch_rqt")),
             output="screen"),
        Node(package="race_control",executable="curvature_speed_planner",
             name="curvature_speed_planner_node",
             parameters=[str(control/"config"/"curvature_speed_planner.yaml")],
             output="screen"),
        Node(package="race_control",executable="course_mission",
             parameters=[str(control/"config"/"course_mission.yaml"),
                         # Use the complete image and the production near-path
                         # requirement now that no bumper/hood is visible.
                         {"initial_section":ParameterValue(
                              LaunchConfiguration("initial_section"),
                              value_type=int),
                          "path_required_near_point_m":1.0,
                          "vehicle_speed_topic":"/mcu/speed_mps",
                          "vehicle_speed_valid_topic":"/mcu/speed_valid",
                          "input_guard_topic":"/video/frame_active",
                          "input_guard_timeout_sec":0.3,
                          "output_maximum_stage":ParameterValue(
                              LaunchConfiguration("maximum_drive_stage"),
                              value_type=int)}],
             output="screen"),
    ])

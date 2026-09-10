"""One D456 process feeding synchronized RGB-D RTAB-Map mapping/localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    mode=LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in ("mapping", "localization"):
        raise RuntimeError("mode must be mapping or localization")
    database=str(Path(LaunchConfiguration("database").perform(context)).expanduser())
    if mode=="localization" and not Path(database).is_file():
        raise RuntimeError(f"database does not exist: {database}")
    camera=Node(package="realsense2_camera", executable="realsense2_camera_node",
        namespace="camera", name="camera", output="screen", emulate_tty=True,
        parameters=[{
            # The serial number uniquely selects the unit. Do not also filter
            # by product string: some D456 firmware reports a D45x-family name.
            "serial_no": ParameterValue(
                LaunchConfiguration("serial_no"), value_type=str),
            "enable_color": True, "enable_depth": True,
            # RTAB-Map only needs aligned RGB-D. Explicitly disable the two
            # infrared streams to avoid saturating the D455 USB connection.
            "enable_infra1": False, "enable_infra2": False,
            "rgb_camera.color_profile": "640x480x30",
            # Keep RGB at 30 Hz, but use the lower supported depth rate to
            # reduce USB bandwidth and avoid intermittent depth-start errors.
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_format": "RGB8", "depth_module.depth_format": "Z16",
            "align_depth.enable": True, "enable_sync": True,
            "enable_gyro": False, "enable_accel": False, "enable_motion": False,
            "pointcloud.enable": False, "publish_tf": False,
            "wait_for_device_timeout": 10.0}])
    base_tf=Node(package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x",LaunchConfiguration("camera_x"),"--y",LaunchConfiguration("camera_y"),
                   "--z",LaunchConfiguration("camera_z"),"--roll","0","--pitch",LaunchConfiguration("camera_pitch"),
                   "--yaw",LaunchConfiguration("camera_yaw"),"--frame-id","base_link","--child-frame-id","camera_link"])
    optical_tf=Node(package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x","0","--y","0","--z","0","--roll","-1.57079632679",
                   "--pitch","0","--yaw","-1.57079632679","--frame-id","camera_link",
                   "--child-frame-id","camera_color_optical_frame"])
    args=("-d --Grid/FromDepth true --Grid/RangeMax 8.0" if mode=="mapping" else
          "--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true")
    # Some RealSense ROS image messages carry sequence_id=0. Let RTAB-Map
    # generate its own IDs so those frames are not discarded.
    args += " --Mem/GenerateIds true"
    slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory("rtabmap_launch"))/"launch"/"rtabmap.launch.py")),
        launch_arguments={"localization":"true" if mode=="localization" else "false",
            "frame_id":"base_link", "rgb_topic":"/camera/camera/color/image_raw",
            "depth_topic":"/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic":"/camera/camera/color/camera_info",
            # At 30 Hz, a ~33 ms difference means adjacent frames were paired.
            # Reject RGB/depth pairs above 10 ms so corrupted/dropped frames
            # cannot shift depth measurements by a complete video frame.
            "approx_sync":"true", "approx_sync_max_interval":"0.01",
            "rgbd_sync":"true", "approx_rgbd_sync":"true",
            "subscribe_rgbd":"true",
            "topic_queue_size":"30", "sync_queue_size":"30",
            "qos":"2", "visual_odometry":"true",
            "database_path":database, "args":args,
            "rviz":LaunchConfiguration("rviz"), "rtabmap_viz":"false"}.items())
    monitor=Node(package="race_control", executable="sensor_sync_monitor", output="screen")
    return [camera,base_tf,optical_tf,slam,monitor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode",default_value="mapping"),
        DeclareLaunchArgument("database",default_value="/home/parkjinwoo/slam/maps/competition_d456.db"),
        # D455 currently installed on the test vehicle.
        DeclareLaunchArgument("serial_no",default_value="338122302896"),
        DeclareLaunchArgument("rviz",default_value="true"),
        DeclareLaunchArgument("camera_x",default_value="0.38"),
        DeclareLaunchArgument("camera_y",default_value="0.0"),
        DeclareLaunchArgument("camera_z",default_value="0.97"),
        DeclareLaunchArgument("camera_pitch",default_value="0.0872664626"),
        DeclareLaunchArgument("camera_yaw",default_value="0.0"),
        OpaqueFunction(function=setup)])

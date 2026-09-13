# camera_bringup

Minimal ROS 2 Jazzy bring-up for one Intel RealSense D456. It starts the
installed `realsense2_camera_node` once, enables RGB plus gyro/accel, and remaps the
wrapper's RGB outputs to a stable interface for future `camera_perception` use.
No relay, image conversion, RViz, rosbag, or composable container is started.

## Connection and USB check

Connect the D456 directly to a USB 3.x port with a USB 3-capable cable. The
expected default device serial is `338122302896`. Check discovery, firmware,
profiles, and USB link information with:

```bash
rs-enumerate-devices -s
rs-enumerate-devices | grep -E 'Name|Serial Number|Firmware Version|Usb Type Descriptor'
lsusb -t
```

The USB descriptor should report `3.x` and `lsusb -t` should show `5000M` or
faster. This launch file deliberately does not add a process to inspect USB.

## Build and run

```bash
cd /home/ww/camera_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_bringup
source install/setup.bash
ros2 launch camera_bringup d456_bringup.launch.py
```

Override the camera or RGB profile as follows:

```bash
ros2 launch camera_bringup d456_bringup.launch.py serial_no:=OTHER_SERIAL
ros2 launch camera_bringup d456_bringup.launch.py \
  color_width:=640 color_height:=480 color_fps:=60
```

Only positive integer dimensions and FPS are accepted. The requested profile
must be supported by the connected camera.

## ROS interface and checks

The wrapper normally publishes RGB below `/camera/camera/color`. Launch remaps
the two endpoints without copying image data:

| Wrapper endpoint | Public endpoint | Type |
|---|---|---|
| `/camera/camera/color/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera/color/camera_info` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

Measure and inspect the streams with:

```bash
ros2 topic type /camera/image_raw
ros2 topic type /camera/camera_info
timeout 15s ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --once --field width
ros2 topic echo /camera/image_raw --once --field height
timeout 10s ros2 topic echo /camera/camera_info --once
ros2 topic info -v /camera/image_raw
ros2 topic info -v /camera/camera_info
```

The RealSense publisher uses its supported `color_qos: SENSOR_DATA` setting;
no publisher or subscriber is added by this package.

## Design

The installed wrapper 4.58.1 exposes `realsense2_camera_node` and `rs_launch.py`.
This package starts the executable directly because the installed include file
does not expose a remapping argument. Direct launch provides the same parameter
interface while making the two required remaps explicit and runs exactly one
camera node. The ROS parameter file uses the fully qualified `/camera/camera`
node key; launch overrides its serial and `WIDTHxHEIGHTxFPS` profile.

Color uses `RGB8`, automatic exposure (which also leaves gain automatic), and
640x480 at 30 Hz by default. The same camera node publishes raw D456 gyro at
200 Hz and accel at 100 Hz; `unite_imu_method=0` deliberately preserves the
separate raw streams for `imu_manager`. Depth, all infrared streams, DDS
motion, RGBD, point cloud, depth alignment, and synchronization are explicitly
disabled. Wrapper TF publication is also disabled. These features are outside
Stage 1 and would consume USB bandwidth and compute.
USB webcams are not supported. Future `camera_perception` should
subscribe only to `/camera/image_raw` (and use `/camera/camera_info` when camera
calibration is needed).

The installed Wrapper 4.58.1 publishes the raw IMU inputs as
`/camera/camera/gyro/sample` and `/camera/camera/accel/sample`, both with
`sensor_msgs/msg/Imu`. `camera_bringup` remains the only process that opens the
D456; IMU consumers must subscribe to these topics rather than starting another
RealSense node.

## Troubleshooting

If no image or CameraInfo appears:

1. Run `rs-enumerate-devices -s` and confirm the requested serial exists.
2. Run `lsusb -t` and confirm a 5000M-or-faster USB link; change the cable/port
   if it is 480M (USB 2.0).
3. Confirm `ros2 pkg prefix realsense2_camera` resolves under `/opt/ros/jazzy`.
4. Read the camera-node error on screen. A missing device times out after 10
   seconds; an unsupported RGB profile is reported by the wrapper.
5. Confirm the installed wrapper arguments with
   `ros2 launch realsense2_camera rs_launch.py --show-args` and compare names
   with `config/d456.yaml` after wrapper upgrades.
6. Confirm the installed parameter file exists with
   `ros2 pkg prefix camera_bringup`, then inspect
   `share/camera_bringup/config/d456.yaml` below that prefix.
7. Use `ros2 node list`, `ros2 topic list`, and the commands above to distinguish
   a missing image publisher from a missing CameraInfo publisher.

The launch shuts down when the RealSense process exits, so startup failures are
visible rather than leaving an apparently running bring-up process.

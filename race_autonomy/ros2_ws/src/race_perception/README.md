# race_perception

ROS 2 Jazzy node for trained YOLOv8n detection on RealSense color images.

## External Python dependency

Install Ultralytics in the Python environment used by ROS:

```bash
python3 -m pip install ultralytics
```

The node subscribes to `/camera/camera/color/image_raw` from
`realsense2_camera`. It does not open the laptop camera or `/dev/video0`.
The configured model is `models/yolo/hanla_yolov8n_best.pt`.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd /home/parkjinwoo/urrc_hanla/race_autonomy/ros2_ws
colcon build --symlink-install --packages-select race_perception
source install/setup.bash
ros2 launch race_perception yolo_camera.launch.py
```

View `/perception/detections_image` with `rqt_image_view`, and inspect structured
results with:

```bash
ros2 topic echo /perception/detections_json
```

## Local path

Both the trained YOLO segmentation node and the optional classical
`lane_center` node can publish a short vehicle-relative path on
`/perception/local_path_json`. Do not run both path producers together.
Pure Pursuit control is kept in the separate `race_control` package.

```bash
ros2 launch race_control autonomy_stack.launch.py
ros2 topic echo /control/pure_pursuit_status_json
```

The stack is safe by default: `commanded_speed_mps` is `0.0`, and the vehicle
interface has its own actuation lock. Calibrate the four image-to-ground path
parameters and `wheelbase_m` on the actual vehicle before enabling propulsion.

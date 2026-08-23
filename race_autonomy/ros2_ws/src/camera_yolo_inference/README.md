# camera_yolo_inference

ROS 2 Jazzy adapter from an Ultralytics **segmentation** model to the existing
`camera_navigation` external-mask contract. It does not implement geometry,
Pure Pursuit, speed planning, or actuator commands.

The default deployment model is `hanla_yolo11n_seg_0811_best.engine`, exported
from `combined_0811_seg-2/weights/best.pt`. It is a fixed-shape `640x480`,
batch-1, FP16 TensorRT engine built for the local MX450. Ultralytics is
lazy-imported when the node loads the
engine. A detection checkpoint, missing road mapping, CUDA
requirement violation, stale/mismatched image, empty road, NaN mask, or latency
limit produces `perception_valid=false` and zero masks with the input header.

Input is the distorted raw 640×480 D456 RGB/BGR image. This package never
undistorts it. Letterbox padding is removed, probability masks are restored to
the original 640×480 coordinates, instances are OR-merged by semantic role,
and final masks are mono8 0/255 with the exact input timestamp and frame ID.

```bash
ros2 launch camera_yolo_inference yolo_inference.launch.py \
  segmentation_model_path:=/absolute/camera_seg_v001/best.engine \
  class_manifest_path:=/absolute/camera_seg_v001/class_manifest.yaml \
  device:=cuda:0

ros2 launch camera_yolo_inference camera_yolo_navigation.launch.py \
  segmentation_model_path:=/absolute/camera_seg_v001/best.engine \
  class_manifest_path:=/absolute/camera_seg_v001/class_manifest.yaml \
  device:=cuda:0 launch_camera:=false
```

`launch_camera=false` prevents duplicate D456 ownership. Outputs are road,
white-line, yellow-line masks, `/camera/perception_valid`, latency and status.
Optional stop/C-line/words roles are mapped in the manifest but not published
to navigation in this stage. Mask freshness, not Bool alone, is authoritative.

The default camera and inference rate limits are 60 Hz. The annotated detection
image is limited to 30 Hz to leave CPU and ROS transport headroom. Two
`rqt_image_view` windows open by default: `/camera/image_raw` for the input and
`/perception/detections_image` for the annotated output. Set
`launch_rqt:=false` for headless operation. The engine input is
exactly `NCHW=(1,3,480,640)`; do not reuse it on another GPU architecture or
with another input shape. Rebuild the engine from the `.pt` file on the target
GPU when either changes.

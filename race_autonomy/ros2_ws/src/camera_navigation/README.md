# camera_navigation

ROS 2 Jazzy metric camera-navigation prototype for the pre-YOLO stage. It turns
road/line masks into a `base_link` path and publishes target speed (m/s) and
steering (degree). It never publishes `/cam_drive` or `/cam_wheel`; there is no
actuator-output parameter or implementation in this stage.

## Safety and calibration status

Current measured camera position is `(x,y,z)=(0.245,0.004,0.850) m` and mounting
RPY `(0,-5,0) degree`. Negative mounting pitch means that the camera looks
down under this package's documented REP-103 convention. X/Y, Roll/Yaw,
wheelbase, steering limit, BEV bounds, lane width, and every speed/acceleration
limit are temporary or unconfirmed. Measure camera height and mounting pitch
again before vehicle use. The actual optical-frame orientation must also be
verified.

The D456 IMU cannot automatically determine camera height or mounting yaw.
IMU compensation does not replace the complete camera extrinsic calibration.

| Parameter | Current value | Used in | Purpose |
|---|---:|---|---|
| `camera_z_m` | 0.86 m (provisional lens-center estimate) | `camera_geometry.py`, `bev_transform.py` | ray/ground intersection, pixel-to-meter scale, BEV and near/far ROI |
| `camera_x_m`, `camera_y_m` | 0, 0 m (unconfirmed) | `camera_geometry.py`, `bev_transform.py` | translate camera-ground points into `base_link` |
| `camera_mount_pitch_deg` | -10.0° (measured downward angle) | `camera_geometry.py`, `bev_transform.py` | camera-to-base rotation, horizon, projection and homography |
| mount Roll/Yaw | 0°, 0° (unconfirmed) | `camera_geometry.py`, `bev_transform.py` | lateral leveling/asymmetry and camera-forward alignment |
| IMU Pitch/Roll | runtime degree | planner diagnostics only | slope observation/future policy; deliberately excluded from ordinary BEV |
| IMU Yaw | relative degree | `turn_path_generator.py` interface | turn progress, exit-heading assistance |

The explicit optical mapping is optical +Z → base +X, optical +X → base -Y,
and optical +Y → base -Z. Fixed rotation is `R_mount @ R_optical`, followed by
the fixed camera translation. Ground is the local vehicle plane `base_link
z=0`. Since the camera and local road plane move with the vehicle, IMU
Pitch/Roll are not multiplied into this transform; doing so would double-correct
a slope. IMU validity therefore never invalidates ordinary lane following.
Relative IMU Yaw is used only as turn-progress evidence when it is valid.

## Camera contract and coordinates

`camera_bringup` remaps D456 RGB8 to `/camera/image_raw` and CameraInfo to
`/camera/camera_info` (default 640×480 at 30 Hz). Its configuration does not
record a trustworthy runtime RGB optical frame. With
`camera_optical_frame_id: ""`, the planner obtains the actual frame from the
first `CameraInfo.header.frame_id`, locks it, and invalidates paths if it
changes. Check it on hardware with:

```bash
ros2 topic echo /camera/camera_info --once --field header.frame_id
```

BEV and Path use `base_link`: X forward in metres and Y left in metres. Current
temporary BEV bounds are X 0.3–8.0 m, Y -2.0–+2.0 m, at 0.02 m/pixel.

## Interfaces and behavior

The planner consumes CameraInfo, road/white/yellow masks, perception validity,
`/imu_valid`, `/imu_pitch`, `/imu_roll`, `/imu_yaw`, and
`/mission/turn_direction`. It publishes `/camera/path` (`nav_msgs/Path`, metres,
`base_link`), validity, confidence, and mode. Modes are: 0 INVALID, 1 BOTH,
2 LEFT_ONLY, 3 RIGHT_ONLY, 4 ROAD_ONLY, 5 TEMPORAL_HOLD, 6 TURN_TEMPLATE, and
7 LANE_REACQUIRE. Lane-follow priority is both, single boundary, road-only,
temporal hold, then invalid.

`input_mode` is exactly `mock` or `external`; invalid values fail node startup.
Mock mode directly processes numpy arrays and creates no mask subscriptions or
mask publishers. External mode disables mock generation and validates all three
masks for matching source timestamp (0.05 s tolerance), frame, dimensions,
mono encoding, CameraInfo resolution, timestamp order, and 0.20 s freshness.
`perception_valid` alone is never treated as synchronization proof.

The controller publishes only `/camera/target_speed_mps` and
`/camera/target_steering_deg`. Invalid input produces zero targets. Curvature
and lateral acceleration constrain speed; turn, single-boundary, and road-only
modes have successively conservative caps. Pure pursuit uses metric lookahead,
placeholder wheelbase/steering limits, and a steering rate limit.

The turn module provides LANE_FOLLOW → TURN_PREPARE → TURN_ENTER →
TURN_EXECUTE → EXIT_SEARCH → LANE_REACQUIRE, plus ABORT. TURN_EXECUTE uses a
cubic Bezier quarter-turn path with configurable minimum radius. Relative IMU
yaw progress and detected exit/lane evidence—not elapsed time—advance normal
completion. Invalid Yaw lowers confidence and lack of progress reaches ABORT
instead of holding a turn indefinitely. Road bounds reject escaping paths.
Without encoder/odometry, actual turn-position progress remains limited; future
odometry and steering feedback are required for vehicle deployment.

Mock scenarios are STRAIGHT_BOTH, CURVE_LEFT_BOTH, CURVE_RIGHT_BOTH,
LEFT_BOUNDARY_ONLY, RIGHT_BOUNDARY_ONLY, ROAD_ONLY, NO_BOUNDARY,
INTERSECTION_LEFT/RIGHT, TURN_EXIT_LEFT/RIGHT, and STALE_INPUT. They publish the
the same internal mask schema as future perception without publishing ROS mask
topics. Pitch/Roll and IMU validity do not change its ordinary BEV geometry.

## Build, test, and run

```bash
cd /home/ww/camera_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select camera_bringup camera_navigation
source install/setup.bash
colcon test --packages-select camera_navigation
colcon test-result --verbose

ros2 launch camera_navigation camera_navigation_mock.launch.py
# Real masks/IMU, while camera_bringup is started separately exactly once:
ros2 launch camera_navigation camera_navigation.launch.py
```

After sensors arrive, verify CameraInfo K/frame/encoding and optical axes,
measure all extrinsics, wheelbase and steering limits, then connect YOLO mask
publishers, `/imu_*`, mission direction, and later `/vehicle/odom` plus
`/vehicle/steering_angle_deg`. Actuator mapping is deliberately outside this
package and this stage. Current outputs are only `/camera/target_speed_mps` and
`/camera/target_steering_deg`; actuator commands follow encoder and steering
calibration in a later stage.

## Stage 1B: physical geometry validation

`camera_geometry_validation` is a one-shot diagnostic executable, not a third
continuous runtime node. It receives real `/camera/camera_info`, observes frame
stability and publication frequency, reads surveyed points, evaluates
pixel-to-ground and ground-to-pixel errors, and writes a YAML report. The
default destination is outside source/build/install:

```text
~/.config/camera_navigation/camera_geometry_validation.yaml
```

The shipped [reference template](config/camera_geometry_reference.yaml) leaves
every pixel as `null`. Null pixels are reported as unmeasured and excluded; they
are never replaced with invented values. Populate them using this flat-ground
procedure:

The public `/camera/image_raw` stream is distorted raw RGB. Reference pixels
must be measured on its original 640×480 coordinates—never on a resized image.
The geometry contract is raw-only: `pixel_to_ground` uses CameraInfo K and the
five plumb-bob D coefficients through `cv2.undistortPoints`, then constructs a
normalized optical ray. `ground_to_pixel` uses the same K and D through
`cv2.projectPoints` to return a distorted raw pixel. P is recorded and
validated but is not substituted for raw K. Rectified pixels and unsupported
distortion models are rejected explicitly; D=0 remains the pinhole case.

1. Rigidly mount the D456 at approximately 0.86 m, facing forward and about
   5° down. The mounting Pitch is physically measured; verify the remaining
   extrinsics before final competition use.
2. Put the vehicle on a level surface and mark the ground projection of the
   `base_link` origin.
3. Mark X=0.5, 1.0, and 2.0 m on the centerline.
4. At X=1.0 m, mark left Y=+0.5 m and right Y=-0.5 m.
5. View `/camera/image_raw` with `rqt_image_view`, or save one image whose
   CameraInfo timestamp/configuration matches, and record each mark's pixel
   `(u,v)`.
6. Copy the reference template to a writable survey file, fill only observed
   pixels and the real `frame_id`, then run the validator.

```bash
cd /home/ww/camera_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

# Terminal 1: camera_bringup remains the only D456 owner.
ros2 launch camera_bringup d456_bringup.launch.py

# Terminal 2: inspect the physical stream.
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image_raw
rqt_image_view /camera/image_raw

mkdir -p ~/.config/camera_navigation
cp install/camera_navigation/share/camera_navigation/config/camera_geometry_reference.yaml \
  ~/.config/camera_navigation/camera_geometry_reference.yaml
# Edit the copied file with physically observed pixel_u/pixel_v values.
ros2 run camera_navigation camera_geometry_validation \
  --reference ~/.config/camera_navigation/camera_geometry_reference.yaml \
  --output ~/.config/camera_navigation/camera_geometry_validation.yaml
```

CameraInfo passes only when frame ID is nonempty and stable, resolution is
640×480, `fx/fy` are positive, `cx/cy` lie inside the image, and D/K/R/P are
finite. The supported physical contract additionally requires `plumb_bob`, five
finite D coefficients, identity R, no binning, and an uncropped full-image ROI.
Reference dimensions and optional frame ID must match CameraInfo. The measured
D456 frame in this stage is `camera_color_optical_frame`; the reference template
is intentionally left blank so a copied user survey file must opt into that
frame rather than being silently overwritten.
Default development tolerances are 0.15 m near (X≤1 m), 0.25 m far (up to 3 m),
0.15 m lateral, and 1 pixel round trip. These are not final vehicle limits.

PASS means every measured point meets position, lateral-sign, forward, and
round-trip checks. PARTIAL means geometry remains finite and directionally
correct but a minority exceeds tolerance. FAIL covers CameraInfo/transform
failure, X≤0, lateral sign reversal, nonfinite results, or a majority exceeding
tolerance. With no measured pixels the explicit result is NOT_RUN.

The rotation is active: vectors are transformed as
`v_base = R_mount @ R_optical_to_base @ v_optical`. For optical forward, base Z
is -0.0872 at -5°, -0.1736 at -10°, and -0.2588 at -15°. Thus increasingly
negative mount Pitch points the optical axis increasingly downward under this
package's convention. With the synthetic K and pixel (320,400), corresponding
flat-ground distances are 1.073, 0.856, and 0.700 m. Real surveyed points are
still required to validate the provisional -10° mounting value itself.

## Stage 1C: manual external-mask validation

Stage 1C validates the existing external-input path without YOLO. These tools
are invoked only for a test session and do not change the two persistent nodes.
No physical sample is shipped or synthesized by the package.

Capture one real D456 frame and corresponding CameraInfo:

```bash
ros2 run camera_navigation camera_capture_frame \
  --output ~/.config/camera_navigation/manual_samples/sample_0001
```

The capture requires matching frame IDs/timestamps, original 640×480 RGB8 or
BGR8, and writes lossless PNG plus metadata containing source encoding,
timestamp, frame, K/D and the provisional extrinsic. Output under source,
build, or install is rejected. Complete the directory manually:

```text
sample_0001/
├── image_raw.png
├── metadata.yaml
├── road_mask.png
├── white_line_mask.png
└── yellow_line_mask.png
```

All masks must be lossless 640×480 mono8 PNG in exactly the raw image pixel
coordinates, with only values 0 and 255. Never resize, crop, or rotate the RGB
or masks. Road must be nonempty; an absent white or yellow class is represented
by an all-zero mask. Lane strokes must lie on or immediately beside the road
region.

Validate the files and produce path/control metrics plus optional debug PNGs:

```bash
ros2 run camera_navigation camera_manual_mask_validation \
  --sample ~/.config/camera_navigation/manual_samples/sample_0001 \
  --output ~/.config/camera_navigation/manual_samples/sample_0001/report.yaml \
  --debug-output ~/.config/camera_navigation/manual_samples/sample_0001/debug
```

The report records validity, mode/confidence, pose count and X/Y ranges,
curvature, target speed/steering, finite status, path points outside road, mask
pixel counts, and failure reason. Debug output is disabled when
`--debug-output` is omitted. When enabled it writes raw overlay, three
K+D-undistorted masks, three metric BEV masks, and a BEV composite containing
boundaries, center path, Pure Pursuit target and control text.

Run the normal two-node external launch and temporary playback in separate
terminals:

```bash
ros2 launch camera_navigation camera_navigation.launch.py input_mode:=external

ros2 run camera_navigation camera_manual_mask_playback \
  --sample ~/.config/camera_navigation/manual_samples/sample_0001

# RViz2: Fixed Frame=base_link, Path topic=/camera/path
rviz2
```

Playback validates the sample before publishing and sends road/white/yellow
`mono8` images with one shared current timestamp, frame ID and dimensions, plus
`/camera/perception_valid=true`. It runs for five seconds at 10 Hz by default;
`--once` is also available. The planner remains `input_mode=external`, creates
no mask publisher, and mock generation stays disabled.

Suggested real samples are straight, gentle left/right curves, left-only,
right-only, road-only, intersection entry, lane loss during left/right turns,
and exit-lane reappearance. Because the five Stage 1B surveyed pixels remain
null, Stage 1C can exercise the full data path but cannot yet establish absolute
metre accuracy.

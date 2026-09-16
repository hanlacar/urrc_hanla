# imu_manager

Filters the D456 gyro and accelerometer streams for the vehicle stack. The
camera is owned by `camera_bringup`; this package never opens a second camera.

Run the normal node with:

```bash
ros2 launch imu_manager imu_manager.launch.py
```

Keep the vehicle stationary on level ground during startup calibration.


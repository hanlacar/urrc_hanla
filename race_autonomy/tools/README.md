# Vehicle terminal test

Run the low-stage Mega v21 test controller from one terminal:

```bash
cd /home/parkjinwoo/urrc_hanla
python3 race_autonomy/tools/vehicle_terminal_test.py
```

Press `w` repeatedly for forward stages 1, 2, and 3, or `r` repeatedly for
reverse stages 1, 2, and 3. Press `a`/`d` repeatedly for left/right steering
stages 1, 2, and 3. A direction change starts again at stage 1; the maximum is
clamped at stage 3. Press `c` to center the steering.

Do not run it at the same time as `arduino_serial_bridge_node`; only one
process may own the serial port. Press `Space`, `s`, or `x` for an immediate
software stop. Press `q` or `Ctrl+C` to stop and exit. The physical E-Stop
remains the primary emergency control.

Monitor calibrated IMU Pitch continuously while manually raising or lowering
the vehicle to a 15-degree target:

```bash
python3 race_autonomy/tools/imu_angle_monitor.py --target-deg 15
```

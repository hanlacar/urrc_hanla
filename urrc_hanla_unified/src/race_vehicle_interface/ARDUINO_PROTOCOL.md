# Arduino serial protocol

The ROS bridge uses 115200 baud and newline-terminated ASCII frames.

## Arduino to ROS (T870 v29)

```text
T,<encoder_count>,<rpm>,<steering_angle_deg>\n
```

The installed T870 v29 firmware emits `STATUS` on the non-actuating `S` query:

```text
STATUS,state,fault,adc,center_adc,drive_pwm,rpm,encoder_count,
       steering_position_ms,steering_target_ms,steering_limit_ms
```

Example:

```text
T,1234,56.7,-12.5
```

## ROS to Arduino (T870 v29 firmware)

ROS uses signed stages `-3..3`. The bridge maps them to the firmware lines:

```text
0  -> 1.00 (stop)
+1 -> 2.00 (forward 1)
+2 -> 3.00 (forward 2)
+3 -> 4.00 (forward 3)
-1 -> 6.00 (reverse 1)
-2 -> 7.00 (reverse 2)
-3 -> 8.00 (reverse 3)
```

Nonzero Lookahead steering degrees are converted to the proportional command
`V,-1.0..1.0`. For a configured maximum of 27 degrees, `13.5 deg` becomes
`V,0.500`. An exact `0 deg` Lookahead target sends `C`, returning the
firmware's accumulated steering position to logical zero. This does not use
the A0 neutral reference.

Current measured/assumed vehicle geometry:

```text
wheelbase: 0.78 m
wheel diameter: 0.30 m (radius 0.15 m)
maximum steering: +/-27 deg
L1/R1 steering: approximately -/+9 deg until detailed angle calibration
ROS stage 1 -> firmware 2.00 (PWM 50): 0.229 m/s
ROS stage 2 -> firmware 3.00 (PWM 100): 0.526 m/s
ROS stage 3 -> firmware 4.00 (PWM 150): not measured
linear fallback calibration: 3.8022813688 stage/(m/s), based on stage 2
```

`M` sets the current manually aligned steering position as logical zero. It is
intentionally not sent automatically by ROS because an incorrect wheel
position would store a bad zero.

The third `T` telemetry field is accumulated steering motor drive time in
milliseconds, with a documented maximum of about `+/-440 ms`; it is not a
measured angle. The bridge publishes the raw value on `/steer_position_ms` and
a linear estimate on `/steer_angle`, using `+/-440 ms == +/-27 deg`. The angle
must therefore be treated as an estimate until physically calibrated.

T870 v29 declares its measured straight-center reference as `A0=363`. This
value is diagnostic only and is not used to generate steering
commands. The bridge stores it as `steering_neutral_a0` and publishes the raw
sensor reading on `/steer_a0`, its signed difference from neutral on
`/steer_a0_error`, and the tolerance check on `/steer_at_neutral`. This A0
reference is the authoritative physical neutral; the firmware's accumulated
motor time is not a measured steering center.

The ROS side also starts locked. Transmission requires all of the following:

1. `allow_transmit: true` in `arduino_bridge.yaml`.
2. `maximum_abs_stage` set within the verified range `1..3`.
3. Fresh Arduino telemetry and fresh `/mcu_drive` (`Float32`),
   `/mcu_wheel` (`Int32`) messages.
4. A successful `/arduino_bridge/set_tx_enabled` service call.

The supplied v29 firmware currently has a 60-second command watchdog intended
for manual serial testing. The ROS bridge repeats the active drive command
every 0.5 seconds and independently disarms on stale commands after 0.3 seconds.
If ROS commands or feedback
go stale, the bridge sends `1.00` and `C`, then disarms. If the bridge
crashes or USB disconnects, the firmware-only fallback may take up to 60
seconds; change `DRIVE_COMMAND_TIMEOUT_MS` to 2000 for competition. The physical
E-Stop remains the emergency stop.

Do the first transmission test with the drive wheels lifted from the ground,
using forward stage 1 only.

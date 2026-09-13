# lidar_ws_plus_bringup

This package owns the shared real-vehicle LiDAR drivers, LiDAR static TFs, and
the only publishers of `/lidar_drive`, `/lidar_wheel`, and `/lidar_stop`.

Use the workspace-level `README.md` for build, launch, and safety procedures.
All real entry points force `use_sim_time:=false`; the parking entry points use
AMCL for `map -> odom` and expect the external MCU bridge to own `/odom` and
`odom -> base_link`.

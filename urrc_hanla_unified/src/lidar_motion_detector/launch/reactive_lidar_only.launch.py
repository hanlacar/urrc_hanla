#!/usr/bin/env python3

"""Minimal bringup for reactive_avoidance_controller testing.

Publishes only:
  - /front/scan (frame_id: front_laser) via rplidar_node
  - base_link -> front_laser static TF

No rear lidar, no lidar_motion_detector nodes, no /lidar_drive or other
command topics. reactive_avoidance_controller (avoidance_gazebo) is the
sole consumer of /front/scan and the sole publisher of driving commands
in this configuration.

WARNING - only one base_link->front_laser publisher may run at a time.
This launch file's `base_to_front_laser_static_tf` node publishes that
transform. If a manually-started
`ros2 run tf2_ros static_transform_publisher ...` for the same frames is
still running from earlier testing, the two publishers race and TF2 will
keep serving whichever one last latched its transform -- edits to the
front_laser_roll/pitch/yaw arguments below will appear to have no effect.
Before every run of this launch file:
    pkill -f static_transform_publisher
    ros2 daemon stop && ros2 daemon start
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    front_serial_port = LaunchConfiguration('front_serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    base_frame = LaunchConfiguration('base_frame')
    front_laser_frame = LaunchConfiguration('front_laser_frame')
    front_laser_inverted = LaunchConfiguration('front_laser_inverted')

    pose_names = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    front_pose = {
        name: LaunchConfiguration(f'front_laser_{name}')
        for name in pose_names
    }

    declarations = [
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('front_laser_frame', default_value='front_laser'),
        # Lidar is mounted upside-down, which mirrors the scan left/right.
        # Rigid TF rotation (roll/yaw) cannot undo a mirror, only the
        # driver's inverted flag can. Default True for this mount;
        # override at launch if the mount changes.
        DeclareLaunchArgument('front_laser_inverted', default_value='true'),
        # TODO: confirm these against the real vehicle's front lidar mount
        # position before trusting reactive_avoidance_controller output.
        DeclareLaunchArgument('front_laser_x', default_value='0.75'),
        DeclareLaunchArgument('front_laser_y', default_value='0.0'),
        DeclareLaunchArgument('front_laser_z', default_value='0.10'),
        # roll/pitch/yaw stay at 0 (identity): the upside-down mount's
        # left/right mirror is corrected entirely by the driver's inverted
        # flag above, not by a TF rotation. Do NOT set roll/yaw to pi to
        # "fix" the mirror -- verified with the rotation math below, a
        # nonzero roll/pitch/yaw here either does nothing for the mirror
        # (inverted already handles it) or flips the +x (front/rear) axis,
        # which is the exact bug this config fixes. If tf2_echo ever shows
        # rear objects on the +x side, the fix is a duplicate static TF
        # publisher (see module docstring warning) or a wrong
        # front_laser_inverted value -- never a nonzero roll/pitch/yaw here.
        DeclareLaunchArgument('front_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('front_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('front_laser_yaw', default_value='0.0'),
    ]

    front_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='front_rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': front_serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': front_laser_frame,
            'inverted': front_laser_inverted,
            'angle_compensate': True,
        }],
        remappings=[('/scan', '/front/scan')],
    )

    front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_front_laser_static_tf',
        output='screen',
        arguments=[
            '--x', front_pose['x'], '--y', front_pose['y'],
            '--z', front_pose['z'], '--roll', front_pose['roll'],
            '--pitch', front_pose['pitch'], '--yaw', front_pose['yaw'],
            '--frame-id', base_frame, '--child-frame-id', front_laser_frame,
        ],
    )

    return LaunchDescription(declarations + [
        front_lidar,
        front_tf,
    ])


# Verification commands after building and sourcing the workspace:
#   ros2 topic echo /front/scan --qos-reliability best_effort --once
#   ros2 run tf2_tools view_frames        # base_link, front_laser must appear
#   ros2 topic list | grep -E "scan|front"
#   ros2 topic hz /front/scan
#
# Full verification procedure (front/rear + left/right orientation fix):
#   1. Kill any leftover manual static TF publisher and clear the TF
#      cache, or step 5 may still show the old, wrong transform:
#        pkill -f static_transform_publisher
#        ros2 daemon stop && ros2 daemon start
#   2. cd <workspace> && source install/setup.bash
#   3. ros2 launch lidar_motion_detector reactive_lidar_only.launch.py \
#        front_serial_port:=/dev/ttyUSB0
#   4. ros2 node list | grep -i static
#      -> exactly one match: /base_to_front_laser_static_tf
#         (a second match means a duplicate publisher is still running;
#         go back to step 1)
#   5. ros2 run tf2_ros tf2_echo base_link front_laser
#      -> Rotation RPY (degree) must be ~[0, 0, 0]
#      -> Matrix first row (x-axis) must be ~[1, 0, 0]
#      This is guaranteed by construction as long as
#      front_laser_roll/pitch/yaw are left at their 0.0 defaults and no
#      other node is publishing this transform (step 4) -- it does not by
#      itself prove the physical mount is correct, only that no stray
#      rotation or duplicate publisher is corrupting the TF.
#   6. Physical check (this is what actually proves the mount is right):
#      In RViz, set Fixed Frame = base_link and View = TopDownOrtho with
#      Angle = 0. Place one object directly in front of the vehicle -> it
#      must render toward the top of the screen (+x). Move that object to
#      the vehicle's right side -> it must render on the right side of the
#      screen (-y, per REP-103: +y is left). If front/rear is swapped,
#      recheck steps 1 and 4 for a duplicate TF publisher -- do not
#      "fix" it with a nonzero roll/pitch/yaw here (see the reasoning
#      comment above front_laser_roll). If only left/right is swapped,
#      flip front_laser_inverted (true<->false) and repeat step 6; leave
#      roll/pitch/yaw untouched.

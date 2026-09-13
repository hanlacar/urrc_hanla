#!/usr/bin/env python3

import json
import math

from geometry_msgs.msg import Point
from lidar_parking_planner.lidar_safety import LidarSafetyGate
from lidar_parking_planner.obstacle_tracker import (
    DYNAMIC_APPROACHING,
    DYNAMIC_RECEDING,
    ObstacleSafetyGate,
    ObstacleTracker,
    SafetyConfig,
    STATIC,
    track_to_dict,
    TrackerConfig,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# This node's existing vehicle-command interface is std_msgs/Float32 for both
# topics. The safety gate is applied to cmd_drive before this publisher and does
# not create a competing publisher.
CMD_DRIVE_MSG_TYPE = Float32
CMD_WHEEL_MSG_TYPE = Float32


class ParkingPlanner(Node):
    IDLE = 'IDLE'
    FAIL_SAFE_STOP = 'FAIL_SAFE_STOP'
    WAIT_ODOM = 'WAIT_ODOM'

    T_SCAN = 'T_SCAN_RIGHT'
    T_READY = 'T_BACKUP_READY_BY_FRONT_CURB'
    T_REVERSE = 'T_REVERSE_IN_SCAN_ONLY'
    T_DONE = 'T_PARK_DONE'
    T_EXIT = 'T_EXIT_RIGHT_SCAN_ONLY'

    P_SCAN = 'P_SCAN_RIGHT'
    P_ALIGN = 'P_ALIGN_SCAN_ONLY'
    P_REVERSE = 'P_REVERSE_IN_SCAN_ONLY'
    P_DONE = 'P_PARK_DONE'
    P_EXIT = 'P_EXIT_RIGHT_SCAN_ONLY'

    def __init__(self):
        super().__init__('parking_planner_node')
        self._declare_parameters()
        self._read_parameters()

        if self.localization_mode not in ('scan_only', 'odom'):
            self.get_logger().warning(
                f'Unsupported localization_mode={self.localization_mode}; using scan_only')
            self.localization_mode = 'scan_only'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.front_points = []
        self.rear_points = []
        self.front_range_min = math.inf
        self.rear_range_min = math.inf
        self.front_stamp = None
        self.rear_stamp = None
        self.odom_stamp = None
        self.front_tf_ok = False
        self.rear_tf_ok = False

        self.state = self.WAIT_ODOM if self.localization_mode == 'odom' else self.IDLE
        self.state_entered = self.get_clock().now()
        self.selected_slot = 'unknown'
        self.slot_candidate = 'unknown'
        self.slot_confirm_count = 0
        self.mission_t_mode = False
        self.mission_parallel_mode = False
        self.t_mode_request = False
        self.parallel_mode_request = False
        self.t_mode = False
        self.parallel_mode = False
        self.exit_requested = False
        self.fail_reason = ''
        self.safety_scan_stamp = None
        self.encoder_stamp = None
        self.ego_speed_mps = 0.0
        self.last_commanded_steering_deg = 0.0
        self.obstacle_tracks = []
        self.test_drive_request = 0.0
        self.test_drive_request_stamp = None
        self._test_request_logged = False
        self._last_test_drive_report = None
        self.lidar_safety = LidarSafetyGate(
            stop_distance_m=self.stop_distance_m,
            resume_distance_m=self.resume_distance_m,
            front_sector_deg=self.front_sector_deg,
            min_valid_range_m=self.min_valid_range_m,
            stop_confirm_scans=self.stop_confirm_scans,
            clear_confirm_scans=self.clear_confirm_scans,
            scan_timeout_sec=self.scan_timeout_sec,
            stop_on_scan_timeout=self.stop_on_scan_timeout,
        )
        self.obstacle_tracker = ObstacleTracker(TrackerConfig(
            front_sector_deg=self.front_sector_deg,
            min_valid_range_m=self.min_valid_range_m,
            cluster_distance_threshold_m=self.cluster_distance_threshold_m,
            min_cluster_points=self.min_cluster_points,
            association_distance_m=self.association_distance_m,
            association_angle_deg=self.association_angle_deg,
            track_timeout_sec=self.track_timeout_sec,
            min_dt_sec=self.min_dt_sec,
            max_dt_sec=self.max_dt_sec,
            velocity_filter_alpha=self.velocity_filter_alpha,
            max_reasonable_object_speed_mps=self.max_reasonable_object_speed_mps,
            static_speed_threshold_mps=self.static_speed_threshold_mps,
            dynamic_speed_threshold_mps=self.dynamic_speed_threshold_mps,
            classification_confirm_frames=self.classification_confirm_frames,
        ))
        self.obstacle_safety = ObstacleSafetyGate(SafetyConfig(
            static_stop_distance_m=self.static_stop_distance_m,
            dynamic_stop_distance_m=self.dynamic_stop_distance_m,
            unknown_stop_distance_m=self.unknown_stop_distance_m,
            static_resume_distance_m=self.static_resume_distance_m,
            dynamic_resume_distance_m=self.dynamic_resume_distance_m,
            unknown_resume_distance_m=self.unknown_resume_distance_m,
            stop_confirm_frames=self.stop_confirm_frames,
            clear_confirm_frames=self.clear_confirm_frames,
            scan_timeout_sec=self.scan_timeout_sec,
            stop_on_scan_timeout=self.stop_on_scan_timeout,
            enable_ttc_stop=self.enable_ttc_stop,
            ttc_stop_sec=self.ttc_stop_sec,
        ))

        self.create_subscription(
            LaserScan, self.front_scan_topic, self._front_scan_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.rear_scan_topic, self._rear_scan_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.safety_scan_topic, self._safety_scan_cb,
            qos_profile_sensor_data)
        if self.encoder_speed_topic:
            # The only encoder-related interface found in the workspace is the
            # std_msgs/Float32 /ego_speed_mps input configured by the read-only
            # lidar_motion_detector package.  Runtime graph verification is still
            # required before encoder_source_verified may safely be enabled.
            self.create_subscription(
                Float32, self.encoder_speed_topic, self._encoder_speed_cb,
                qos_profile_sensor_data)
        self.create_subscription(
            Float32, '/parking/test_drive_request',
            self._test_drive_request_cb, 10)
        self.create_subscription(String, '/mission/state', self._mission_cb, 10)
        self.create_subscription(Bool, '/parking/t_mode', self._t_mode_cb, 10)
        self.create_subscription(
            Bool, '/parking/paring_mode_request', self._parallel_mode_cb, 10)
        self.create_subscription(Bool, '/parking/exit_request', self._exit_cb, 10)
        if self.localization_mode == 'odom':
            self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        reliable = 10
        self.state_pub = self.create_publisher(
            String, '/parking/planner_state', reliable)
        self.slot_pub = self.create_publisher(
            String, '/parking/selected_slot', reliable)
        self.space_pub = self.create_publisher(Bool, '/parking/space_found', reliable)
        self.drive_pub = self.create_publisher(
            Float32, '/parking/drive_cmd_mps', reliable)
        self.steering_pub = self.create_publisher(
            Float32, '/parking/steering_cmd_deg', reliable)
        self.done_pub = self.create_publisher(Bool, '/parking/done', reliable)
        self.fail_pub = self.create_publisher(
            Bool, '/parking/fail_safe_stop', reliable)
        self.status_text_pub = self.create_publisher(
            String, '/parking/status_text', reliable)
        self.status_json_pub = self.create_publisher(
            String, '/parking/status_json', reliable)
        self.slot_markers_pub = self.create_publisher(
            MarkerArray, '/parking/slot_markers', reliable)
        self.trajectory_marker_pub = self.create_publisher(
            MarkerArray, '/parking/trajectory_marker', reliable)
        self.safety_marker_pub = self.create_publisher(
            MarkerArray, '/parking/safety_margin_marker', reliable)
        self.lidar_safety_status_pub = self.create_publisher(
            String, '/parking/safety/status', reliable)
        self.lidar_safety_obstacle_pub = self.create_publisher(
            Bool, '/parking/safety/obstacle_detected', reliable)
        self.lidar_safety_distance_pub = self.create_publisher(
            Float32, '/parking/safety/front_min_distance', reliable)
        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray, '/parking/obstacles/markers', reliable)
        self.obstacle_tracks_pub = self.create_publisher(
            String, '/parking/obstacles/tracks_json', reliable)
        self.obstacle_nearest_distance_pub = self.create_publisher(
            Float32, '/parking/obstacles/nearest_distance', reliable)
        self.obstacle_nearest_type_pub = self.create_publisher(
            String, '/parking/obstacles/nearest_type', reliable)
        self.cmd_drive_pub = self.create_publisher(
            CMD_DRIVE_MSG_TYPE, self.cmd_drive_topic, reliable)
        self.cmd_wheel_pub = self.create_publisher(
            CMD_WHEEL_MSG_TYPE, self.cmd_wheel_topic, reliable)
        self.test_requested_drive_pub = self.create_publisher(
            Float32, '/parking/test/requested_drive', reliable)
        self.test_final_drive_pub = self.create_publisher(
            Float32, '/parking/test/final_drive', reliable)

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'Parking planner started: localization_mode={self.localization_mode}, '
            f'initial_state={self.state}; encoder_topic={self.encoder_speed_topic or "disabled"}, '
            f'encoder_source_verified={self.encoder_source_verified}')
        if not self.encoder_source_verified:
            self.get_logger().warning(
                'Encoder source/sign has not been verified on the live ROS graph; '
                'obstacle motion classification will remain UNKNOWN')
        if self.constant_drive_test_enabled:
            self.get_logger().warning('[CONSTANT DRIVE TEST] ENABLED')

    def _declare_parameters(self):
        defaults = {
            'localization_mode': 'scan_only',
            'require_odom_for_reverse': False,
            'base_frame': 'base_link',
            'front_scan_topic': '/front/scan',
            'rear_scan_topic': '/rear/scan',
            'odom_topic': '/odom',
            'safety_margin_m': 0.5,
            'max_steering_deg': 45.0,
            'forward_scan_speed_mps': 0.20,
            'forward_align_speed_mps': 0.15,
            'reverse_speed_mps': -0.15,
            'exit_speed_mps': 0.20,
            't_reverse_steering_deg': -30.0,
            'parallel_phase1_steering_deg': -35.0,
            'parallel_phase2_steering_deg': 30.0,
            'exit_right_steering_deg': -30.0,
            'parallel_phase1_duration_sec': 2.0,
            'parallel_phase2_duration_sec': 2.0,
            'parallel_phase3_duration_sec': 1.0,
            'rear_done_distance_m': 0.75,
            'min_clearance_m': 0.5,
            'scan_timeout_sec': 0.5,
            'tf_timeout_sec': 0.05,
            'odom_timeout_sec': 0.5,
            'slot_min_depth_m': 1.0,
            'slot_max_points': 3,
            'slot_confirm_frames': 3,
            'slot_a_x_min': 0.0,
            'slot_a_x_max': 1.0,
            'slot_b_x_min': -1.0,
            'slot_b_x_max': 0.0,
            'slot_right_y_min': -2.0,
            'slot_right_y_max': -1.0,
            'front_curb_min_m': 0.5,
            'front_curb_max_m': 1.0,
            'backup_ready_stop_sec': 0.5,
            'parallel_align_duration_sec': 1.0,
            'cmd_drive_topic': 'cmd_drive',
            'cmd_wheel_topic': 'cmd_wheel',
            'safety_scan_topic': '/scan',
            'stop_distance_m': 0.50,
            'resume_distance_m': 0.60,
            'front_sector_deg': 30.0,
            'min_valid_range_m': 0.05,
            'stop_confirm_scans': 2,
            'clear_confirm_scans': 3,
            'stop_on_scan_timeout': True,
            'encoder_speed_topic': '/ego_speed_mps',
            'encoder_speed_unit': 'mps',
            'encoder_forward_sign': 1.0,
            'encoder_source_verified': False,
            'encoder_timeout_sec': 0.5,
            'constant_drive_test_enabled': False,
            'test_command_timeout_sec': 0.5,
            'max_steering_for_motion_classification_deg': 5.0,
            'cluster_distance_threshold_m': 0.15,
            'min_cluster_points': 3,
            'association_distance_m': 0.35,
            'association_angle_deg': 10.0,
            'track_timeout_sec': 0.5,
            'min_dt_sec': 0.02,
            'max_dt_sec': 0.5,
            'velocity_filter_alpha': 0.35,
            'max_reasonable_object_speed_mps': 5.0,
            'static_speed_threshold_mps': 0.15,
            'dynamic_speed_threshold_mps': 0.30,
            'classification_confirm_frames': 3,
            'static_stop_distance_m': 0.50,
            'dynamic_stop_distance_m': 0.50,
            'unknown_stop_distance_m': 0.50,
            'static_resume_distance_m': 0.60,
            'dynamic_resume_distance_m': 0.60,
            'unknown_resume_distance_m': 0.60,
            'stop_confirm_frames': 2,
            'clear_confirm_frames': 3,
            'enable_ttc_stop': False,
            'ttc_stop_sec': 1.5,
            'front_scan_yaw_offset_deg': 180.0,
            'rear_scan_yaw_offset_deg': 0.0,
            'marker_yaw_offset_deg': 180.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        names = [parameter.name for parameter in self._parameters.values()]
        for name in names:
            setattr(self, name, self.get_parameter(name).value)
        self.safety_margin_m = max(0.5, float(self.safety_margin_m))
        self.min_clearance_m = max(0.5, float(self.min_clearance_m))
        self.max_steering_deg = min(45.0, abs(float(self.max_steering_deg)))
        self.forward_scan_speed_mps = self._clamp(
            self.forward_scan_speed_mps, 0.0, 0.25)
        self.forward_align_speed_mps = self._clamp(
            self.forward_align_speed_mps, 0.0, 0.25)
        self.exit_speed_mps = self._clamp(self.exit_speed_mps, 0.0, 0.25)
        self.reverse_speed_mps = self._clamp(self.reverse_speed_mps, -0.20, 0.0)
        if self.encoder_speed_unit not in ('mps', 'kmph'):
            self.get_logger().warning(
                f'Unsupported encoder_speed_unit={self.encoder_speed_unit}; using mps')
            self.encoder_speed_unit = 'mps'
        self.encoder_forward_sign = 1.0 if float(self.encoder_forward_sign) >= 0.0 else -1.0

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, float(value)))

    def _mission_cb(self, msg):
        mission = msg.data.strip().upper()
        self.mission_t_mode = mission == 'T_PARKING'
        self.mission_parallel_mode = mission == 'PARALLEL_PARKING'
        self._refresh_mode_requests()

    def _t_mode_cb(self, msg):
        self.t_mode_request = msg.data
        self._refresh_mode_requests()

    def _parallel_mode_cb(self, msg):
        self.parallel_mode_request = msg.data
        self._refresh_mode_requests()

    def _refresh_mode_requests(self):
        self.t_mode = self.mission_t_mode or self.t_mode_request
        self.parallel_mode = self.mission_parallel_mode or self.parallel_mode_request

    def _exit_cb(self, msg):
        self.exit_requested = msg.data

    def _odom_cb(self, _msg):
        self.odom_stamp = self.get_clock().now()

    def _encoder_speed_cb(self, msg):
        value = float(msg.data)
        if not math.isfinite(value):
            self.get_logger().warning('Ignoring non-finite encoder speed sample')
            return
        if self.encoder_speed_unit == 'kmph':
            value /= 3.6
        self.ego_speed_mps = self.encoder_forward_sign * value
        self.encoder_stamp = self.get_clock().now()

    def _test_drive_request_cb(self, msg):
        value = float(msg.data)
        if not math.isfinite(value):
            self.get_logger().warning('Ignoring non-finite test drive request')
            return
        self.test_drive_request = value
        self.test_drive_request_stamp = self.get_clock().now()
        if self.constant_drive_test_enabled and not self._test_request_logged:
            self.get_logger().warning(
                f'Requested drive command: {self.test_drive_request:.3f}')
            self._test_request_logged = True

    @staticmethod
    def _select_requested_drive(
            nominal_drive, test_enabled, test_drive_request,
            test_request_age_sec, test_command_timeout_sec):
        """Select the pre-safety drive request, failing closed on test timeout."""
        if not test_enabled:
            return float(nominal_drive)
        if test_request_age_sec > max(0.0, float(test_command_timeout_sec)):
            return 0.0
        return float(test_drive_request)

    def _front_scan_cb(self, msg):
        self.front_stamp = self.get_clock().now()
        self.front_range_min = self._minimum_valid_range(msg)
        points = self._transform_scan(msg, math.radians(self.front_scan_yaw_offset_deg))
        if points is None:
            self.front_tf_ok = False
            return
        self.front_points = points
        self.front_tf_ok = True
        if self.state in (self.T_SCAN, self.P_SCAN):
            self._update_slot_selection()

    def _rear_scan_cb(self, msg):
        self.rear_stamp = self.get_clock().now()
        self.rear_range_min = self._minimum_valid_range(msg)
        points = self._transform_scan(msg, math.radians(self.rear_scan_yaw_offset_deg))
        if points is None:
            self.rear_tf_ok = False
            return
        self.rear_points = points
        self.rear_tf_ok = True

    def _safety_scan_cb(self, msg):
        self.safety_scan_stamp = self.get_clock().now()
        previous, current = self.lidar_safety.update_scan(msg)
        self._log_lidar_safety_transition(previous, current)
        encoder_valid = self._encoder_valid()
        motion_classification_allowed = (
            abs(self.last_commanded_steering_deg)
            <= self.max_steering_for_motion_classification_deg)
        timestamp = self.safety_scan_stamp.nanoseconds / 1e9
        self.obstacle_tracks = self.obstacle_tracker.update(
            msg, timestamp, self.ego_speed_mps, encoder_valid,
            motion_classification_allowed)
        previous, current = self.obstacle_safety.update(self.obstacle_tracks)
        if previous != current and current == ObstacleSafetyGate.OBSTACLE_STOP:
            self.get_logger().warning(
                '[SAFETY STOP] Tracked front obstacle: '
                f'{self.obstacle_safety.nearest_distance:.2f} m '
                f'({self.obstacle_safety.nearest_type})')
        self._publish_obstacle_diagnostics(
            msg.header, encoder_valid, motion_classification_allowed)

    def _encoder_valid(self):
        return bool(
            self.encoder_source_verified
            and self.encoder_stamp is not None
            and self._age(self.encoder_stamp) <= self.encoder_timeout_sec)

    def _combined_lidar_safety_state(self):
        states = (self.lidar_safety.state, self.obstacle_safety.state)
        if LidarSafetyGate.LIDAR_TIMEOUT in states:
            return LidarSafetyGate.LIDAR_TIMEOUT
        if LidarSafetyGate.WAIT_SCAN in states:
            return LidarSafetyGate.WAIT_SCAN
        if LidarSafetyGate.OBSTACLE_STOP in states:
            return LidarSafetyGate.OBSTACLE_STOP
        return LidarSafetyGate.CLEAR

    def _publish_obstacle_diagnostics(
            self, header, encoder_valid, motion_classification_allowed):
        payload = {
            'ego_speed_mps': round(self.ego_speed_mps, 4) if encoder_valid else None,
            'encoder_valid': encoder_valid,
            'encoder_source_verified': bool(self.encoder_source_verified),
            'motion_classification_allowed': motion_classification_allowed,
            'encoder_age_sec': (
                None if self.encoder_stamp is None
                else round(self._age(self.encoder_stamp), 4)),
            'objects': [track_to_dict(track) for track in self.obstacle_tracks],
        }
        self.obstacle_tracks_pub.publish(String(
            data=json.dumps(payload, separators=(',', ':'), allow_nan=False)))
        self.obstacle_nearest_distance_pub.publish(Float32(
            data=float(self.obstacle_safety.nearest_distance)))
        self.obstacle_nearest_type_pub.publish(String(
            data=self.obstacle_safety.nearest_type))

        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        colors = {
            STATIC: (0.1, 0.9, 0.1),
            DYNAMIC_APPROACHING: (1.0, 0.1, 0.1),
            DYNAMIC_RECEDING: (0.1, 0.4, 1.0),
        }
        for index, track in enumerate(self.obstacle_tracks):
            red, green, blue = colors.get(
                track.classification, (1.0, 0.75, 0.1))
            body = Marker()
            body.header = header
            body.ns = 'tracked_obstacles'
            body.id = 2 * index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = track.center_x
            body.pose.position.y = track.center_y
            body.pose.position.z = 0.10
            body.pose.orientation.w = 1.0
            body.scale.x = max(0.08, track.width)
            body.scale.y = max(0.08, track.width)
            body.scale.z = 0.20
            body.color.r = red
            body.color.g = green
            body.color.b = blue
            body.color.a = 0.85
            markers.append(body)

            text = Marker()
            text.header = header
            text.ns = 'tracked_obstacle_labels'
            text.id = 2 * index + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = track.center_x
            text.pose.position.y = track.center_y
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = red
            text.color.g = green
            text.color.b = blue
            text.color.a = 1.0
            speed = track.filtered_object_speed
            speed_text = 'n/a' if speed is None else f'{speed:.2f} m/s'
            text.text = (
                f'{track.track_id}\n{track.classification}\n'
                f'{track.front_min_range:.2f} m  {speed_text}')
            markers.append(text)
        self.obstacle_markers_pub.publish(MarkerArray(markers=markers))

    def _log_lidar_safety_transition(self, previous, current):
        if previous == current:
            return
        distance = self.lidar_safety.front_min_distance
        if current == LidarSafetyGate.OBSTACLE_STOP:
            self.get_logger().warning(
                f'[SAFETY STOP] Front obstacle detected: {distance:.2f} m '
                f'<= {self.lidar_safety.stop_distance_m:.2f} m')
        elif previous == LidarSafetyGate.OBSTACLE_STOP and current == LidarSafetyGate.CLEAR:
            self.get_logger().info(
                f'[SAFETY CLEAR] Front obstacle cleared: {distance:.2f} m')
        elif current == LidarSafetyGate.LIDAR_TIMEOUT:
            self.get_logger().warning('[SAFETY STOP] LaserScan timeout')
        elif previous == LidarSafetyGate.LIDAR_TIMEOUT and current == LidarSafetyGate.CLEAR:
            self.get_logger().info(
                f'[SAFETY CLEAR] LaserScan restored: {distance:.2f} m')

    @staticmethod
    def _minimum_valid_range(scan):
        valid = [
            value for value in scan.ranges
            if math.isfinite(value) and scan.range_min <= value <= scan.range_max
        ]
        return min(valid, default=math.inf)

    def _transform_scan(self, scan, yaw_offset=0.0):
        source = scan.header.frame_id
        if not source:
            self.fail_reason = 'scan_frame_empty'
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=float(self.tf_timeout_sec)))
        except TransformException as error:
            self.fail_reason = f'tf_error:{source}:{error}'
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                x = distance * math.cos(angle + yaw_offset)
                y = distance * math.sin(angle + yaw_offset)
                points.append(self._rotate_translate(x, y, rotation, translation))
            angle += scan.angle_increment
        return points

    @staticmethod
    def _rotate_translate(x, y, q, t):
        # First two columns of the 3-D quaternion rotation matrix, with z=0.
        rx = (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * x
        rx += 2.0 * (q.x * q.y - q.z * q.w) * y
        ry = 2.0 * (q.x * q.y + q.z * q.w) * x
        ry += (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * y
        return (rx + t.x, ry + t.y)

    def _update_slot_selection(self):
        points = self.front_points + self.rear_points
        a_count = self._count_in_box(
            points, self.slot_a_x_min, self.slot_a_x_max,
            self.slot_right_y_min, self.slot_right_y_max)
        b_count = self._count_in_box(
            points, self.slot_b_x_min, self.slot_b_x_max,
            self.slot_right_y_min, self.slot_right_y_max)
        depth_ok = abs(self.slot_right_y_max - self.slot_right_y_min) >= self.slot_min_depth_m
        if depth_ok and a_count <= self.slot_max_points:
            candidate = 'A'
        elif depth_ok and b_count <= self.slot_max_points:
            candidate = 'B'
        else:
            candidate = 'blocked_all'
        if candidate == self.slot_candidate:
            self.slot_confirm_count += 1
        else:
            self.slot_candidate = candidate
            self.slot_confirm_count = 1
        if self.slot_confirm_count >= int(self.slot_confirm_frames):
            self.selected_slot = candidate

    @staticmethod
    def _count_in_box(points, x_min, x_max, y_min, y_max):
        return sum(
            1 for x, y in points
            if x_min <= x <= x_max and y_min <= y <= y_max)

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _state_age(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds / 1e9

    def _set_state(self, state, reason=''):
        if state == self.state:
            return
        self.get_logger().info(f'Parking state: {self.state} -> {state} ({reason})')
        self.state = state
        self.state_entered = self.get_clock().now()
        self.fail_reason = reason if state in (self.FAIL_SAFE_STOP, self.WAIT_ODOM) else ''

    def _active(self):
        return self.state != self.IDLE

    def _minimum_clearance(self):
        points = self.front_points + self.rear_points
        return min((math.hypot(x, y) for x, y in points), default=math.inf)

    def _front_curb_ready(self):
        distances = [
            math.hypot(x, y) for x, y in self.front_points
            if x >= 0.0 and y <= 0.25
        ]
        return any(
            self.front_curb_min_m <= distance <= self.front_curb_max_m
            for distance in distances)

    def _safety_reason(self, drive):
        if not self._active() or self.state in (self.T_DONE, self.P_DONE):
            return ''
        if drive > 0.0 and self._age(self.front_stamp) > self.scan_timeout_sec:
            return 'front_scan_timeout'
        if drive < 0.0 and self._age(self.rear_stamp) > self.scan_timeout_sec:
            return 'rear_scan_timeout'
        if not self.front_tf_ok or not self.rear_tf_ok:
            return 'tf_unavailable'
        if self._minimum_clearance() < self.min_clearance_m:
            return 'clearance_below_limit'
        if drive < 0.0 and self.selected_slot in ('unknown', 'blocked_all'):
            return f'reverse_blocked_slot_{self.selected_slot}'
        if drive < 0.0 and self.localization_mode == 'odom':
            if self._age(self.odom_stamp) > self.odom_timeout_sec:
                return 'odom_timeout'
        return ''

    def _nominal_command(self):
        if self.state in (self.T_SCAN, self.P_SCAN):
            return self.forward_scan_speed_mps, 0.0
        if self.state == self.T_READY:
            return 0.0, 0.0
        if self.state == self.T_REVERSE:
            return self.reverse_speed_mps, self.t_reverse_steering_deg
        if self.state == self.P_ALIGN:
            return self.forward_align_speed_mps, 0.0
        if self.state == self.P_REVERSE:
            age = self._state_age()
            if age < self.parallel_phase1_duration_sec:
                return self.reverse_speed_mps, self.parallel_phase1_steering_deg
            if age < self.parallel_phase1_duration_sec + self.parallel_phase2_duration_sec:
                return max(-0.20, self.reverse_speed_mps * 0.8), self.parallel_phase2_steering_deg
            return max(-0.20, self.reverse_speed_mps * (2.0 / 3.0)), 0.0
        if self.state in (self.T_EXIT, self.P_EXIT):
            return self.exit_speed_mps, self.exit_right_steering_deg
        return 0.0, 0.0

    def _advance_state(self):
        if self.state == self.IDLE:
            if (self.localization_mode == 'odom'
                    and self._age(self.odom_stamp) > self.odom_timeout_sec):
                self._set_state(self.WAIT_ODOM, 'waiting_for_odom')
                return
            if self.t_mode:
                self._start_mode(self.T_SCAN)
            elif self.parallel_mode:
                self._start_mode(self.P_SCAN)
            return

        if self.state == self.FAIL_SAFE_STOP:
            # A fail-safe is latched until the mission source is cleared. Reasserting
            # the mission after correcting the fault starts a fresh, conservative scan.
            if not self.t_mode and not self.parallel_mode:
                self._set_state(self.IDLE, 'mission_cleared_after_fail_safe')
            return

        if self.state == self.WAIT_ODOM:
            if self._age(self.odom_stamp) > self.odom_timeout_sec:
                return
            if self.t_mode or self.parallel_mode:
                self._set_state(self.T_SCAN if self.t_mode else self.P_SCAN, 'odom_ready')
            else:
                self._set_state(self.IDLE, 'odom_ready_mission_inactive')
            return

        if self.state == self.T_SCAN and self.selected_slot in ('A', 'B'):
            if self._front_curb_ready():
                self._set_state(self.T_READY, 'slot_and_front_curb_confirmed')
        elif self.state == self.T_READY:
            if self._state_age() >= self.backup_ready_stop_sec:
                self._set_state(self.T_REVERSE, 'ready_stop_complete')
        elif self.state == self.T_REVERSE:
            if self.rear_range_min <= self.rear_done_distance_m:
                self._set_state(self.T_DONE, 'rear_done_distance')
        elif self.state == self.T_DONE and self.exit_requested:
            self.exit_requested = False
            self._set_state(self.T_EXIT, 'exit_requested')
        elif self.state == self.P_SCAN and self.selected_slot in ('A', 'B'):
            self._set_state(self.P_ALIGN, 'slot_confirmed')
        elif self.state == self.P_ALIGN:
            if self._state_age() >= self.parallel_align_duration_sec or self._front_curb_ready():
                self._set_state(self.P_REVERSE, 'align_condition_met')
        elif self.state == self.P_REVERSE:
            if self.rear_range_min <= self.rear_done_distance_m:
                self._set_state(self.P_DONE, 'rear_done_distance')
            elif self._state_age() >= (
                    self.parallel_phase1_duration_sec
                    + self.parallel_phase2_duration_sec
                    + self.parallel_phase3_duration_sec):
                self._set_state(
                    self.FAIL_SAFE_STOP,
                    'parallel_reverse_phases_complete_without_rear_confirmation')
        elif self.state == self.P_DONE and self.exit_requested:
            self.exit_requested = False
            self._set_state(self.P_EXIT, 'exit_requested')

    def _start_mode(self, scan_state):
        self.selected_slot = 'unknown'
        self.slot_candidate = 'unknown'
        self.slot_confirm_count = 0
        self.fail_reason = ''
        if self.localization_mode == 'odom' and self._age(self.odom_stamp) > self.odom_timeout_sec:
            self._set_state(self.WAIT_ODOM, 'waiting_for_odom')
        else:
            self._set_state(scan_state, 'mission_requested')

    def _tick(self):
        previous, current = self.lidar_safety.update_timeout(
            self._age(self.safety_scan_stamp))
        self._log_lidar_safety_transition(previous, current)
        self.obstacle_safety.update_timeout(self._age(self.safety_scan_stamp))

        self._advance_state()
        drive, steering = self._nominal_command()
        reason = self._safety_reason(drive)
        if reason and self.state not in (self.FAIL_SAFE_STOP, self.WAIT_ODOM):
            self._set_state(self.FAIL_SAFE_STOP, reason)
            drive, steering = 0.0, 0.0
        if self.state in (self.FAIL_SAFE_STOP, self.WAIT_ODOM):
            drive, steering = 0.0, 0.0
        if self.state in (self.T_EXIT, self.P_EXIT) and self.front_range_min <= 0.5:
            drive = 0.0
        drive = self._clamp(drive, -0.20, 0.25)
        steering = self._clamp(steering, -self.max_steering_deg, self.max_steering_deg)
        requested_drive = self._select_requested_drive(
            drive,
            self.constant_drive_test_enabled,
            self.test_drive_request,
            self._age(self.test_drive_request_stamp),
            self.test_command_timeout_sec,
        )
        drive = requested_drive
        self.last_commanded_steering_deg = steering

        # Final drive-only safety gate. Steering remains owned by the existing
        # parking state machine, even while the requested drive is clamped to 0.
        drive, steering = self.lidar_safety.filter_command(drive, steering)
        drive = self.obstacle_safety.filter_drive(drive)

        self._publish_constant_drive_test_status(requested_drive, drive)
        self._publish(drive, steering)
        self.cmd_drive_pub.publish(CMD_DRIVE_MSG_TYPE(data=float(drive)))
        self.cmd_wheel_pub.publish(CMD_WHEEL_MSG_TYPE(data=float(steering)))

    def _publish_constant_drive_test_status(self, requested_drive, final_drive):
        if not self.constant_drive_test_enabled:
            return
        self.test_requested_drive_pub.publish(Float32(data=float(requested_drive)))
        self.test_final_drive_pub.publish(Float32(data=float(final_drive)))

        report = (float(requested_drive), float(final_drive))
        if self.test_drive_request_stamp is None:
            return
        if self._last_test_drive_report is not None:
            unchanged = all(
                math.isclose(current, previous, abs_tol=1e-6)
                for current, previous in zip(report, self._last_test_drive_report))
            if unchanged:
                return
        self.get_logger().warning(
            f'[CONSTANT DRIVE TEST] requested={requested_drive:.3f} '
            f'final={final_drive:.3f}')
        self._last_test_drive_report = report

    def _publish(self, drive, steering):
        done = self.state in (self.T_DONE, self.P_DONE)
        failed = self.state in (self.FAIL_SAFE_STOP, self.WAIT_ODOM)
        space_found = self.selected_slot in ('A', 'B')

        self.state_pub.publish(String(data=self.state))
        self.slot_pub.publish(String(data=self.selected_slot))
        self.space_pub.publish(Bool(data=space_found))
        self.drive_pub.publish(Float32(data=float(drive)))
        self.steering_pub.publish(Float32(data=float(steering)))
        self.done_pub.publish(Bool(data=done))
        self.fail_pub.publish(Bool(data=failed))
        combined_safety_state = self._combined_lidar_safety_state()
        self.lidar_safety_status_pub.publish(String(data=combined_safety_state))
        self.lidar_safety_obstacle_pub.publish(
            Bool(data=(
                self.lidar_safety.obstacle_detected
                or self.obstacle_safety.obstacle_detected)))
        nearest_distance = min(
            self.lidar_safety.front_min_distance,
            self.obstacle_safety.nearest_distance)
        self.lidar_safety_distance_pub.publish(
            Float32(data=float(nearest_distance)))

        clearance = self._minimum_clearance()
        status = (
            f'state={self.state} slot={self.selected_slot} '
            f'drive={drive:.2f} steering={steering:.1f} '
            f'clearance={clearance:.2f} reason={self.fail_reason or "none"}')
        self.status_text_pub.publish(String(data=status))
        payload = {
            'state': self.state,
            'localization_mode': self.localization_mode,
            'selected_slot': self.selected_slot,
            'space_found': space_found,
            'drive_cmd_mps': round(drive, 3),
            'steering_cmd_deg': round(steering, 3),
            'done': done,
            'fail_safe_stop': failed,
            'fail_reason': self.fail_reason,
            'min_clearance_m': None if math.isinf(clearance) else round(clearance, 3),
            'lidar_safety_state': combined_safety_state,
            'front_min_distance_m': (
                None if math.isinf(nearest_distance)
                else round(nearest_distance, 3)),
            'nearest_obstacle_type': self.obstacle_safety.nearest_type,
            'ego_speed_mps': round(self.ego_speed_mps, 3)
            if self._encoder_valid() else None,
            'encoder_valid': self._encoder_valid(),
            'constant_drive_test_enabled': bool(
                self.constant_drive_test_enabled),
            'front_scan_age_sec': round(self._age(self.front_stamp), 3),
            'rear_scan_age_sec': round(self._age(self.rear_stamp), 3),
            'odom_age_sec': (
                None if self.odom_stamp is None
                else round(self._age(self.odom_stamp), 3)),
        }
        self.status_json_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self._publish_markers(drive, steering)

    def _rotate_marker_xy(self, x, y):
        c = math.cos(math.radians(self.marker_yaw_offset_deg))
        s = math.sin(math.radians(self.marker_yaw_offset_deg))
        return (c * x - s * y, s * x + c * y)

    def _marker(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_markers(self, drive, steering):
        slots = MarkerArray()
        for marker_id, name, x_min, x_max in (
                (0, 'A', self.slot_a_x_min, self.slot_a_x_max),
                (1, 'B', self.slot_b_x_min, self.slot_b_x_max)):
            marker = self._marker(marker_id, 'parking_slots', Marker.CUBE)
            slot_x, slot_y = self._rotate_marker_xy(
                (x_min + x_max) / 2.0,
                (self.slot_right_y_min + self.slot_right_y_max) / 2.0)
            marker.pose.position.x = slot_x
            marker.pose.position.y = slot_y
            marker.scale.x = max(0.01, x_max - x_min)
            marker.scale.y = max(0.01, self.slot_right_y_max - self.slot_right_y_min)
            marker.scale.z = 0.03
            marker.color.a = 0.35
            marker.color.g = 1.0 if self.selected_slot == name else 0.3
            marker.color.r = 0.2 if self.selected_slot == name else 0.8
            slots.markers.append(marker)
        self.slot_markers_pub.publish(slots)

        trajectory = self._marker(0, 'parking_trajectory', Marker.LINE_STRIP)
        trajectory.scale.x = 0.05
        trajectory.color.a = 0.9
        trajectory.color.b = 1.0
        for index in range(11):
            distance = drive * index * 0.5
            heading = math.radians(steering) * index / 10.0
            point_x, point_y = self._rotate_marker_xy(
                distance * math.cos(heading), distance * math.sin(heading))
            trajectory.points.append(Point(x=point_x, y=point_y, z=0.05))
        self.trajectory_marker_pub.publish(MarkerArray(markers=[trajectory]))

        safety = self._marker(0, 'parking_safety_margin', Marker.CYLINDER)
        safety_x, safety_y = self._rotate_marker_xy(
            safety.pose.position.x, safety.pose.position.y)
        safety.pose.position.x = safety_x
        safety.pose.position.y = safety_y
        safety.scale.x = 2.0 * self.safety_margin_m
        safety.scale.y = 2.0 * self.safety_margin_m
        safety.scale.z = 0.03
        safety.color.a = 0.25
        safety.color.r = 1.0
        self.safety_marker_pub.publish(MarkerArray(markers=[safety]))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

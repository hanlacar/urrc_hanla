#!/usr/bin/env python3
"""ROS node that detects and remembers parking spaces in the map frame."""

import json
import math
import os
import zlib

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .parking_space_detector import (
    DetectorConfig,
    GridMap,
    MapMetadata,
    ParkingCandidate,
    ParkingSpaceDetector,
    normalize_angle,
)
from .slot_tracker import (
    SlotTracker,
    TrackerConfig,
    load_slots_json,
    map_metadata_compatible,
    save_slots_json,
)


class ParkingSpaceMemoryNode(Node):
    WAIT_MAP = 'WAIT_MAP'
    WAIT_SCAN = 'WAIT_SCAN'
    WAIT_ODOM_TF = 'WAIT_ODOM_TF'
    WAIT_LIDAR_TF = 'WAIT_LIDAR_TF'
    MAPPING = 'MAPPING'
    MEMORY_ACTIVE = 'MEMORY_ACTIVE'
    ERROR = 'ERROR'

    def __init__(self):
        super().__init__('parking_space_memory_node')
        self._declare_parameters()
        self._read_parameters()

        self.detector = ParkingSpaceDetector(DetectorConfig(
            occupied_threshold=int(self.occupied_threshold),
            free_threshold=int(self.free_threshold),
            min_free_ratio=float(self.min_free_ratio),
            max_unknown_ratio=float(self.max_unknown_ratio),
            safety_margin_m=float(self.safety_margin_m),
            vehicle_length_m=float(self.vehicle_length_m),
            vehicle_width_m=float(self.vehicle_width_m),
            t_slot_width_m=float(self.t_slot_width_m),
            t_slot_depth_m=float(self.t_slot_depth_m),
            parallel_slot_length_m=float(self.parallel_slot_length_m),
            parallel_slot_width_m=float(self.parallel_slot_width_m),
            search_forward_m=float(self.search_forward_m),
            search_backward_m=float(self.search_backward_m),
            search_right_min_m=float(self.search_right_min_m),
            search_right_max_m=float(self.search_right_max_m),
            sample_step_m=float(self.sample_step_m),
            nms_distance_m=float(self.nms_distance_m),
            boundary_band_m=float(self.boundary_band_m),
            max_candidates_per_type=int(self.max_candidates_per_type),
        ))
        self.tracker = SlotTracker(TrackerConfig(
            association_distance_m=float(self.association_distance_m),
            association_yaw_deg=float(self.association_yaw_deg),
            association_size_tolerance_m=float(
                self.association_size_tolerance_m),
            confirm_observations=int(self.confirm_observations),
            lost_timeout_sec=float(self.lost_timeout_sec),
            delete_timeout_sec=float(self.delete_timeout_sec),
        ))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grid = None
        self.map_message_frame = ''
        self.scan_frame = ''
        self.last_scan_time = None
        self.state = self.WAIT_MAP
        self.state_detail = 'waiting for the first OccupancyGrid'
        self.mapping_ready = False
        self.robot_pose = None
        self._published_marker_slot_ids = set()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback,
            qos_profile_sensor_data)

        self.status_pub = self.create_publisher(
            String, '/parking/memory/status', 10)
        self.space_found_pub = self.create_publisher(
            Bool, '/parking/memory/space_found', 10)
        self.selected_slot_id_pub = self.create_publisher(
            String, '/parking/memory/selected_slot_id', 10)
        self.selected_slot_pose_pub = self.create_publisher(
            PoseStamped, '/parking/memory/selected_slot_pose', 10)
        self.slots_json_pub = self.create_publisher(
            String, '/parking/memory/slots_json', 10)
        self.slot_markers_pub = self.create_publisher(
            MarkerArray, '/parking/memory/slot_markers', marker_qos)

        self.create_service(
            Trigger, '/parking/memory/save', self._save_callback)
        self.create_service(
            Trigger, '/parking/memory/load', self._load_callback)
        self.create_service(
            Trigger, '/parking/memory/clear', self._clear_callback)

        self.timer = self.create_timer(
            max(0.1, float(self.detection_period_sec)), self._tick)
        self.get_logger().info(
            'Parking-space memory started; '
            f'map={self.map_topic}, scan={self.scan_topic}, '
            f'frames={self.map_frame}->{self.odom_frame}->{self.base_frame}. '
            'This node publishes no motion commands.')

    def _declare_parameters(self):
        defaults = {
            'map_topic': '/map',
            'scan_topic': '/front/scan',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'detection_period_sec': 0.5,
            'scan_timeout_sec': 2.0,
            'tf_timeout_sec': 0.05,
            'occupied_threshold': 65,
            'free_threshold': 25,
            'min_free_ratio': 0.85,
            'max_unknown_ratio': 0.15,
            'safety_margin_m': 0.30,
            'vehicle_length_m': 1.0,
            'vehicle_width_m': 0.6,
            't_slot_width_m': 1.2,
            't_slot_depth_m': 1.8,
            'parallel_slot_length_m': 2.0,
            'parallel_slot_width_m': 1.0,
            'search_forward_m': 4.0,
            'search_backward_m': 2.0,
            'search_right_min_m': 0.5,
            'search_right_max_m': 3.0,
            'sample_step_m': 0.40,
            'nms_distance_m': 0.75,
            'boundary_band_m': 0.20,
            'max_candidates_per_type': 12,
            'association_distance_m': 0.5,
            'association_yaw_deg': 15.0,
            'association_size_tolerance_m': 0.5,
            'confirm_observations': 5,
            'lost_timeout_sec': 10.0,
            'delete_timeout_sec': 60.0,
            'occupied_evidence_ratio': 0.02,
            'minimum_map_known_ratio': 0.001,
            'storage_path':
                '~/.ros/lidar_parking_planner/parking_slots.json',
            'map_resolution_tolerance': 0.001,
            'map_origin_tolerance_m': 0.5,
            'map_origin_yaw_tolerance_deg': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        for name in self._parameters:
            setattr(self, name, self.get_parameter(name).value)

    @staticmethod
    def _quaternion_yaw(quaternion) -> float:
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _yaw_quaternion(yaw: float):
        from geometry_msgs.msg import Quaternion
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    def _map_callback(self, message: OccupancyGrid):
        message_frame = message.header.frame_id or self.map_frame
        if message_frame != self.map_frame:
            self.grid = None
            self.map_message_frame = message_frame
            self._set_state(
                self.ERROR,
                f'OccupancyGrid frame is {message_frame}, expected {self.map_frame}',
                warning=True)
            return
        try:
            metadata = MapMetadata(
                resolution=float(message.info.resolution),
                width=int(message.info.width),
                height=int(message.info.height),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                origin_yaw=self._quaternion_yaw(
                    message.info.origin.orientation),
            )
            values = np.asarray(message.data, dtype=np.int16).reshape(
                (metadata.height, metadata.width))
            self.grid = GridMap(values, metadata)
            self.map_message_frame = message_frame
        except (TypeError, ValueError) as error:
            self.grid = None
            self._set_state(
                self.ERROR, f'invalid OccupancyGrid: {error}', warning=True)

    def _scan_callback(self, message: LaserScan):
        self.last_scan_time = self.get_clock().now()
        self.scan_frame = message.header.frame_id.strip()

    def _scan_age(self) -> float:
        if self.last_scan_time is None:
            return math.inf
        return (
            self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9

    def _lookup(self, target_frame: str, source_frame: str):
        return self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=float(self.tf_timeout_sec)),
        )

    def _set_state(self, state: str, detail: str, warning: bool = False):
        changed = state != self.state or detail != self.state_detail
        self.state = state
        self.state_detail = detail
        if not changed:
            return
        message = f'{state}: {detail}'
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _prerequisites(self):
        if self.grid is None:
            if self.state != self.ERROR:
                self._set_state(
                    self.WAIT_MAP, f'waiting for {self.map_topic}')
            return None
        if self.last_scan_time is None or self._scan_age() > float(
                self.scan_timeout_sec):
            self._set_state(
                self.WAIT_SCAN,
                f'waiting for a current scan on {self.scan_topic}')
            return None

        try:
            self._lookup(self.odom_frame, self.base_frame)
        except TransformException as error:
            self._set_state(
                self.WAIT_ODOM_TF,
                f'missing transform {self.odom_frame} -> '
                f'{self.base_frame}: {error}',
                warning=True)
            return None

        if not self.scan_frame:
            self._set_state(
                self.WAIT_LIDAR_TF,
                f'{self.scan_topic} has an empty header.frame_id',
                warning=True)
            return None
        try:
            self._lookup(self.base_frame, self.scan_frame)
        except TransformException as error:
            self._set_state(
                self.WAIT_LIDAR_TF,
                f'missing transform {self.base_frame} -> '
                f'{self.scan_frame}: {error}',
                warning=True)
            return None

        try:
            map_to_base = self._lookup(self.map_frame, self.base_frame)
        except TransformException as error:
            self._set_state(
                self.MAPPING,
                f'waiting for SLAM transform {self.map_frame} -> '
                f'{self.base_frame}: {error}')
            return None
        translation = map_to_base.transform.translation
        rotation = map_to_base.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            self._quaternion_yaw(rotation),
        )

    def _occupied_slot_ids(self):
        occupied = []
        for slot in self.tracker.slots:
            candidate = ParkingCandidate(
                parking_type=slot.parking_type,
                center_x=slot.center_x,
                center_y=slot.center_y,
                yaw=slot.yaw,
                width=slot.width,
                length=slot.length,
                confidence=slot.confidence,
            )
            if candidate.parking_type not in self.detector.VALID_TYPES:
                continue
            metrics = self.detector.evaluate_candidate(self.grid, candidate)
            if (
                    metrics.inside_map
                    and metrics.unknown_ratio <= float(self.max_unknown_ratio)
                    and metrics.occupied_ratio >= float(
                        self.occupied_evidence_ratio)):
                occupied.append(slot.slot_id)
        return occupied

    def _tick(self):
        try:
            robot_pose = self._prerequisites()
            if robot_pose is None:
                self.mapping_ready = False
                self.robot_pose = None
                self._publish_outputs()
                return
            self.robot_pose = robot_pose
            if self.grid.known_ratio() < float(self.minimum_map_known_ratio):
                self.mapping_ready = False
                self._set_state(
                    self.MAPPING,
                    'map is available but does not contain enough known cells')
                self._publish_outputs()
                return

            candidates = self.detector.detect(self.grid, *robot_pose)
            occupied = self._occupied_slot_ids()
            now = self.get_clock().now().nanoseconds / 1e9
            self.tracker.update(candidates, now, occupied)
            self.mapping_ready = True
            self._set_state(
                self.MEMORY_ACTIVE,
                f'{len(candidates)} observations, '
                f'{len(self.tracker.slots)} remembered slots')
            self._publish_outputs()
        except Exception as error:  # Keep the node alive and make faults observable.
            self.mapping_ready = False
            self.robot_pose = None
            self._set_state(
                self.ERROR,
                f'{type(error).__name__}: {error}',
                warning=True)
            self._publish_outputs()

    def _selected_slot(self):
        if not self.mapping_ready or self.robot_pose is None:
            return None
        confirmed = [
            slot for slot in self.tracker.slots
            if slot.status == 'CONFIRMED']
        if not confirmed:
            return None
        robot_x, robot_y, _ = self.robot_pose
        return min(
            confirmed,
            key=lambda slot: math.hypot(
                slot.center_x - robot_x, slot.center_y - robot_y))

    def _publish_outputs(self):
        selected = self._selected_slot()
        selected_id = selected.slot_id if selected is not None else ''
        self.status_pub.publish(String(
            data=f'{self.state}: {self.state_detail}'))
        self.space_found_pub.publish(Bool(data=selected is not None))
        self.selected_slot_id_pub.publish(String(data=selected_id))

        now_message = self.get_clock().now().to_msg()
        if selected is not None:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = now_message
            pose.pose.position.x = selected.center_x
            pose.pose.position.y = selected.center_y
            pose.pose.orientation = self._yaw_quaternion(selected.yaw)
            self.selected_slot_pose_pub.publish(pose)

        payload = {
            'frame_id': self.map_frame,
            'mapping_ready': bool(self.mapping_ready),
            'state': self.state,
            'state_detail': self.state_detail,
            'selected_slot_id': selected_id,
            'slots': [slot.to_dict() for slot in self.tracker.slots],
        }
        self.slots_json_pub.publish(String(
            data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))
        self._publish_markers(now_message)

    @staticmethod
    def _marker_base_id(slot_id: str) -> int:
        return (zlib.crc32(slot_id.encode('utf-8')) & 0x3fffffff) * 2

    def _delete_markers(self, slot_ids, stamp):
        markers = MarkerArray()
        for slot_id in sorted(slot_ids):
            base_id = self._marker_base_id(slot_id)
            for marker_id, namespace in (
                    (base_id, 'parking_memory_slots'),
                    (base_id + 1, 'parking_memory_labels')):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = stamp
                marker.ns = namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                markers.markers.append(marker)
        return markers

    def _publish_markers(self, stamp=None):
        stamp = stamp or self.get_clock().now().to_msg()
        current_ids = {slot.slot_id for slot in self.tracker.slots}
        markers = self._delete_markers(
            self._published_marker_slot_ids - current_ids, stamp)
        colors = {
            'CANDIDATE': (1.0, 0.65, 0.0, 0.25),
            'CONFIRMED': (0.05, 0.90, 0.20, 0.70),
            'TEMPORARILY_LOST': (0.55, 0.55, 0.55, 0.18),
            'OCCUPIED': (1.0, 0.05, 0.05, 0.75),
        }
        for slot in self.tracker.slots:
            base_id = self._marker_base_id(slot.slot_id)
            red, green, blue, alpha = colors.get(
                slot.status, colors['CANDIDATE'])

            rectangle = Marker()
            rectangle.header.frame_id = self.map_frame
            rectangle.header.stamp = stamp
            rectangle.ns = 'parking_memory_slots'
            rectangle.id = base_id
            rectangle.type = Marker.CUBE
            rectangle.action = Marker.ADD
            rectangle.pose.position.x = slot.center_x
            rectangle.pose.position.y = slot.center_y
            rectangle.pose.position.z = 0.03
            rectangle.pose.orientation = self._yaw_quaternion(slot.yaw)
            rectangle.scale.x = max(0.01, slot.length)
            rectangle.scale.y = max(0.01, slot.width)
            rectangle.scale.z = 0.05
            rectangle.color.r = red
            rectangle.color.g = green
            rectangle.color.b = blue
            rectangle.color.a = alpha
            markers.markers.append(rectangle)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'parking_memory_labels'
            label.id = base_id + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = slot.center_x
            label.pose.position.y = slot.center_y
            label.pose.position.z = 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color.r = red
            label.color.g = green
            label.color.b = blue
            label.color.a = max(0.45, alpha)
            label.text = (
                f'{slot.slot_id} {slot.parking_type} '
                f'{slot.confidence:.2f}')
            markers.markers.append(label)
        self.slot_markers_pub.publish(markers)
        self._published_marker_slot_ids = current_ids

    def _save_callback(self, _request, response):
        if self.grid is None:
            response.success = False
            response.message = 'No map metadata is available; nothing was saved.'
            return response
        try:
            destination = os.path.expanduser(str(self.storage_path))
            save_slots_json(
                destination, self.tracker.slots,
                self.map_frame, self.grid.metadata)
            response.success = True
            response.message = (
                f'Saved {len(self.tracker.slots)} slots to {destination}')
        except (OSError, TypeError, ValueError) as error:
            response.success = False
            response.message = f'Failed to save slots: {error}'
            self.get_logger().error(response.message)
        return response

    def _load_callback(self, _request, response):
        if self.grid is None:
            response.success = False
            response.message = (
                'A current map is required before stored coordinates can be loaded.')
            return response
        try:
            source = os.path.expanduser(str(self.storage_path))
            slots, payload = load_slots_json(source)
            if payload.get('map_frame') != self.map_frame:
                response.success = False
                response.message = (
                    f'Saved map_frame={payload.get("map_frame")} does not match '
                    f'current map_frame={self.map_frame}; slots were not loaded.')
                self.get_logger().warning(response.message)
                return response
            compatible, reason = map_metadata_compatible(
                payload,
                self.grid.metadata,
                float(self.map_resolution_tolerance),
                float(self.map_origin_tolerance_m),
                float(self.map_origin_yaw_tolerance_deg),
            )
            if not compatible:
                response.success = False
                response.message = (
                    f'Map metadata mismatch ({reason}); slots were not loaded.')
                self.get_logger().warning(response.message)
                return response
            now = self.get_clock().now().nanoseconds / 1e9
            self.tracker.replace_slots(slots, rebase_last_seen_time=now)
            response.success = True
            response.message = f'Loaded {len(slots)} slots from {source}'
            self._publish_outputs()
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            response.success = False
            response.message = f'Failed to load slots: {error}'
            self.get_logger().error(response.message)
        return response

    def _clear_callback(self, _request, response):
        deleted = self.tracker.clear()
        self._publish_outputs()
        response.success = True
        response.message = f'Cleared {len(deleted)} remembered slots'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceMemoryNode()
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

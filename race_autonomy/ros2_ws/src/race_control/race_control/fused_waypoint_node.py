"""Publish atomic mission waypoint state from GPS fixes and wheel odometry."""
import json
import math
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from .route_waypoints import load_waypoints
from .fused_route import FusedProgress, RouteGeometry, WaypointSchedule


class FusedWaypointNode(Node):
    def __init__(self):
        super().__init__('fused_waypoint_node')
        defaults = dict(route_csv='', segments_yaml='', gps_topic='/fix', odom_topic='/mcu/odom',
                        initial_index=0, ramp_first_index=-1, ramp_stop_index=-1,
                        front_offset_m=0.0, acceleration_start_index=-1, acceleration_end_index=-1,
                        gps_max_std_m=0.5, max_cross_track_m=0.75, innovation_m=1.5,
                        max_dr_sec=3.0, max_dr_m=2.0, max_std_m=0.5,
                        odom_timeout_sec=0.3, initial_window_m=3.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.route = RouteGeometry(load_waypoints(self.p('route_csv'), self.p('segments_yaml')))
        with Path(self.p('route_csv')).with_suffix('.yaml').open() as stream:
            meta = yaml.safe_load(stream)
        self.origin_lat, self.origin_lon = float(meta['origin_lat']), float(meta['origin_lon'])
        if not (-90 <= self.origin_lat <= 90 and -180 <= self.origin_lon <= 180):
            raise ValueError('invalid route GPS origin')
        self.reset()
        self.state_pub = self.create_publisher(String, '/mission/waypoint_state', 10)
        self.section_pub = self.create_publisher(Int8, '/mission/section', 10)
        self.valid_pub = self.create_publisher(Bool, '/mission/route_valid', 10)
        self.create_subscription(Odometry, self.p('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, self.p('gps_topic'), self.on_gps, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/waypoint_release', self.on_release, 10)
        self.create_service(Trigger, '/mission/waypoints/reset', self.on_reset)
        self.create_timer(0.05, self.publish)

    def p(self, key):
        return self.get_parameter(key).value

    def reset(self):
        self.session = uuid.uuid4().hex
        self.estimator = FusedProgress(self.route, **{k: self.p(k) for k in (
            'initial_index', 'innovation_m', 'max_cross_track_m', 'gps_max_std_m',
            'max_dr_sec', 'max_dr_m', 'max_std_m', 'odom_timeout_sec', 'initial_window_m')})
        self.schedule = WaypointSchedule(self.route, **{k: self.p(k) for k in (
            'initial_index', 'ramp_stop_index', 'ramp_first_index', 'front_offset_m',
            'acceleration_start_index', 'acceleration_end_index')})
        self.last_pose = None
        self.last_stamps = {}
        self.last_state = None

    def on_reset(self, request, response):
        self.reset()
        response.success = True
        response.message = 'new waypoint session; waiting for GPS and odometry'
        return response

    def fresh_stamp(self, key, header):
        stamp = header.stamp.sec+header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if not math.isfinite(stamp) or not -0.1 <= age <= 0.5:
            return False
        if stamp <= self.last_stamps.get(key, -float('inf')):
            return False
        self.last_stamps[key] = stamp
        return True

    def on_odom(self, msg):
        if not self.fresh_stamp('odom', msg.header):
            return
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (pos.x, pos.y, q.x, q.y, q.z, q.w)
        norm = sum(v*v for v in values[2:])
        if not all(math.isfinite(v) for v in values) or not 0.9 <= norm <= 1.1:
            self.estimator.fault = 'invalid odometry pose; reset required'
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        frame = (msg.header.frame_id, msg.child_frame_id)
        distance = 0.0
        if self.last_pose:
            x, y, previous_yaw, old_frame = self.last_pose
            if frame != old_frame:
                self.estimator.fault = 'odometry frame changed; reset required'
                return
            dx, dy = pos.x-x, pos.y-y
            distance = math.hypot(dx, dy)
            if dx*math.cos(previous_yaw)+dy*math.sin(previous_yaw) < 0:
                distance = -distance
        self.last_pose = (pos.x, pos.y, yaw, frame)
        direction = self.route.points[self.route.index(self.estimator.s)].direction
        self.estimator.predict(distance*direction, time.monotonic())

    def on_gps(self, msg):
        if not self.fresh_stamp('gps', msg.header):
            return
        if (msg.status.status < 0 or msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
                or not -90 <= msg.latitude <= 90 or not -180 <= msg.longitude <= 180):
            self.estimator.gps_reason = 'GPS fix/covariance unavailable'
            return
        # Local tangent approximation matching route x=east, y=north in metres.
        x = math.radians(msg.longitude-self.origin_lon)*6378137.0*math.cos(math.radians(self.origin_lat))
        y = math.radians(msg.latitude-self.origin_lat)*6378137.0
        covariance = (msg.position_covariance[0], msg.position_covariance[4])
        if any(not math.isfinite(v) or v < 0 for v in covariance):
            return
        self.estimator.correct(x, y, max(covariance), time.monotonic())

    def on_release(self, msg):
        state = self.last_state
        if not state or not state['valid'] or not state['event_id']:
            return
        valid, _ = self.estimator.validity(time.monotonic())
        if valid and msg.data == state['event_id']:
            self.schedule.release(state['stop_index'])

    def publish(self):
        valid, reason = self.estimator.validity(time.monotonic())
        state = self.schedule.snapshot(self.estimator.s)
        if state['section'] == 2 and not state['ramp_configured']:
            valid, reason = False, 'first/second ramp waypoint indices are not configured'
        if state['section'] == 9 and not state['acceleration_configured']:
            valid, reason = False, 'acceleration waypoint indices are not configured'
        state.update(valid=valid, reason=reason, session=self.session,
                     progress_m=self.estimator.s, std_m=math.sqrt(self.estimator.variance),
                     gps_reason=self.estimator.gps_reason,
                     event_id=f"{self.session}:{state['stop_index']}" if state['stop_index'] >= 0 else '')
        self.last_state = state
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))
        self.valid_pub.publish(Bool(data=valid))
        if valid:
            self.section_pub.publish(Int8(data=state['section']))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusedWaypointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

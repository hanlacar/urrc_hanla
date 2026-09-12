"""ROS same-process integration test; use a separate ROS_DOMAIN_ID (see docs)."""
import csv
import json
import math
import tempfile
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String
from race_control.fused_waypoint_node import FusedWaypointNode
from race_control.course_mission_node import CourseMissionNode
from race_perception.traffic_light_color_node import TrafficLightColorNode


def main():
    with tempfile.TemporaryDirectory(prefix='fused_waypoint_') as tmp:
        path = Path(tmp)/'route.csv'
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['index','mode','x_m','y_m','latitude','longitude','event'])
            for i in range(30):
                writer.writerow([i,2 if i<10 else 8 if i<20 else 9,i,0,37,127,
                                 'STOP_LINE' if i in (3,7,15,18) else 'NONE'])
        path.with_suffix('.yaml').write_text('origin_lat: 37.0\norigin_lon: 127.0\n')
        rclpy.init(args=['--ros-args', '-p', f'route_csv:={path}',
            '-p','ramp_first_index:=3','-p','ramp_stop_index:=7',
            '-p','acceleration_start_index:=21','-p','acceleration_end_index:=27',
            '-p','waypoint_control:=true','-p','waypoint_signal_control:=true',
            '-p','input_guard_topic:=/mission/route_valid','-p','ramp_second_line_stop_sec:=0.1'])
        fused, mission, perception = FusedWaypointNode(), CourseMissionNode(), TrafficLightColorNode()
        source = Node('fused_waypoint_test_source')
        executor = SingleThreadedExecutor()
        for node in (fused, mission, perception, source):
            executor.add_node(node)
        pubs = {name: source.create_publisher(kind, name, 10) for name, kind in (
            ('/fix',NavSatFix),('/mcu/odom',Odometry),('/vehicle/speed_mps',Float32),
            ('/vehicle/speed_valid',Bool),('/camera/path_valid',Bool),
            ('/camera/target_steering_deg',Float32),('/control/curvature_plan_valid',Bool),
            ('/control/curvature_drive_stage',Int32),('/imu_pitch',Float32),('/imu_valid',Bool),
            ('/perception/detections_json',String))}
        x = 0.
        speed = 0.

        def step(seconds=.09, gps=True):
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                stamp = source.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
                odom.pose.pose.orientation.w = 1.
                odom.pose.pose.position.x = x
                pubs['/mcu/odom'].publish(odom)
                if gps:
                    fix = NavSatFix()
                    fix.header.stamp = stamp
                    fix.status.status = 0
                    fix.position_covariance_type = 2
                    fix.position_covariance[0] = fix.position_covariance[4] = .0025
                    fix.latitude = 37.
                    fix.longitude = 127.+math.degrees(x/(6378137.*math.cos(math.radians(37.))))
                    pubs['/fix'].publish(fix)
                for topic, msg in (
                    ('/vehicle/speed_mps',Float32(data=speed)),('/vehicle/speed_valid',Bool(data=True)),
                    ('/camera/path_valid',Bool(data=True)),('/camera/target_steering_deg',Float32(data=0.)),
                    ('/control/curvature_plan_valid',Bool(data=True)),('/control/curvature_drive_stage',Int32(data=2)),
                    ('/imu_pitch',Float32(data=0.)),('/imu_valid',Bool(data=True))):
                    pubs[topic].publish(msg)
                for _ in range(15):
                    executor.spin_once(timeout_sec=.001)

        def frame(color):
            stamp = source.get_clock().now().to_msg()
            message = String(data=json.dumps({'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
                'detections':[{'class_name':color,'confidence':.95,'xyxy':[0,0,10,10]}]}))
            pubs['/perception/detections_json'].publish(message)
            step()
            return message

        try:
            step(.5)
            assert fused.estimator.initialized
            speed = .4
            for i in range(1,25):
                x = i*.25
                step()
                if x == 3.:
                    assert not fused.schedule.completed  # first line ignored
            speed = 0.
            step(.5)
            assert 7 in fused.schedule.completed
            for i in range(25,53):
                speed, x = .4, i*.25
                step()
            speed = 0.
            step(.3)
            assert mission.logic.signal_window
            for color in ['Left']*3+['R_light']*4:
                last = frame(color)
            assert 15 not in fused.schedule.completed
            assert perception.vote.frames.count('RED') == 4
            # Repeated delivery of the same image stamp is not another frame.
            pubs['/perception/detections_json'].publish(last)
            step()
            assert perception.vote.frames.count('RED') == 4
            for _ in range(7):
                frame('Left')
            step(.2)
            assert 15 in fused.schedule.completed
            for i in range(53,65):
                speed, x = .4, i*.25
                step()
            speed = 0.
            step(.3)
            assert 18 not in fused.schedule.completed
            assert mission.logic.signal_window.startswith(fused.session+':18/')
            assert not mission.data.traffic_left  # no first-intersection vote reuse
            # Fresh odometry permits only bounded GPS loss.
            step(.4, gps=False)
            assert fused.last_state['valid']
            step(3.2, gps=False)
            assert not fused.last_state['valid']
            assert not mission.logic.signal_window
            print('PASS: fused GPS/odom, ignored first line, ramp hold/release, 7-frame majority, second stop rearm, bounded outage')
        finally:
            for node in (fused, mission, perception, source):
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

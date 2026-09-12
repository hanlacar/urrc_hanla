import json
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from race_control.section_transition_node import SectionTransitionNode
from race_control.course_mission_node import CourseMissionNode

root=Path(__file__).resolve().parents[1] / 'routes'
rclpy.init(args=['--ros-args','-p',f'route_csv:={root / "hand_gps_aaaa.csv"}', '-p', f'segments_yaml:={root / "hand_gps_aaaa_segments.yaml"}', '-p','ramp_dr_stop_topic:=/mission/ramp_waypoint_reached', '-p','input_guard_topic:=/mission/route_valid'])
bridge=SectionTransitionNode()
mission=CourseMissionNode()
source=Node('waypoint_test_source')
pub=source.create_publisher(String,'/gps_navigation/status',10)
executor=SingleThreadedExecutor()
for node in [bridge,mission,source]: executor.add_node(node)

def run(seconds, data=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if data is not None: pub.publish(String(data=json.dumps(data)))
        executor.spin_once(timeout_sec=.01)

try:
    data=dict(route_index=164,mode='2',segment_id='SEG02',state='TRACKING',gps_healthy=True,imu_healthy=True)
    run(.8,data)
    assert mission.data.section==2
    assert not mission.data.ramp_dr_stop_reached
    data.update(state='STOPPED_AT_STOP_LINE',reason='stop line reached index=165')
    run(.3,data)
    assert mission.data.ramp_dr_stop_reached
    data.update(route_index=251, mode='3',segment_id='SEG03',state='TRACKING',reason='')
    run(.3,data)
    assert mission.data.section==3 and not mission.data.ramp_dr_stop_reached
    run(.8)
    assert not mission.data.input_guard_alive
    assert bridge.arrival is None
    print('ROS smoke passed: section, ramp acknowledgement/arrival, section exit, stale guard')
finally:
    for node in [bridge,mission,source]: node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

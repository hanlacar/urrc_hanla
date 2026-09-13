#!/usr/bin/env python3
"""Repeated deterministic mission inputs for a fixed RViz route simulation."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class AllASimInputs(Node):
    def __init__(self):
        super().__init__('dr_all_a_sim_inputs')
        self.declare_parameter('t_branch', 'T_A')
        self.declare_parameter('v_branch', 'V_A')
        self.declare_parameter('end_branch', 'END_AA')
        self.t_branch = str(self.get_parameter('t_branch').value)
        self.v_branch = str(self.get_parameter('v_branch').value)
        self.end_branch = str(self.get_parameter('end_branch').value)
        self.t_pub = self.create_publisher(String, '/t_parking/selected_slot', 10)
        self.v_pub = self.create_publisher(String, '/parallel_parking/selected_slot', 10)
        self.end_pub = self.create_publisher(String, '/dr/end_branch', 10)
        self.go_pub = self.create_publisher(Bool, '/mission/intersection_go', 10)
        self.create_timer(0.10, self.tick)
        self.get_logger().info(
            f'RViz simulation branches fixed: {self.t_branch}, '
            f'{self.v_branch}, {self.end_branch}')

    def tick(self):
        self.t_pub.publish(String(data=self.t_branch))
        self.v_pub.publish(String(data=self.v_branch))
        self.end_pub.publish(String(data=self.end_branch))
        self.go_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = AllASimInputs()
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

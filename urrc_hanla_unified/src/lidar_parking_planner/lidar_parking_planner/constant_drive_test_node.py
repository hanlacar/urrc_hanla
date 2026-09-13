#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


TEST_DRIVE_REQUEST_TOPIC = '/parking/test_drive_request'
DEFAULT_DRIVE_VALUE = 2.0
DEFAULT_PUBLISH_RATE_HZ = 20.0


class ConstantDriveTestNode(Node):
    """Publish only a test drive request; the planner remains command owner."""

    def __init__(self):
        super().__init__('constant_drive_test_node')
        self.declare_parameter('drive_value', DEFAULT_DRIVE_VALUE)
        self.declare_parameter('publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ)

        self.drive_value = float(self.get_parameter('drive_value').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(self.drive_value):
            raise ValueError('drive_value must be finite')
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be finite and greater than zero')

        self.request_pub = self.create_publisher(
            Float32, TEST_DRIVE_REQUEST_TOPIC, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_request)
        self.get_logger().info(
            f'Constant drive request publisher started: '
            f'value={self.drive_value:.3f}, rate={publish_rate_hz:.1f} Hz')

    def _publish_request(self):
        self.request_pub.publish(Float32(data=self.drive_value))


def main(args=None):
    rclpy.init(args=args)
    node = ConstantDriveTestNode()
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

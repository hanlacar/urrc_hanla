#!/usr/bin/env python3
"""Display a ROS image with OpenCV, without rqt plugins."""
import argparse
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", nargs="?", default="/perception/detections_image")
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node("opencv_result_view")
    latest = [None]

    def receive(msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            node.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        latest[0] = (frame[:, :, ::-1] if msg.encoding == "rgb8" else frame).copy()

    node.create_subscription(Image, args.topic, receive, qos_profile_sensor_data)
    window = "YOLO result - q to quit"
    waiting = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(waiting, "Waiting for " + args.topic, (10, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        print(f"Listening: {args.topic}; ROS domain: {node.context.get_domain_id()}", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            cv2.imshow(window, latest[0] if latest[0] is not None else waiting)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

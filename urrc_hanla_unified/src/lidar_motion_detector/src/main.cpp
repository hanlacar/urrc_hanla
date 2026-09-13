#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lidar_motion_detector/motion_detector_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_motion_detector::MotionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

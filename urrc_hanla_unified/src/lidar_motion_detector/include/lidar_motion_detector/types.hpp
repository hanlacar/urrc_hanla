#ifndef LIDAR_MOTION_DETECTOR__TYPES_HPP_
#define LIDAR_MOTION_DETECTOR__TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"

namespace lidar_motion_detector
{

enum class MotionClass
{
  STATIC,
  DYNAMIC
};

enum class TerrainClass
{
  OBSTACLE,
  RAMP_CANDIDATE,
  TRAVERSABLE_RAMP
};

struct Point2D
{
  // Planar coordinates in the configured target frame (base_link by default).
  double x;
  double y;
  double range;
  double angle;
  std::size_t scan_index;
  bool in_roi;
  double roi_arc_length;
  TerrainClass terrain_class;
};

struct RoiZoneCounts
{
  std::size_t stop_points;
  std::size_t slow_points;
  std::size_t caution_points;
};

struct RoiPathSample
{
  double x;
  double y;
  double heading;
  double arc_length;
};

struct WarpedRoiGeometry
{
  std::vector<RoiPathSample> centerline;
  std::vector<RoiPathSample> left_boundary;
  std::vector<RoiPathSample> right_boundary;
  std::vector<RoiPathSample> polygon;
};

struct RoiConfig
{
  bool use_roi_filter;
  bool display_all_points;
  bool classify_roi_only;
  bool publish_roi_filtered_points_only;
  double front_angle_offset_rad;
  double angle_half_width_rad;
  double length_m;
  double width_m;
  std::string roi_mode;
  bool use_steering_roi;
  double default_steering_angle_rad;
  double steering_gain;
  double max_roi_center_shift_rad;
};

struct Cluster
{
  std::vector<Point2D> points;
  Point2D centroid;
  double min_range;
  double width;
};

struct Track
{
  uint32_t id;
  Point2D centroid;
  Point2D previous_centroid;
  std::vector<Point2D> points;
  double vx;
  double vy;
  double speed;
  MotionClass motion_class;
  int age;
  int missed_count;
  int dynamic_count;
  int static_count;
  rclcpp::Time last_stamp;
};

}  // namespace lidar_motion_detector

#endif  // LIDAR_MOTION_DETECTOR__TYPES_HPP_

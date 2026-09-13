#include "lidar_motion_detector/motion_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lidar_motion_detector
{
namespace
{

constexpr double kDefaultMinRange = 0.15;
constexpr double kDefaultMaxRange = 12.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDefaultFrontAngleOffsetDeg = 0.0;
constexpr double kDefaultRoiAngleHalfWidthDeg = 30.0;
constexpr double kDefaultRoiLength = 1.5;
constexpr double kDefaultRoiWidth = 1.8;
constexpr double kDefaultMaxRoiCenterShiftDeg = 45.0;
constexpr double kDefaultClusterDistanceThreshold = 0.18;
constexpr double kDefaultClusterDistanceScale = 0.04;
constexpr int kDefaultMinClusterPoints = 3;
constexpr double kDefaultMaxClusterWidth = 2.0;
constexpr double kDefaultTrackMatchDistance = 0.7;
constexpr int kDefaultMaxMissedFrames = 5;
constexpr double kDefaultVelocityFilterAlpha = 0.7;
constexpr double kDefaultDynamicSpeedThreshold = 0.15;
constexpr int kDefaultDynamicConfirmFrames = 1;
constexpr double kDefaultStaticSpeedThreshold = 0.10;
constexpr int kDefaultStaticConfirmFrames = 3;
constexpr double kDefaultPointMarkerScale = 0.04;
constexpr double kDefaultMarkerLifetimeSec = 0.2;
constexpr double kDefaultLogThrottleSec = 1.0;

double pointDistance(const Point2D & lhs, const Point2D & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace

MotionDetectorNode::MotionDetectorNode(const rclcpp::NodeOptions & options)
: Node("motion_detector_node", options)
{
  declareParameters();
  loadParameters();
  validateParameters();
  stable_drive_since_ = now();
  pending_drive_since_ = stable_drive_since_;
  last_stop_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  static_points_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    static_points_topic_, rclcpp::QoS(10));
  dynamic_points_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    dynamic_points_topic_, rclcpp::QoS(10));
  all_motion_points_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    all_motion_points_topic_, rclcpp::QoS(10));
  roi_marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    roi_marker_topic_, rclcpp::QoS(10));
  stop_required_pub_ = create_publisher<std_msgs::msg::Bool>(
    stop_required_topic_, rclcpp::QoS(10).reliable());
  slow_required_pub_ = create_publisher<std_msgs::msg::Bool>(
    slow_required_topic_, rclcpp::QoS(10).reliable());
  roi_risk_level_pub_ = create_publisher<std_msgs::msg::Int32>(
    roi_risk_level_topic_, rclcpp::QoS(10).reliable());
  speed_limit_pub_ = create_publisher<std_msgs::msg::Float32>(
    speed_limit_topic_, rclcpp::QoS(10).reliable());
  ramp_candidate_pub_ = create_publisher<std_msgs::msg::Bool>(
    ramp_candidate_topic_, rclcpp::QoS(10).reliable());
  traversable_ramp_pub_ = create_publisher<std_msgs::msg::Bool>(
    traversable_ramp_topic_, rclcpp::QoS(10).reliable());

  auto reliable = rclcpp::QoS(10).reliable();
  parking_t_mode_pub_ = create_publisher<std_msgs::msg::Bool>("/parking/parking_t_mode", reliable);
  paring_mode_pub_ = create_publisher<std_msgs::msg::Bool>("/parking/parking_mode", reliable);
  parking_mode_text_pub_ = create_publisher<std_msgs::msg::String>("/parking/mode_text", reliable);
  parking_space_found_pub_ = create_publisher<std_msgs::msg::Bool>("/parking/space_found", reliable);
  parking_selected_side_pub_ = create_publisher<std_msgs::msg::String>("/parking/selected_side", reliable);
  parking_selected_slot_pub_ = create_publisher<std_msgs::msg::String>("/parking/selected_slot", reliable);
  parking_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/parking/ready_to_start", reliable);
  parking_entry_x_pub_ = create_publisher<std_msgs::msg::Float32>("/parking/rear_axle_entry_x_m", reliable);
  parking_entry_y_pub_ = create_publisher<std_msgs::msg::Float32>("/parking/rear_axle_entry_y_m", reliable);
  if (lidar_role_ == "rear") {
    parking_done_pub_ = create_publisher<std_msgs::msg::Bool>("/parking/done", reliable);
  }
  final_status_text_pub_ = create_publisher<std_msgs::msg::String>(output_namespace_ + "/final_status_text", reliable);
  final_status_json_pub_ = create_publisher<std_msgs::msg::String>(output_namespace_ + "/final_status_json", reliable);
  final_stop_pub_ = create_publisher<std_msgs::msg::Bool>(output_namespace_ + "/final_stop_required", reliable);
  final_slow_pub_ = create_publisher<std_msgs::msg::Bool>(output_namespace_ + "/final_slow_required", reliable);
  final_risk_pub_ = create_publisher<std_msgs::msg::Int32>(output_namespace_ + "/final_risk_level", reliable);
  final_speed_pub_ = create_publisher<std_msgs::msg::Float32>(output_namespace_ + "/final_speed_limit_mps", reliable);
  if (lidar_role_ == "front") {
    lidar_drive_text_pub_ = create_publisher<std_msgs::msg::String>("/lidar_drive_text", reliable);
    if (publish_lidar_drive_command_) {
      lidar_drive_pub_ = create_publisher<std_msgs::msg::Float32>(
        "/lidar_reactive/drive_cmd", reliable);
      RCLCPP_WARN(
        get_logger(),
        "Legacy drive output enabled on private /lidar_reactive/drive_cmd");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Private lidar reactive drive output disabled; publishing diagnostics only");
    }
  }

  mode_subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
    "/mission/state", reliable, [this](std_msgs::msg::String::ConstSharedPtr m) {mission_state_ = m->data; updateModeState();}));
  auto bool_sub = [this, reliable](const char * topic, bool * value) {
      mode_subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        topic, reliable, [this, value](std_msgs::msg::Bool::ConstSharedPtr m) {*value = m->data; updateModeState();}));
    };
  bool_sub("/parking/start", &parking_start_);
  bool_sub("/parking/t_mode", &parking_t_request_);
  bool_sub("/parking/parking_mode_request", &parking_parallel_request_);
  bool_sub("/camera/parking_mode", &camera_parking_mode_);
  bool_sub("/gps/parking_mode", &gps_parking_mode_);

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    input_scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MotionDetectorNode::onScan, this, std::placeholders::_1));

  if (roi_config_.use_steering_roi) {
    steering_subscription_ = create_subscription<std_msgs::msg::Float32>(
      steering_angle_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &MotionDetectorNode::onSteeringAngle, this, std::placeholders::_1));
  }
  if (use_speed_based_roi_length_) {
    ego_speed_subscription_ = create_subscription<std_msgs::msg::Float32>(
      ego_speed_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MotionDetectorNode::onEgoSpeed, this, std::placeholders::_1));
  }
  if (use_imu_pitch_for_ramp_) {
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MotionDetectorNode::onImu, this, std::placeholders::_1));
  }
  if (use_camera_ramp_classifier_) {
    camera_ramp_subscription_ = create_subscription<std_msgs::msg::Bool>(
      camera_ramp_topic_, rclcpp::QoS(10),
      std::bind(&MotionDetectorNode::onCameraRamp, this, std::placeholders::_1));
  }

  RCLCPP_INFO(
    get_logger(),
    "lidar role=%s input=%s source_frame=%s target_frame=%s; ROI mode=%s, "
    "steering ROI=%s, ego compensation=%s",
    lidar_role_.c_str(), input_scan_topic_.c_str(), source_scan_frame_.c_str(), target_frame_.c_str(),
    roi_config_.roi_mode.c_str(),
    roi_config_.use_steering_roi ? "enabled" : "disabled",
    use_ego_speed_compensation_ ? "enabled" : "disabled");
}

void MotionDetectorNode::declareParameters()
{
  declare_parameter<std::string>("lidar_role", "front");
  declare_parameter<std::string>("output_namespace", "/lidar");
  declare_parameter<bool>("publish_lidar_drive_command", false);
  declare_parameter<std::string>("input_scan_topic", "/scan");
  declare_parameter<bool>("use_tf_transform", true);
  declare_parameter<std::string>("source_scan_frame", "laser");
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<std::string>("output_frame", "base_link");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  declare_parameter<double>("tf_warn_throttle_sec", 1.0);
  declare_parameter<bool>("allow_no_tf_fallback", false);
  declare_parameter<bool>("use_lidar_tf_as_roi_origin", false);
  declare_parameter<std::string>("static_points_topic", "/lidar/static_points");
  declare_parameter<std::string>("dynamic_points_topic", "/lidar/dynamic_points");
  declare_parameter<std::string>(
    "all_motion_points_topic", "/lidar/all_motion_points");
  declare_parameter<std::string>("roi_marker_topic", "/lidar/roi_marker");

  declare_parameter<double>("min_range", kDefaultMinRange);
  declare_parameter<double>("max_range", kDefaultMaxRange);
  declare_parameter<bool>("use_roi_filter", true);
  declare_parameter<bool>("display_all_points", true);
  declare_parameter<bool>("classify_roi_only", true);
  declare_parameter<bool>("publish_roi_filtered_points_only", false);
  declare_parameter<double>(
    "front_angle_offset_deg", kDefaultFrontAngleOffsetDeg);
  declare_parameter<double>(
    "roi_angle_half_width_deg", kDefaultRoiAngleHalfWidthDeg);
  declare_parameter<double>("roi_length_m", kDefaultRoiLength);
  declare_parameter<double>("roi_width_m", kDefaultRoiWidth);
  declare_parameter<std::string>("roi_mode", "steering_warped_corridor");
  declare_parameter<double>("roi_zone_step_m", 0.5);
  declare_parameter<double>("stop_zone_m", 0.5);
  declare_parameter<double>("slow_zone_m", 1.0);
  declare_parameter<double>("caution_zone_m", 1.5);
  declare_parameter<bool>("use_speed_based_roi_length", true);
  declare_parameter<std::string>("ego_speed_topic", "/ego_speed_mps");
  declare_parameter<std::string>("ego_speed_unit", "mps");
  declare_parameter<double>("min_roi_length_m", 1.5);
  declare_parameter<double>("max_roi_length_m", 5.0);
  declare_parameter<double>("roi_time_horizon_sec", 1.5);
  declare_parameter<bool>("use_ratio_zones", true);
  declare_parameter<double>("stop_zone_ratio", 0.33);
  declare_parameter<double>("slow_zone_ratio", 0.66);
  declare_parameter<double>("caution_zone_ratio", 1.0);
  declare_parameter<bool>("use_steering_roi", true);
  declare_parameter<std::string>("steering_angle_topic", "/steering_angle");
  declare_parameter<std::string>("steering_angle_unit", "deg");
  declare_parameter<double>("steering_gain", 1.0);
  declare_parameter<double>(
    "max_roi_center_shift_deg", kDefaultMaxRoiCenterShiftDeg);
  declare_parameter<double>("default_steering_angle_deg", 0.0);
  declare_parameter<double>("steering_sign", 1.0);
  declare_parameter<double>("wheelbase_m", 0.33);
  declare_parameter<double>("steering_deadband_deg", 3.0);
  declare_parameter<double>("max_steering_angle_deg", 30.0);
  declare_parameter<bool>("use_warped_rectangle_roi", true);
  declare_parameter<double>("warped_roi_angle_gain", 1.0);
  declare_parameter<double>("max_warped_roi_angle_deg", 30.0);
  declare_parameter<int>("warped_roi_segments", 40);
  declare_parameter<double>("warped_roi_deadband_deg", 0.5);
  declare_parameter<bool>("use_ackermann_roi_shape", false);
  declare_parameter<double>(
    "cluster_distance_threshold", kDefaultClusterDistanceThreshold);
  declare_parameter<double>("cluster_distance_scale", kDefaultClusterDistanceScale);
  declare_parameter<int>("min_cluster_points", kDefaultMinClusterPoints);
  declare_parameter<double>("max_cluster_width", kDefaultMaxClusterWidth);
  declare_parameter<double>("track_match_distance", kDefaultTrackMatchDistance);
  declare_parameter<int>("max_missed_frames", kDefaultMaxMissedFrames);
  declare_parameter<double>("velocity_filter_alpha", kDefaultVelocityFilterAlpha);
  declare_parameter<double>(
    "dynamic_speed_threshold", kDefaultDynamicSpeedThreshold);
  declare_parameter<int>("dynamic_confirm_frames", kDefaultDynamicConfirmFrames);
  declare_parameter<double>(
    "static_speed_threshold", kDefaultStaticSpeedThreshold);
  declare_parameter<int>("static_confirm_frames", kDefaultStaticConfirmFrames);
  declare_parameter<double>("point_marker_scale", kDefaultPointMarkerScale);
  declare_parameter<double>("marker_lifetime_sec", kDefaultMarkerLifetimeSec);
  declare_parameter<double>("ego_speed_mps", 0.0);
  declare_parameter<bool>("use_ego_speed_compensation", false);
  declare_parameter<bool>("enable_roi_marker", true);
  declare_parameter<bool>("stop_on_roi_obstacle", true);
  declare_parameter<std::string>(
    "stop_required_topic", "/lidar/stop_required");
  declare_parameter<std::string>(
    "slow_required_topic", "/lidar/slow_required");
  declare_parameter<std::string>(
    "roi_risk_level_topic", "/lidar/roi_risk_level");
  declare_parameter<std::string>(
    "speed_limit_topic", "/lidar/speed_limit_mps");
  declare_parameter<int>("min_stop_points", 3);
  declare_parameter<int>("min_slow_points", 3);
  declare_parameter<int>("min_caution_points", 3);
  declare_parameter<int>("stop_confirm_frames", 1);
  declare_parameter<int>("stop_release_frames", 3);
  declare_parameter<int>("slow_confirm_frames", 1);
  declare_parameter<int>("slow_release_frames", 3);
  declare_parameter<double>("default_speed_limit_mps", 1.0);
  declare_parameter<double>("caution_speed_limit_mps", 0.6);
  declare_parameter<double>("slow_speed_limit_mps", 0.3);
  declare_parameter<double>("stop_speed_limit_mps", 0.0);
  declare_parameter<int>("curved_roi_marker_segments", 24);
  declare_parameter<bool>("enable_ramp_candidate_filter", true);
  declare_parameter<std::string>(
    "ramp_candidate_topic", "/lidar/ramp_candidate");
  declare_parameter<std::string>(
    "traversable_ramp_topic", "/lidar/traversable_ramp");
  declare_parameter<bool>("allow_ramp_pass_with_lidar_only", false);
  declare_parameter<int>("ramp_min_continuous_points", 8);
  declare_parameter<double>("ramp_max_lateral_width_m", 2.0);
  declare_parameter<double>("ramp_min_longitudinal_length_m", 0.8);
  declare_parameter<double>("ramp_candidate_speed_limit_mps", 0.3);
  declare_parameter<bool>("use_imu_pitch_for_ramp", false);
  declare_parameter<std::string>("imu_topic", "/imu/data");
  declare_parameter<double>("ramp_pitch_threshold_deg", 8.0);
  declare_parameter<bool>("use_camera_ramp_classifier", false);
  declare_parameter<std::string>(
    "camera_ramp_topic", "/camera/ramp_detected");
  declare_parameter<std::string>(
    "unknown_roi_object_policy", "slow_or_stop");
  declare_parameter<double>("log_throttle_sec", kDefaultLogThrottleSec);
  declare_parameter<bool>("parking_side_roi.enable", true);
  declare_parameter<double>("parking_side_roi.width_scale", 1.5);
  declare_parameter<double>("parking_side_roi.base_half_angle_deg", 30.0);
  declare_parameter<double>("parking_side_roi.search_length_m", 2.5);
  declare_parameter<int>("parking_side_roi.blocked_point_threshold", 8);
  declare_parameter<int>("parking_side_roi.free_point_threshold", 3);
  declare_parameter<int>("parking_side_roi.confirm_count", 3);
  declare_parameter<int>("parking_side_roi.marker_segments", 24);
  declare_parameter<bool>("parking_side_roi.apex_from_drive_roi_start_midpoint", true);
  declare_parameter<bool>("parking_side_roi.rotate_from_drive_roi_heading", true);
  declare_parameter<bool>("parking_side_roi.show_only_for_current_lidar_role", true);
  declare_parameter<double>("normal_roi.roi_start_x_m", 0.0);
  declare_parameter<double>("parking_entry.t_entry_x_m", 0.5);
  declare_parameter<double>("parking_entry.t_entry_lateral_offset_m", 0.75);
  declare_parameter<double>("parking_entry.parallel_entry_x_m", 0.8);
  declare_parameter<double>("parking_entry.parallel_entry_lateral_offset_m", 0.65);
  declare_parameter<double>("parking_entry.rear_axle_offset_from_base_m", -0.13);
  declare_parameter<double>("parking_entry.ready_x_tolerance_m", 0.15);
  declare_parameter<bool>("parking_completion.enable_rear_distance_done", true);
  declare_parameter<double>("parking_completion.rear_done_distance_m", 0.5);
  declare_parameter<bool>("static_obstacle_hold.enable", true);
  declare_parameter<int>("static_obstacle_hold.hold_frames", 3);
  declare_parameter<int>("static_obstacle_hold.release_frames", 3);
  declare_parameter<bool>("lidar_drive_filter.enable", true);
  declare_parameter<int>("lidar_drive_filter.confirm_frames", 5);
  declare_parameter<int>("lidar_drive_filter.release_frames", 5);
  declare_parameter<int>("lidar_drive_filter.default_drive", 2);
  declare_parameter<bool>("lidar_drive_filter.immediate_stop_on_zone0", true);
  declare_parameter<bool>("lidar_drive_filter.immediate_stop_on_tf_error", true);
  declare_parameter<double>("lidar_drive_filter.min_hold_duration_sec", 1.0);
  declare_parameter<double>("lidar_drive_filter.candidate_confirm_duration_sec", 1.0);
  declare_parameter<double>("lidar_drive_filter.hold_stop_duration_sec", 1.0);
  declare_parameter<double>("speed_policy.stop_speed_mps", 0.0);
  declare_parameter<double>("speed_policy.slow_speed_mps", 0.6);
  declare_parameter<double>("speed_policy.drive_speed_mps", 1.0);
  declare_parameter<double>("speed_policy.accel_speed_mps", 1.2);
}

void MotionDetectorNode::loadParameters()
{
  get_parameter("lidar_role", lidar_role_);
  get_parameter("output_namespace", output_namespace_);
  get_parameter("input_scan_topic", input_scan_topic_);
  get_parameter("use_tf_transform", use_tf_transform_);
  get_parameter("source_scan_frame", source_scan_frame_);
  get_parameter("target_frame", target_frame_);
  get_parameter("output_frame", output_frame_);
  get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  get_parameter("tf_warn_throttle_sec", tf_warn_throttle_sec_);
  get_parameter("allow_no_tf_fallback", allow_no_tf_fallback_);
  get_parameter("use_lidar_tf_as_roi_origin", use_lidar_tf_as_roi_origin_);
  active_output_frame_ = output_frame_;
  get_parameter("static_points_topic", static_points_topic_);
  get_parameter("dynamic_points_topic", dynamic_points_topic_);
  get_parameter("all_motion_points_topic", all_motion_points_topic_);
  get_parameter("roi_marker_topic", roi_marker_topic_);
  get_parameter("publish_lidar_drive_command", publish_lidar_drive_command_);
  get_parameter("min_range", min_range_);
  get_parameter("max_range", max_range_);
  get_parameter("use_roi_filter", roi_config_.use_roi_filter);
  get_parameter("display_all_points", roi_config_.display_all_points);
  get_parameter("classify_roi_only", roi_config_.classify_roi_only);
  get_parameter(
    "publish_roi_filtered_points_only",
    roi_config_.publish_roi_filtered_points_only);

  double front_angle_offset_deg = 0.0;
  double angle_half_width_deg = 0.0;
  double max_roi_center_shift_deg = 0.0;
  double default_steering_angle_deg = 0.0;
  double steering_deadband_deg = 0.0;
  double max_steering_angle_deg = 0.0;
  double max_warped_roi_angle_deg = 0.0;
  double warped_roi_deadband_deg = 0.0;
  double ramp_pitch_threshold_deg = 0.0;
  get_parameter("front_angle_offset_deg", front_angle_offset_deg);
  get_parameter("roi_angle_half_width_deg", angle_half_width_deg);
  get_parameter("roi_length_m", roi_config_.length_m);
  get_parameter("roi_width_m", roi_config_.width_m);
  get_parameter("roi_mode", roi_config_.roi_mode);
  get_parameter("roi_zone_step_m", roi_zone_step_m_);
  get_parameter("stop_zone_m", stop_zone_m_);
  get_parameter("slow_zone_m", slow_zone_m_);
  get_parameter("caution_zone_m", caution_zone_m_);
  get_parameter("use_speed_based_roi_length", use_speed_based_roi_length_);
  get_parameter("ego_speed_topic", ego_speed_topic_);
  get_parameter("ego_speed_unit", ego_speed_unit_);
  get_parameter("min_roi_length_m", min_roi_length_m_);
  get_parameter("max_roi_length_m", max_roi_length_m_);
  get_parameter("roi_time_horizon_sec", roi_time_horizon_sec_);
  get_parameter("use_ratio_zones", use_ratio_zones_);
  get_parameter("stop_zone_ratio", stop_zone_ratio_);
  get_parameter("slow_zone_ratio", slow_zone_ratio_);
  get_parameter("caution_zone_ratio", caution_zone_ratio_);
  get_parameter("use_steering_roi", roi_config_.use_steering_roi);
  get_parameter("steering_angle_topic", steering_angle_topic_);
  get_parameter("steering_angle_unit", steering_angle_unit_);
  get_parameter("steering_gain", roi_config_.steering_gain);
  get_parameter("max_roi_center_shift_deg", max_roi_center_shift_deg);
  get_parameter("default_steering_angle_deg", default_steering_angle_deg);
  get_parameter("steering_sign", steering_sign_);
  get_parameter("wheelbase_m", wheelbase_m_);
  get_parameter("steering_deadband_deg", steering_deadband_deg);
  get_parameter("max_steering_angle_deg", max_steering_angle_deg);
  get_parameter("use_warped_rectangle_roi", use_warped_rectangle_roi_);
  get_parameter("warped_roi_angle_gain", warped_roi_angle_gain_);
  get_parameter("max_warped_roi_angle_deg", max_warped_roi_angle_deg);
  get_parameter("warped_roi_segments", warped_roi_segments_);
  get_parameter("warped_roi_deadband_deg", warped_roi_deadband_deg);
  get_parameter("use_ackermann_roi_shape", use_ackermann_roi_shape_);
  roi_config_.front_angle_offset_rad = front_angle_offset_deg * kDegToRad;
  roi_config_.angle_half_width_rad = angle_half_width_deg * kDegToRad;
  roi_config_.max_roi_center_shift_rad = max_roi_center_shift_deg * kDegToRad;
  roi_config_.default_steering_angle_rad = default_steering_angle_deg * kDegToRad;
  steering_deadband_rad_ = steering_deadband_deg * kDegToRad;
  max_steering_angle_rad_ = max_steering_angle_deg * kDegToRad;
  max_warped_roi_angle_rad_ = max_warped_roi_angle_deg * kDegToRad;
  warped_roi_deadband_rad_ = warped_roi_deadband_deg * kDegToRad;
  current_steering_angle_rad_ = roi_config_.default_steering_angle_rad;
  get_parameter("cluster_distance_threshold", cluster_distance_threshold_);
  get_parameter("cluster_distance_scale", cluster_distance_scale_);
  get_parameter("min_cluster_points", min_cluster_points_);
  get_parameter("max_cluster_width", max_cluster_width_);
  get_parameter("track_match_distance", track_match_distance_);
  get_parameter("max_missed_frames", max_missed_frames_);
  get_parameter("velocity_filter_alpha", velocity_filter_alpha_);
  get_parameter("dynamic_speed_threshold", dynamic_speed_threshold_);
  get_parameter("dynamic_confirm_frames", dynamic_confirm_frames_);
  get_parameter("static_speed_threshold", static_speed_threshold_);
  get_parameter("static_confirm_frames", static_confirm_frames_);
  get_parameter("point_marker_scale", point_marker_scale_);
  get_parameter("marker_lifetime_sec", marker_lifetime_sec_);
  get_parameter("ego_speed_mps", ego_speed_mps_);
  get_parameter("use_ego_speed_compensation", use_ego_speed_compensation_);
  get_parameter("enable_roi_marker", enable_roi_marker_);
  get_parameter("stop_on_roi_obstacle", stop_on_roi_obstacle_);
  get_parameter("stop_required_topic", stop_required_topic_);
  get_parameter("slow_required_topic", slow_required_topic_);
  get_parameter("roi_risk_level_topic", roi_risk_level_topic_);
  get_parameter("speed_limit_topic", speed_limit_topic_);
  get_parameter("min_stop_points", min_stop_points_);
  get_parameter("min_slow_points", min_slow_points_);
  get_parameter("min_caution_points", min_caution_points_);
  get_parameter("stop_confirm_frames", stop_confirm_frames_);
  get_parameter("stop_release_frames", stop_release_frames_);
  get_parameter("slow_confirm_frames", slow_confirm_frames_);
  get_parameter("slow_release_frames", slow_release_frames_);
  get_parameter("default_speed_limit_mps", default_speed_limit_mps_);
  get_parameter("caution_speed_limit_mps", caution_speed_limit_mps_);
  get_parameter("slow_speed_limit_mps", slow_speed_limit_mps_);
  get_parameter("stop_speed_limit_mps", stop_speed_limit_mps_);
  get_parameter("curved_roi_marker_segments", curved_roi_marker_segments_);
  get_parameter("enable_ramp_candidate_filter", enable_ramp_candidate_filter_);
  get_parameter("ramp_candidate_topic", ramp_candidate_topic_);
  get_parameter("traversable_ramp_topic", traversable_ramp_topic_);
  get_parameter(
    "allow_ramp_pass_with_lidar_only", allow_ramp_pass_with_lidar_only_);
  get_parameter("ramp_min_continuous_points", ramp_min_continuous_points_);
  get_parameter("ramp_max_lateral_width_m", ramp_max_lateral_width_m_);
  get_parameter(
    "ramp_min_longitudinal_length_m", ramp_min_longitudinal_length_m_);
  get_parameter(
    "ramp_candidate_speed_limit_mps", ramp_candidate_speed_limit_mps_);
  get_parameter("use_imu_pitch_for_ramp", use_imu_pitch_for_ramp_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("ramp_pitch_threshold_deg", ramp_pitch_threshold_deg);
  get_parameter("use_camera_ramp_classifier", use_camera_ramp_classifier_);
  get_parameter("camera_ramp_topic", camera_ramp_topic_);
  get_parameter("unknown_roi_object_policy", unknown_roi_object_policy_);
  ramp_pitch_threshold_rad_ = ramp_pitch_threshold_deg * kDegToRad;
  get_parameter("log_throttle_sec", log_throttle_sec_);
  get_parameter("parking_side_roi.enable", parking_side_roi_enable_);
  get_parameter("parking_side_roi.width_scale", parking_side_width_scale_);
  double parking_half_deg = 30.0;
  get_parameter("parking_side_roi.base_half_angle_deg", parking_half_deg);
  parking_side_base_half_angle_rad_ = parking_half_deg * kDegToRad;
  get_parameter("parking_side_roi.search_length_m", parking_side_search_length_m_);
  get_parameter("parking_side_roi.blocked_point_threshold", parking_blocked_point_threshold_);
  get_parameter("parking_side_roi.free_point_threshold", parking_free_point_threshold_);
  get_parameter("parking_side_roi.confirm_count", parking_confirm_count_);
  get_parameter("parking_side_roi.marker_segments", parking_marker_segments_);
  get_parameter("parking_side_roi.apex_from_drive_roi_start_midpoint", parking_apex_from_drive_start_);
  get_parameter("parking_side_roi.rotate_from_drive_roi_heading", parking_rotate_from_drive_heading_);
  get_parameter("parking_side_roi.show_only_for_current_lidar_role", parking_show_current_role_only_);
  get_parameter("normal_roi.roi_start_x_m", normal_roi_start_x_m_);
  get_parameter("parking_entry.t_entry_x_m", t_entry_x_m_);
  get_parameter("parking_entry.t_entry_lateral_offset_m", t_entry_lateral_offset_m_);
  get_parameter("parking_entry.parallel_entry_x_m", parallel_entry_x_m_);
  get_parameter("parking_entry.parallel_entry_lateral_offset_m", parallel_entry_lateral_offset_m_);
  get_parameter("parking_entry.rear_axle_offset_from_base_m", rear_axle_offset_from_base_m_);
  get_parameter("parking_completion.enable_rear_distance_done", enable_rear_distance_done_);
  get_parameter("parking_completion.rear_done_distance_m", rear_done_distance_m_);
  get_parameter("static_obstacle_hold.enable", static_obstacle_hold_enable_);
  get_parameter("static_obstacle_hold.hold_frames", static_obstacle_hold_frames_);
  get_parameter("static_obstacle_hold.release_frames", static_obstacle_release_frames_);
  get_parameter("lidar_drive_filter.enable", lidar_drive_filter_enable_);
  get_parameter("lidar_drive_filter.confirm_frames", lidar_drive_confirm_frames_);
  get_parameter("lidar_drive_filter.release_frames", lidar_drive_release_frames_);
  get_parameter("lidar_drive_filter.default_drive", lidar_drive_default_);
  get_parameter("lidar_drive_filter.immediate_stop_on_zone0", immediate_stop_on_zone0_);
  get_parameter("lidar_drive_filter.immediate_stop_on_tf_error", immediate_stop_on_tf_error_);
  get_parameter("lidar_drive_filter.min_hold_duration_sec", min_hold_duration_sec_);
  get_parameter("lidar_drive_filter.candidate_confirm_duration_sec", candidate_confirm_duration_sec_);
  get_parameter("lidar_drive_filter.hold_stop_duration_sec", hold_stop_duration_sec_);
  get_parameter("speed_policy.stop_speed_mps", stop_speed_limit_mps_);
  get_parameter("speed_policy.slow_speed_mps", slow_speed_limit_mps_);
  get_parameter("speed_policy.drive_speed_mps", default_speed_limit_mps_);
  get_parameter("speed_policy.accel_speed_mps", accel_speed_limit_mps_);
}

void MotionDetectorNode::validateParameters()
{
  auto reset_double = [this](double & value, const double fallback, const char * name) {
      RCLCPP_WARN(
        get_logger(), "Invalid %s; using %.3f", name, fallback);
      value = fallback;
    };

  if (source_scan_frame_.empty()) {
    source_scan_frame_ = "laser";
  }
  if (target_frame_.empty()) {
    target_frame_ = "base_link";
  }
  if (output_frame_.empty()) {
    output_frame_ = target_frame_;
  }
  if (output_frame_ != target_frame_) {
    RCLCPP_WARN(
      get_logger(), "output_frame must match transformed point frame; using %s",
      target_frame_.c_str());
    output_frame_ = target_frame_;
  }
  active_output_frame_ = output_frame_;
  if (!std::isfinite(tf_lookup_timeout_sec_) || tf_lookup_timeout_sec_ < 0.0) {
    reset_double(tf_lookup_timeout_sec_, 0.05, "tf_lookup_timeout_sec");
  }
  if (!std::isfinite(tf_warn_throttle_sec_) || tf_warn_throttle_sec_ <= 0.0) {
    reset_double(tf_warn_throttle_sec_, 1.0, "tf_warn_throttle_sec");
  }

  if (!std::isfinite(min_range_) || min_range_ < 0.0) {
    reset_double(min_range_, kDefaultMinRange, "min_range");
  }
  if (!std::isfinite(max_range_) || max_range_ <= min_range_) {
    reset_double(max_range_, kDefaultMaxRange, "max_range");
    if (max_range_ <= min_range_) {
      reset_double(min_range_, kDefaultMinRange, "min_range");
    }
  }
  if (!std::isfinite(roi_config_.front_angle_offset_rad)) {
    reset_double(
      roi_config_.front_angle_offset_rad,
      kDefaultFrontAngleOffsetDeg * kDegToRad,
      "front_angle_offset_deg");
  }
  roi_config_.front_angle_offset_rad = normalizeAngle(
    roi_config_.front_angle_offset_rad);
  if (!std::isfinite(roi_config_.angle_half_width_rad) ||
    roi_config_.angle_half_width_rad <= 0.0 ||
    roi_config_.angle_half_width_rad > kPi)
  {
    reset_double(
      roi_config_.angle_half_width_rad,
      kDefaultRoiAngleHalfWidthDeg * kDegToRad,
      "roi_angle_half_width_deg");
  }
  if (!std::isfinite(roi_config_.length_m) || roi_config_.length_m <= 0.0) {
    reset_double(roi_config_.length_m, kDefaultRoiLength, "roi_length_m");
  }
  if (!std::isfinite(roi_config_.width_m) || roi_config_.width_m <= 0.0) {
    reset_double(roi_config_.width_m, kDefaultRoiWidth, "roi_width_m");
  }
  if (roi_config_.roi_mode != "sector" &&
    roi_config_.roi_mode != "corridor" &&
    roi_config_.roi_mode != "adaptive_sector" &&
    roi_config_.roi_mode != "curved_corridor" &&
    roi_config_.roi_mode != "steering_warped_corridor")
  {
    RCLCPP_WARN(
      get_logger(), "Invalid roi_mode '%s'; using steering_warped_corridor",
      roi_config_.roi_mode.c_str());
    roi_config_.roi_mode = "steering_warped_corridor";
  }
  if (steering_angle_unit_ != "deg" && steering_angle_unit_ != "rad") {
    RCLCPP_WARN(
      get_logger(), "Invalid steering_angle_unit '%s'; using deg",
      steering_angle_unit_.c_str());
    steering_angle_unit_ = "deg";
  }
  if (!std::isfinite(roi_config_.steering_gain)) {
    reset_double(roi_config_.steering_gain, 1.0, "steering_gain");
  }
  if (!std::isfinite(roi_config_.max_roi_center_shift_rad) ||
    roi_config_.max_roi_center_shift_rad < 0.0)
  {
    reset_double(
      roi_config_.max_roi_center_shift_rad,
      kDefaultMaxRoiCenterShiftDeg * kDegToRad,
      "max_roi_center_shift_deg");
  }
  if (!std::isfinite(roi_config_.default_steering_angle_rad)) {
    reset_double(
      roi_config_.default_steering_angle_rad, 0.0,
      "default_steering_angle_deg");
  }
  current_steering_angle_rad_ = roi_config_.default_steering_angle_rad;
  if (!std::isfinite(roi_zone_step_m_) || roi_zone_step_m_ <= 0.0) {
    reset_double(roi_zone_step_m_, 0.5, "roi_zone_step_m");
  }
  if (!std::isfinite(stop_zone_m_) || stop_zone_m_ <= 0.0) {
    reset_double(stop_zone_m_, roi_zone_step_m_, "stop_zone_m");
  }
  if (!std::isfinite(slow_zone_m_) || slow_zone_m_ <= stop_zone_m_) {
    reset_double(slow_zone_m_, 2.0 * roi_zone_step_m_, "slow_zone_m");
  }
  if (!std::isfinite(caution_zone_m_) || caution_zone_m_ <= slow_zone_m_) {
    reset_double(caution_zone_m_, 3.0 * roi_zone_step_m_, "caution_zone_m");
  }
  if (ego_speed_unit_ != "mps" && ego_speed_unit_ != "kmph") {
    RCLCPP_WARN(get_logger(), "Invalid ego_speed_unit; using mps");
    ego_speed_unit_ = "mps";
  }
  if (!std::isfinite(min_roi_length_m_) || min_roi_length_m_ <= 0.0) {
    reset_double(min_roi_length_m_, 1.5, "min_roi_length_m");
  }
  if (!std::isfinite(max_roi_length_m_) || max_roi_length_m_ < min_roi_length_m_) {
    reset_double(max_roi_length_m_, 5.0, "max_roi_length_m");
  }
  if (!std::isfinite(roi_time_horizon_sec_) || roi_time_horizon_sec_ <= 0.0) {
    reset_double(roi_time_horizon_sec_, 1.5, "roi_time_horizon_sec");
  }
  if (!std::isfinite(stop_zone_ratio_) || stop_zone_ratio_ <= 0.0 ||
    !std::isfinite(slow_zone_ratio_) || slow_zone_ratio_ <= stop_zone_ratio_ ||
    !std::isfinite(caution_zone_ratio_) ||
    caution_zone_ratio_ <= slow_zone_ratio_ || caution_zone_ratio_ > 1.0)
  {
    RCLCPP_WARN(get_logger(), "Invalid ROI zone ratios; using 0.33/0.66/1.0");
    stop_zone_ratio_ = 0.33;
    slow_zone_ratio_ = 0.66;
    caution_zone_ratio_ = 1.0;
  }
  if (!std::isfinite(steering_sign_) || steering_sign_ == 0.0) {
    reset_double(steering_sign_, 1.0, "steering_sign");
  }
  if (!std::isfinite(wheelbase_m_) || wheelbase_m_ <= 0.0) {
    reset_double(wheelbase_m_, 0.33, "wheelbase_m");
  }
  if (!std::isfinite(steering_deadband_rad_) || steering_deadband_rad_ < 0.0) {
    reset_double(steering_deadband_rad_, 3.0 * kDegToRad, "steering_deadband_deg");
  }
  if (!std::isfinite(max_steering_angle_rad_) || max_steering_angle_rad_ <= 0.0) {
    reset_double(
      max_steering_angle_rad_, 30.0 * kDegToRad,
      "max_steering_angle_deg");
  }
  if (steering_deadband_rad_ > max_steering_angle_rad_) {
    reset_double(steering_deadband_rad_, 3.0 * kDegToRad, "steering_deadband_deg");
  }
  if (!std::isfinite(warped_roi_angle_gain_)) {
    reset_double(warped_roi_angle_gain_, 1.0, "warped_roi_angle_gain");
  }
  if (!std::isfinite(max_warped_roi_angle_rad_) ||
    max_warped_roi_angle_rad_ <= 0.0)
  {
    reset_double(
      max_warped_roi_angle_rad_, 30.0 * kDegToRad,
      "max_warped_roi_angle_deg");
  }
  if (warped_roi_segments_ < 2) {
    RCLCPP_WARN(get_logger(), "Invalid warped_roi_segments; using 40");
    warped_roi_segments_ = 40;
  }
  if (!std::isfinite(warped_roi_deadband_rad_) ||
    warped_roi_deadband_rad_ < 0.0 ||
    warped_roi_deadband_rad_ > max_warped_roi_angle_rad_)
  {
    reset_double(
      warped_roi_deadband_rad_, 0.5 * kDegToRad,
      "warped_roi_deadband_deg");
  }
  if (!std::isfinite(cluster_distance_threshold_) ||
    cluster_distance_threshold_ <= 0.0)
  {
    reset_double(
      cluster_distance_threshold_, kDefaultClusterDistanceThreshold,
      "cluster_distance_threshold");
  }
  if (!std::isfinite(cluster_distance_scale_) || cluster_distance_scale_ < 0.0) {
    reset_double(
      cluster_distance_scale_, kDefaultClusterDistanceScale,
      "cluster_distance_scale");
  }
  if (min_cluster_points_ < 1) {
    RCLCPP_WARN(
      get_logger(), "Invalid min_cluster_points; using %d",
      kDefaultMinClusterPoints);
    min_cluster_points_ = kDefaultMinClusterPoints;
  }
  if (!std::isfinite(max_cluster_width_) || max_cluster_width_ <= 0.0) {
    reset_double(max_cluster_width_, kDefaultMaxClusterWidth, "max_cluster_width");
  }
  if (!std::isfinite(track_match_distance_) || track_match_distance_ <= 0.0) {
    reset_double(
      track_match_distance_, kDefaultTrackMatchDistance,
      "track_match_distance");
  }
  if (max_missed_frames_ < 0) {
    RCLCPP_WARN(
      get_logger(), "Invalid max_missed_frames; using %d",
      kDefaultMaxMissedFrames);
    max_missed_frames_ = kDefaultMaxMissedFrames;
  }
  if (!std::isfinite(velocity_filter_alpha_) || velocity_filter_alpha_ < 0.0 ||
    velocity_filter_alpha_ > 1.0)
  {
    reset_double(
      velocity_filter_alpha_, kDefaultVelocityFilterAlpha,
      "velocity_filter_alpha");
  }
  if (!std::isfinite(dynamic_speed_threshold_) || dynamic_speed_threshold_ <= 0.0) {
    reset_double(
      dynamic_speed_threshold_, kDefaultDynamicSpeedThreshold,
      "dynamic_speed_threshold");
  }
  if (!std::isfinite(static_speed_threshold_) || static_speed_threshold_ < 0.0 ||
    static_speed_threshold_ >= dynamic_speed_threshold_)
  {
    reset_double(
      static_speed_threshold_, kDefaultStaticSpeedThreshold,
      "static_speed_threshold");
    if (static_speed_threshold_ >= dynamic_speed_threshold_) {
      reset_double(
        dynamic_speed_threshold_, kDefaultDynamicSpeedThreshold,
        "dynamic_speed_threshold");
    }
  }
  if (dynamic_confirm_frames_ < 1) {
    RCLCPP_WARN(
      get_logger(), "Invalid dynamic_confirm_frames; using %d",
      kDefaultDynamicConfirmFrames);
    dynamic_confirm_frames_ = kDefaultDynamicConfirmFrames;
  }
  if (static_confirm_frames_ < 1) {
    RCLCPP_WARN(
      get_logger(), "Invalid static_confirm_frames; using %d",
      kDefaultStaticConfirmFrames);
    static_confirm_frames_ = kDefaultStaticConfirmFrames;
  }
  if (!std::isfinite(point_marker_scale_) || point_marker_scale_ <= 0.0) {
    reset_double(point_marker_scale_, kDefaultPointMarkerScale, "point_marker_scale");
  }
  if (!std::isfinite(marker_lifetime_sec_) || marker_lifetime_sec_ < 0.0) {
    reset_double(
      marker_lifetime_sec_, kDefaultMarkerLifetimeSec, "marker_lifetime_sec");
  }
  if (!std::isfinite(ego_speed_mps_)) {
    reset_double(ego_speed_mps_, 0.0, "ego_speed_mps");
  }
  if (stop_required_topic_.empty()) {
    RCLCPP_WARN(
      get_logger(), "Empty stop_required_topic; using /lidar/stop_required");
    stop_required_topic_ = "/lidar/stop_required";
  }
  if (slow_required_topic_.empty()) {
    slow_required_topic_ = "/lidar/slow_required";
  }
  if (roi_risk_level_topic_.empty()) {
    roi_risk_level_topic_ = "/lidar/roi_risk_level";
  }
  if (speed_limit_topic_.empty()) {
    speed_limit_topic_ = "/lidar/speed_limit_mps";
  }
  if (min_stop_points_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid min_stop_points; using 3");
    min_stop_points_ = 3;
  }
  if (min_slow_points_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid min_slow_points; using 3");
    min_slow_points_ = 3;
  }
  if (min_caution_points_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid min_caution_points; using 3");
    min_caution_points_ = 3;
  }
  if (stop_confirm_frames_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid stop_confirm_frames; using 1");
    stop_confirm_frames_ = 1;
  }
  if (stop_release_frames_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid stop_release_frames; using 3");
    stop_release_frames_ = 3;
  }
  if (slow_confirm_frames_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid slow_confirm_frames; using 1");
    slow_confirm_frames_ = 1;
  }
  if (slow_release_frames_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid slow_release_frames; using 3");
    slow_release_frames_ = 3;
  }
  if (!std::isfinite(default_speed_limit_mps_) || default_speed_limit_mps_ < 0.0) {
    reset_double(default_speed_limit_mps_, 1.0, "default_speed_limit_mps");
  }
  if (!std::isfinite(caution_speed_limit_mps_) || caution_speed_limit_mps_ < 0.0) {
    reset_double(caution_speed_limit_mps_, 0.6, "caution_speed_limit_mps");
  }
  if (!std::isfinite(slow_speed_limit_mps_) || slow_speed_limit_mps_ < 0.0) {
    reset_double(slow_speed_limit_mps_, 0.3, "slow_speed_limit_mps");
  }
  if (!std::isfinite(stop_speed_limit_mps_) || stop_speed_limit_mps_ < 0.0) {
    reset_double(stop_speed_limit_mps_, 0.0, "stop_speed_limit_mps");
  }
  if (caution_speed_limit_mps_ > default_speed_limit_mps_ ||
    slow_speed_limit_mps_ > caution_speed_limit_mps_ ||
    stop_speed_limit_mps_ > slow_speed_limit_mps_)
  {
    RCLCPP_WARN(get_logger(), "Speed limits are not monotonic; using defaults");
    default_speed_limit_mps_ = 1.0;
    caution_speed_limit_mps_ = 0.6;
    slow_speed_limit_mps_ = 0.3;
    stop_speed_limit_mps_ = 0.0;
  }
  current_speed_limit_mps_ = default_speed_limit_mps_;
  if (curved_roi_marker_segments_ < 4) {
    RCLCPP_WARN(get_logger(), "Invalid curved_roi_marker_segments; using 24");
    curved_roi_marker_segments_ = 24;
  }
  if (ramp_candidate_topic_.empty()) {
    ramp_candidate_topic_ = "/lidar/ramp_candidate";
  }
  if (traversable_ramp_topic_.empty()) {
    traversable_ramp_topic_ = "/lidar/traversable_ramp";
  }
  if (ramp_min_continuous_points_ < 2) {
    RCLCPP_WARN(get_logger(), "Invalid ramp_min_continuous_points; using 8");
    ramp_min_continuous_points_ = 8;
  }
  if (!std::isfinite(ramp_max_lateral_width_m_) ||
    ramp_max_lateral_width_m_ <= 0.0)
  {
    reset_double(
      ramp_max_lateral_width_m_, 2.0, "ramp_max_lateral_width_m");
  }
  if (!std::isfinite(ramp_min_longitudinal_length_m_) ||
    ramp_min_longitudinal_length_m_ <= 0.0)
  {
    reset_double(
      ramp_min_longitudinal_length_m_, 0.8,
      "ramp_min_longitudinal_length_m");
  }
  if (!std::isfinite(ramp_candidate_speed_limit_mps_) ||
    ramp_candidate_speed_limit_mps_ < 0.0)
  {
    reset_double(
      ramp_candidate_speed_limit_mps_, 0.3,
      "ramp_candidate_speed_limit_mps");
  }
  if (!std::isfinite(ramp_pitch_threshold_rad_) || ramp_pitch_threshold_rad_ < 0.0) {
    reset_double(
      ramp_pitch_threshold_rad_, 8.0 * kDegToRad,
      "ramp_pitch_threshold_deg");
  }
  if (unknown_roi_object_policy_ != "slow_or_stop" &&
    unknown_roi_object_policy_ != "stop" &&
    unknown_roi_object_policy_ != "slow")
  {
    RCLCPP_WARN(get_logger(), "Invalid unknown_roi_object_policy; using slow_or_stop");
    unknown_roi_object_policy_ = "slow_or_stop";
  }
  effective_roi_length_m_ = min_roi_length_m_;
  updateEffectiveRoiLength();
  if (!std::isfinite(log_throttle_sec_) || log_throttle_sec_ <= 0.0) {
    reset_double(log_throttle_sec_, kDefaultLogThrottleSec, "log_throttle_sec");
  }
  lidar_drive_confirm_frames_ = std::max(1, lidar_drive_confirm_frames_);
  lidar_drive_release_frames_ = std::max(1, lidar_drive_release_frames_);
  if (lidar_drive_default_ < 0 || lidar_drive_default_ > 3) {
    RCLCPP_WARN(get_logger(), "Invalid lidar_drive_filter.default_drive; using 2");
    lidar_drive_default_ = 2;
  }
  stable_lidar_drive_ = lidar_drive_default_;
  pending_lidar_drive_ = lidar_drive_default_;
  lidar_drive_ = lidar_drive_default_;
  if (!std::isfinite(min_hold_duration_sec_) || min_hold_duration_sec_ < 0.0) {
    reset_double(min_hold_duration_sec_, 1.0, "lidar_drive_filter.min_hold_duration_sec");
  }
  if (!std::isfinite(candidate_confirm_duration_sec_) || candidate_confirm_duration_sec_ < 0.0) {
    reset_double(candidate_confirm_duration_sec_, 1.0, "lidar_drive_filter.candidate_confirm_duration_sec");
  }
  if (!std::isfinite(hold_stop_duration_sec_) || hold_stop_duration_sec_ < 0.0) {
    reset_double(hold_stop_duration_sec_, 1.0, "lidar_drive_filter.hold_stop_duration_sec");
  }
  if (!std::isfinite(accel_speed_limit_mps_) || accel_speed_limit_mps_ < 0.0) {
    reset_double(accel_speed_limit_mps_, 1.2, "speed_policy.accel_speed_mps");
  }
}

void MotionDetectorNode::onScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  latest_points_.clear();
  traversable_ramp_indices_.clear();
  ramp_candidate_ = false;
  traversable_ramp_ = false;
  std_msgs::msg::Header output_header = msg->header;
  output_header.frame_id = output_frame_;
  if (!msg->header.frame_id.empty() &&
    msg->header.frame_id != source_scan_frame_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      static_cast<int64_t>(tf_warn_throttle_sec_ * 1000.0),
      "Expected front LiDAR frame '%s' but scan frame is '%s'. "
      "Check the rplidar frame_id argument.",
      source_scan_frame_.c_str(), msg->header.frame_id.c_str());
  }

  if (msg->ranges.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "LaserScan ranges are empty; clearing markers");
    publishEmptyMarkers(output_header);
    updateRiskState(output_header);
    publishRiskState(output_header);
    publishRampState(output_header);
    return;
  }
  if (!std::isfinite(msg->angle_increment) || msg->angle_increment == 0.0F) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "LaserScan angle_increment is invalid; clearing markers");
    publishEmptyMarkers(output_header);
    updateRiskState(output_header);
    publishRiskState(output_header);
    publishRampState(output_header);
    return;
  }
  if (!std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
    msg->range_min >= msg->range_max)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "LaserScan range metadata is invalid; clearing markers");
    publishEmptyMarkers(output_header);
    updateRiskState(output_header);
    publishRiskState(output_header);
    publishRampState(output_header);
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  if (use_tf_transform_) {
    if (!lookupScanToTargetTransform(*msg, transform)) {
      tf_error_ = true;
      if (!allow_no_tf_fallback_) {
        forceSafetyStop();
        publishEmptyMarkers(output_header);
        publishRiskState(output_header);
        publishRampState(output_header);
        return;
      }
      transform.header = msg->header;
      transform.header.frame_id = msg->header.frame_id.empty() ?
        source_scan_frame_ : msg->header.frame_id;
      transform.child_frame_id = transform.header.frame_id;
      transform.transform.rotation.w = 1.0;
      active_output_frame_ = transform.header.frame_id;
    } else {
      tf_error_ = false;
      active_output_frame_ = output_frame_;
    }
  } else {
    tf_error_ = false;
    transform.header = msg->header;
    transform.header.frame_id = msg->header.frame_id.empty() ?
      source_scan_frame_ : msg->header.frame_id;
    transform.child_frame_id = transform.header.frame_id;
    transform.transform.rotation.w = 1.0;
    active_output_frame_ = transform.header.frame_id;
  }
  scan_to_target_transform_ = transform;
  roi_origin_x_ = 0.0;
  roi_origin_y_ = 0.0;
  roi_heading_rad_ = 0.0;
  if (use_lidar_tf_as_roi_origin_) {
    const double transform_yaw = tf2::getYaw(transform.transform.rotation);
    if (std::isfinite(transform.transform.translation.x) &&
      std::isfinite(transform.transform.translation.y) &&
      std::isfinite(transform_yaw))
    {
      roi_origin_x_ = transform.transform.translation.x;
      roi_origin_y_ = transform.transform.translation.y;
      roi_heading_rad_ = normalizeAngle(transform_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), logThrottleMilliseconds(),
        "Invalid LiDAR TF origin/heading; using target-frame ROI origin");
    }
  }
  output_header.frame_id = active_output_frame_;

  updateWarpedRoiGeometry();
  preprocessScan(*msg, latest_points_);
  if (latest_points_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "LaserScan has no valid points after range filtering; clearing markers");
    publishEmptyMarkers(output_header);
    updateRiskState(output_header);
    publishRiskState(output_header);
    publishRampState(output_header);
    return;
  }

  evaluateRampCandidates();

  std::vector<Point2D> classification_points;
  classification_points.reserve(latest_points_.size());
  for (const Point2D & point : latest_points_) {
    if (!roi_config_.classify_roi_only || point.in_roi) {
      classification_points.push_back(point);
    }
  }

  std::vector<Cluster> clusters;
  clusterPoints(classification_points, clusters);
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (clusters.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "No valid clusters in LaserScan; publishing all valid points as STATIC");
  }

  updateTracks(clusters, stamp);
  publishMarkers(output_header);
  publishRoiMarker(output_header);
  updateRiskState(output_header);
  publishRiskState(output_header);
  publishRampState(output_header);

  const RoiZoneCounts zone_counts = countRoiZonePoints();
  const std::size_t roi_point_count = zone_counts.stop_points +
    zone_counts.slow_points + zone_counts.caution_points;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), logThrottleMilliseconds(),
    "mode=%s speed=%.2f L=%.2f steer=%.1fdeg warp=%.1fdeg width=%.2f "
    "valid=%zu roi=%zu stop_zone=%zu slow_zone=%zu caution_zone=%zu "
    "risk=%d speed_limit=%.2f ramp_candidate=%s traversable=%s",
    roi_config_.roi_mode.c_str(), ego_speed_mps_, currentRoiLength(),
    current_steering_angle_rad_ * kRadToDeg,
    computeWarpedRoiAngle() * kRadToDeg, roi_config_.width_m,
    latest_points_.size(), roi_point_count, zone_counts.stop_points,
    zone_counts.slow_points, zone_counts.caution_points, roi_risk_level_,
    current_speed_limit_mps_,
    ramp_candidate_ ? "true" : "false",
    traversable_ramp_ ? "true" : "false");
}

bool MotionDetectorNode::lookupScanToTargetTransform(
  const sensor_msgs::msg::LaserScan & scan,
  geometry_msgs::msg::TransformStamped & transform)
{
  const std::string source_frame = scan.header.frame_id.empty() ?
    source_scan_frame_ : scan.header.frame_id;
  if (source_frame == target_frame_) {
    transform.header = scan.header;
    transform.header.frame_id = target_frame_;
    transform.child_frame_id = source_frame;
    transform.transform.rotation.w = 1.0;
    return true;
  }

  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(
    tf_lookup_timeout_sec_);
  try {
    const rclcpp::Time scan_time(
      scan.header.stamp, get_clock()->get_clock_type());
    transform = tf_buffer_->lookupTransform(
      target_frame_, source_frame, scan_time, timeout);
    return true;
  } catch (const tf2::TransformException & stamped_error) {
    try {
      const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
      transform = tf_buffer_->lookupTransform(
        target_frame_, source_frame, latest_time, timeout);
      return true;
    } catch (const tf2::TransformException & latest_error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(),
        static_cast<int64_t>(tf_warn_throttle_sec_ * 1000.0),
        "TF %s -> %s unavailable: stamped='%s', latest='%s'",
        source_frame.c_str(), target_frame_.c_str(), stamped_error.what(),
        latest_error.what());
      return false;
    }
  }
}

Point2D MotionDetectorNode::transformLaserPointToTarget(
  const double laser_x,
  const double laser_y,
  const std::size_t scan_index,
  const geometry_msgs::msg::TransformStamped & transform) const
{
  geometry_msgs::msg::PointStamped laser_point;
  laser_point.header.frame_id = transform.child_frame_id;
  laser_point.header.stamp = transform.header.stamp;
  laser_point.point.x = laser_x;
  laser_point.point.y = laser_y;
  laser_point.point.z = 0.0;

  geometry_msgs::msg::PointStamped target_point;
  tf2::doTransform(laser_point, target_point, transform);
  const double target_range = std::hypot(
    target_point.point.x, target_point.point.y);
  const double target_angle = std::atan2(
    target_point.point.y, target_point.point.x);
  return Point2D{
    target_point.point.x, target_point.point.y, target_range, target_angle,
    scan_index, false, 0.0, TerrainClass::OBSTACLE};
}

bool MotionDetectorNode::transformScanPoint(
  const double range,
  const double angle,
  const std::size_t scan_index,
  const geometry_msgs::msg::TransformStamped & transform,
  Point2D & out_point) const
{
  const double laser_x = range * std::cos(angle);
  const double laser_y = range * std::sin(angle);
  out_point = transformLaserPointToTarget(
    laser_x, laser_y, scan_index, transform);
  return std::isfinite(out_point.x) && std::isfinite(out_point.y) &&
         std::isfinite(out_point.range) && std::isfinite(out_point.angle);
}

void MotionDetectorNode::forceSafetyStop()
{
  stop_required_ = true;
  slow_required_ = true;
  roi_risk_level_ = 3;
  current_speed_limit_mps_ = stop_speed_limit_mps_;
  stop_counter_ = stop_confirm_frames_;
  stop_clear_counter_ = 0;
  slow_counter_ = slow_confirm_frames_;
  slow_clear_counter_ = 0;
}

void MotionDetectorNode::preprocessScan(
  const sensor_msgs::msg::LaserScan & scan,
  std::vector<Point2D> & points)
{
  points.clear();
  points.reserve(scan.ranges.size());

  const double effective_min_range = std::max(
    static_cast<double>(scan.range_min), min_range_);
  const double effective_max_range = std::min(
    static_cast<double>(scan.range_max), max_range_);

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) || range <= 0.0 || range < effective_min_range ||
      range > effective_max_range)
    {
      continue;
    }

    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    Point2D point{};
    if (!transformScanPoint(
        range, angle, index, scan_to_target_transform_, point))
    {
      continue;
    }
    point.in_roi = isPointInRoi(point);
    if (point.in_roi) {
      point.roi_arc_length = computeArcLengthInRoi(point);
    }
    points.push_back(point);
  }
}

void MotionDetectorNode::clusterPoints(
  const std::vector<Point2D> & points,
  std::vector<Cluster> & clusters)
{
  clusters.clear();
  if (points.empty()) {
    return;
  }

  Cluster current_cluster;
  current_cluster.points.push_back(points.front());

  auto finish_cluster = [this, &clusters](Cluster & cluster) {
      computeClusterProperties(cluster);
      if (cluster.points.size() >= static_cast<std::size_t>(min_cluster_points_) &&
        cluster.width <= max_cluster_width_)
      {
        clusters.push_back(std::move(cluster));
      }
    };

  for (std::size_t index = 1; index < points.size(); ++index) {
    const Point2D & previous = points[index - 1U];
    const Point2D & current = points[index];
    const double adaptive_threshold = cluster_distance_threshold_ +
      cluster_distance_scale_ * current.range;

    if (pointDistance(previous, current) > adaptive_threshold) {
      finish_cluster(current_cluster);
      current_cluster = Cluster{};
    }
    current_cluster.points.push_back(current);
  }
  finish_cluster(current_cluster);
}

void MotionDetectorNode::computeClusterProperties(Cluster & cluster)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  cluster.min_range = std::numeric_limits<double>::infinity();

  for (const Point2D & point : cluster.points) {
    sum_x += point.x;
    sum_y += point.y;
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    cluster.min_range = std::min(cluster.min_range, point.range);
  }

  const double point_count = static_cast<double>(cluster.points.size());
  const double centroid_x = sum_x / point_count;
  const double centroid_y = sum_y / point_count;
  cluster.centroid = Point2D{
    centroid_x,
    centroid_y,
    std::hypot(centroid_x, centroid_y),
    std::atan2(centroid_y, centroid_x),
    cluster.points[cluster.points.size() / 2U].scan_index,
    cluster.points[cluster.points.size() / 2U].in_roi,
    cluster.points[cluster.points.size() / 2U].roi_arc_length,
    cluster.points[cluster.points.size() / 2U].terrain_class};
  cluster.width = std::hypot(max_x - min_x, max_y - min_y);
}

void MotionDetectorNode::updateTracks(
  const std::vector<Cluster> & clusters,
  const rclcpp::Time & stamp)
{
  struct MatchCandidate
  {
    std::size_t track_index;
    std::size_t cluster_index;
    double distance;
  };

  const std::size_t existing_track_count = tracks_.size();
  std::vector<bool> track_matched(existing_track_count, false);
  std::vector<bool> cluster_matched(clusters.size(), false);
  std::vector<MatchCandidate> candidates;
  candidates.reserve(existing_track_count * clusters.size());

  for (std::size_t track_index = 0; track_index < existing_track_count;
    ++track_index)
  {
    for (std::size_t cluster_index = 0; cluster_index < clusters.size();
      ++cluster_index)
    {
      const double distance = pointDistance(
        tracks_[track_index].centroid, clusters[cluster_index].centroid);
      if (distance <= track_match_distance_) {
        candidates.push_back(MatchCandidate{track_index, cluster_index, distance});
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const MatchCandidate & lhs, const MatchCandidate & rhs) {
      return lhs.distance < rhs.distance;
    });

  for (const MatchCandidate & candidate : candidates) {
    if (track_matched[candidate.track_index] ||
      cluster_matched[candidate.cluster_index])
    {
      continue;
    }

    Track & track = tracks_[candidate.track_index];
    const Cluster & cluster = clusters[candidate.cluster_index];
    const double dt = (stamp - track.last_stamp).seconds();

    track.previous_centroid = track.centroid;
    track.centroid = cluster.centroid;
    track.points = cluster.points;
    ++track.age;
    track.missed_count = 0;

    if (dt > 0.0) {
      double measured_vx =
        (track.centroid.x - track.previous_centroid.x) / dt;
      double measured_vy =
        (track.centroid.y - track.previous_centroid.y) / dt;
      applyEgoSpeedCompensation(measured_vx, measured_vy);

      track.vx = velocity_filter_alpha_ * measured_vx +
        (1.0 - velocity_filter_alpha_) * track.vx;
      track.vy = velocity_filter_alpha_ * measured_vy +
        (1.0 - velocity_filter_alpha_) * track.vy;
      track.speed = std::hypot(track.vx, track.vy);

      if (dt > 0.5) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), logThrottleMilliseconds(),
          "Track update interval is %.3f s; classification counters advance by at most one",
          dt);
      }
      classifyTrack(track);
    }
    track.last_stamp = stamp;

    track_matched[candidate.track_index] = true;
    cluster_matched[candidate.cluster_index] = true;
  }

  for (std::size_t index = 0; index < existing_track_count; ++index) {
    if (!track_matched[index]) {
      ++tracks_[index].missed_count;
    }
  }

  for (std::size_t index = 0; index < clusters.size(); ++index) {
    if (cluster_matched[index]) {
      continue;
    }
    const Cluster & cluster = clusters[index];
    tracks_.push_back(Track{
      next_track_id_++,
      cluster.centroid,
      cluster.centroid,
      cluster.points,
      0.0,
      0.0,
      0.0,
      MotionClass::STATIC,
      1,
      0,
      0,
      1,
      stamp});
  }

  removeStaleTracks();
}

void MotionDetectorNode::classifyTrack(Track & track)
{
  if (track.speed >= dynamic_speed_threshold_) {
    track.dynamic_count = std::min(
      track.dynamic_count + 1, dynamic_confirm_frames_);
  } else {
    track.dynamic_count = std::max(0, track.dynamic_count - 1);
  }

  if (track.speed <= static_speed_threshold_) {
    track.static_count = std::min(track.static_count + 1, static_confirm_frames_);
  } else {
    track.static_count = std::max(0, track.static_count - 1);
  }

  if (track.dynamic_count >= dynamic_confirm_frames_) {
    track.motion_class = MotionClass::DYNAMIC;
  } else if (track.static_count >= static_confirm_frames_) {
    track.motion_class = MotionClass::STATIC;
  }
}

void MotionDetectorNode::removeStaleTracks()
{
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) {
        return track.missed_count > max_missed_frames_;
      }),
    tracks_.end());
}

void MotionDetectorNode::publishMarkers(const std_msgs::msg::Header & header)
{
  std::unordered_set<std::size_t> dynamic_scan_indices;
  for (const Track & track : tracks_) {
    if (track.missed_count != 0 || track.motion_class != MotionClass::DYNAMIC) {
      continue;
    }
    dynamic_scan_indices.reserve(dynamic_scan_indices.size() + track.points.size());
    for (const Point2D & point : track.points) {
      dynamic_scan_indices.insert(point.scan_index);
    }
  }

  auto static_marker = createPointMarker(
    header, MotionClass::STATIC, "static_points", 1);
  auto dynamic_marker = createPointMarker(
    header, MotionClass::DYNAMIC, "dynamic_points", 2);
  auto all_static_marker = createPointMarker(
    header, MotionClass::STATIC, "static_points", 1);
  auto all_dynamic_marker = createPointMarker(
    header, MotionClass::DYNAMIC, "dynamic_points", 2);
  static_marker.points.reserve(latest_points_.size());
  dynamic_marker.points.reserve(dynamic_scan_indices.size());
  all_static_marker.points.reserve(latest_points_.size());
  all_dynamic_marker.points.reserve(dynamic_scan_indices.size());

  for (const Point2D & point : latest_points_) {
    geometry_msgs::msg::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.0;
    const bool is_dynamic = dynamic_scan_indices.count(point.scan_index) > 0U;

    if (!roi_config_.publish_roi_filtered_points_only || point.in_roi) {
      auto & class_marker = is_dynamic ? dynamic_marker : static_marker;
      class_marker.points.push_back(marker_point);
    }
    if (roi_config_.display_all_points || point.in_roi) {
      auto & all_marker = is_dynamic ? all_dynamic_marker : all_static_marker;
      all_marker.points.push_back(marker_point);
    }
  }

  const auto delete_all = createDeleteAllMarker(header);

  visualization_msgs::msg::MarkerArray static_markers;
  static_markers.markers.reserve(2U);
  static_markers.markers.push_back(delete_all);
  static_markers.markers.push_back(static_marker);

  visualization_msgs::msg::MarkerArray dynamic_markers;
  dynamic_markers.markers.reserve(2U);
  dynamic_markers.markers.push_back(delete_all);
  dynamic_markers.markers.push_back(dynamic_marker);

  visualization_msgs::msg::MarkerArray all_markers;
  all_markers.markers.reserve(3U);
  all_markers.markers.push_back(delete_all);
  all_markers.markers.push_back(std::move(all_static_marker));
  all_markers.markers.push_back(std::move(all_dynamic_marker));

  static_points_publisher_->publish(static_markers);
  dynamic_points_publisher_->publish(dynamic_markers);
  all_motion_points_publisher_->publish(all_markers);
}

void MotionDetectorNode::publishEmptyMarkers(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(createDeleteAllMarker(header));
  static_points_publisher_->publish(markers);
  dynamic_points_publisher_->publish(markers);
  all_motion_points_publisher_->publish(markers);
  roi_marker_publisher_->publish(markers);
}

void MotionDetectorNode::onSteeringAngle(
  const std_msgs::msg::Float32::ConstSharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "Ignoring non-finite steering angle");
    return;
  }

  current_steering_angle_rad_ = steering_angle_unit_ == "deg" ?
    static_cast<double>(msg->data) * kDegToRad :
    static_cast<double>(msg->data);
}

void MotionDetectorNode::onEgoSpeed(
  const std_msgs::msg::Float32::ConstSharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logThrottleMilliseconds(),
      "Ignoring non-finite ego speed");
    return;
  }
  ego_speed_mps_ = ego_speed_unit_ == "kmph" ?
    static_cast<double>(msg->data) / 3.6 : static_cast<double>(msg->data);
  updateEffectiveRoiLength();
}

void MotionDetectorNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto & q = msg->orientation;
  const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  latest_imu_pitch_rad_ = std::asin(std::clamp(sin_pitch, -1.0, 1.0));
}

void MotionDetectorNode::onCameraRamp(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  camera_ramp_detected_ = msg->data;
}

void MotionDetectorNode::updateEffectiveRoiLength()
{
  effective_roi_length_m_ = use_speed_based_roi_length_ ?
    std::clamp(
      std::abs(ego_speed_mps_) * roi_time_horizon_sec_,
      min_roi_length_m_, max_roi_length_m_) : roi_config_.length_m;
}

double MotionDetectorNode::currentRoiLength() const
{
  return effective_roi_length_m_;
}

double MotionDetectorNode::currentStopZone() const
{
  return use_ratio_zones_ ? currentRoiLength() * stop_zone_ratio_ : stop_zone_m_;
}

double MotionDetectorNode::currentSlowZone() const
{
  return use_ratio_zones_ ? currentRoiLength() * slow_zone_ratio_ : slow_zone_m_;
}

double MotionDetectorNode::currentCautionZone() const
{
  return use_ratio_zones_ ?
    currentRoiLength() * caution_zone_ratio_ : caution_zone_m_;
}

double MotionDetectorNode::computeWarpedRoiAngle() const
{
  if (roi_config_.roi_mode != "steering_warped_corridor" ||
    !use_warped_rectangle_roi_)
  {
    return 0.0;
  }
  const double warped_angle = std::clamp(
    steering_sign_ * current_steering_angle_rad_ * warped_roi_angle_gain_,
    -max_warped_roi_angle_rad_, max_warped_roi_angle_rad_);
  return std::abs(warped_angle) <= warped_roi_deadband_rad_ ? 0.0 :
         warped_angle;
}

void MotionDetectorNode::updateWarpedRoiGeometry()
{
  warped_roi_geometry_.centerline.clear();
  warped_roi_geometry_.left_boundary.clear();
  warped_roi_geometry_.right_boundary.clear();
  warped_roi_geometry_.polygon.clear();
  if (roi_config_.roi_mode != "steering_warped_corridor") {
    return;
  }

  const std::size_t segment_count = static_cast<std::size_t>(warped_roi_segments_);
  const double length = currentRoiLength();
  const double half_width = roi_config_.width_m * 0.5;
  const double warped_angle = computeWarpedRoiAngle();
  const double segment_length = length / static_cast<double>(segment_count);

  auto & centerline = warped_roi_geometry_.centerline;
  auto & left_boundary = warped_roi_geometry_.left_boundary;
  auto & right_boundary = warped_roi_geometry_.right_boundary;
  centerline.reserve(segment_count + 1U);
  left_boundary.reserve(segment_count + 1U);
  right_boundary.reserve(segment_count + 1U);

  double x = 0.0;
  double y = 0.0;
  for (std::size_t index = 0; index <= segment_count; ++index) {
    const double arc_length = static_cast<double>(index) * segment_length;
    const double heading = warped_angle * arc_length / length;
    centerline.push_back(RoiPathSample{x, y, heading, arc_length});
    left_boundary.push_back(RoiPathSample{
      x - std::sin(heading) * half_width,
      y + std::cos(heading) * half_width, heading, arc_length});
    right_boundary.push_back(RoiPathSample{
      x + std::sin(heading) * half_width,
      y - std::cos(heading) * half_width, heading, arc_length});

    if (index < segment_count) {
      const double midpoint_arc =
        (static_cast<double>(index) + 0.5) * segment_length;
      const double midpoint_heading = warped_angle * midpoint_arc / length;
      x += std::cos(midpoint_heading) * segment_length;
      y += std::sin(midpoint_heading) * segment_length;
    }
  }

  auto & polygon = warped_roi_geometry_.polygon;
  polygon.reserve(2U * (segment_count + 1U));
  polygon.insert(polygon.end(), left_boundary.begin(), left_boundary.end());
  polygon.insert(
    polygon.end(), right_boundary.rbegin(), right_boundary.rend());
}

void MotionDetectorNode::pointToRoiLocal(
  const Point2D & point,
  const double center_angle,
  double & local_x,
  double & local_y) const
{
  const double delta_x = point.x - roi_origin_x_;
  const double delta_y = point.y - roi_origin_y_;
  const double cosine = std::cos(center_angle);
  const double sine = std::sin(center_angle);
  local_x = cosine * delta_x + sine * delta_y;
  local_y = -sine * delta_x + cosine * delta_y;
}

bool MotionDetectorNode::isPointInWarpedRoi(const Point2D & point) const
{
  const double center_angle = computeCurrentRoiCenterAngle();
  double local_x = 0.0;
  double local_y = 0.0;
  pointToRoiLocal(point, center_angle, local_x, local_y);
  return isLocalPointInPolygon(local_x, local_y);
}

bool MotionDetectorNode::isLocalPointInPolygon(
  const double x, const double y) const
{
  const auto & polygon = warped_roi_geometry_.polygon;
  if (polygon.size() < 3U) {
    return false;
  }

  bool inside = false;
  std::size_t previous_index = polygon.size() - 1U;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const RoiPathSample & current = polygon[index];
    const RoiPathSample & previous = polygon[previous_index];
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double segment_norm_squared = dx * dx + dy * dy;
    if (segment_norm_squared > 0.0) {
      const double projection = std::clamp(
        ((x - previous.x) * dx + (y - previous.y) * dy) /
        segment_norm_squared, 0.0, 1.0);
      if (std::hypot(
          x - (previous.x + projection * dx),
          y - (previous.y + projection * dy)) <= 1.0e-9)
      {
        return true;
      }
    }
    const bool crosses = (current.y > y) != (previous.y > y);
    if (crosses) {
      const double intersection_x = previous.x +
        (y - previous.y) * (current.x - previous.x) /
        (current.y - previous.y);
      if (x < intersection_x) {
        inside = !inside;
      }
    }
    previous_index = index;
  }
  return inside;
}

double MotionDetectorNode::projectOntoWarpedCenterline(
  const Point2D & point) const
{
  const auto & centerline = warped_roi_geometry_.centerline;
  if (centerline.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  const double center_angle = computeCurrentRoiCenterAngle();
  double local_x = 0.0;
  double local_y = 0.0;
  pointToRoiLocal(point, center_angle, local_x, local_y);
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_arc_length = 0.0;
  for (std::size_t index = 0; index + 1U < centerline.size(); ++index) {
    const RoiPathSample & start = centerline[index];
    const RoiPathSample & end = centerline[index + 1U];
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double segment_norm_squared = vx * vx + vy * vy;
    if (segment_norm_squared <= 0.0) {
      continue;
    }
    const double projection = std::clamp(
      ((local_x - start.x) * vx + (local_y - start.y) * vy) /
      segment_norm_squared, 0.0, 1.0);
    const double projected_x = start.x + projection * vx;
    const double projected_y = start.y + projection * vy;
    const double distance_squared =
      (local_x - projected_x) * (local_x - projected_x) +
      (local_y - projected_y) * (local_y - projected_y);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_arc_length = start.arc_length +
        projection * std::sqrt(segment_norm_squared);
    }
  }
  return best_arc_length;
}

void MotionDetectorNode::evaluateRampCandidates()
{
  ramp_candidate_ = false;
  traversable_ramp_ = false;
  traversable_ramp_indices_.clear();
  if (!enable_ramp_candidate_filter_) {
    return;
  }

  std::vector<std::size_t> run;
  std::vector<std::size_t> candidate_point_indices;
  run.reserve(latest_points_.size());
  candidate_point_indices.reserve(latest_points_.size());

  const double center_angle = computeCurrentRoiCenterAngle();
  auto finish_run = [this, center_angle, &run, &candidate_point_indices]() {
      if (run.size() < static_cast<std::size_t>(ramp_min_continuous_points_)) {
        run.clear();
        return;
      }
      double min_arc = std::numeric_limits<double>::infinity();
      double max_arc = -std::numeric_limits<double>::infinity();
      double min_lateral = std::numeric_limits<double>::infinity();
      double max_lateral = -std::numeric_limits<double>::infinity();
      for (const std::size_t point_index : run) {
        const Point2D & point = latest_points_[point_index];
        double local_x = 0.0;
        double local_y = 0.0;
        pointToRoiLocal(point, center_angle, local_x, local_y);
        (void)local_x;
        min_arc = std::min(min_arc, point.roi_arc_length);
        max_arc = std::max(max_arc, point.roi_arc_length);
        min_lateral = std::min(min_lateral, local_y);
        max_lateral = std::max(max_lateral, local_y);
      }
      if (max_arc - min_arc >= ramp_min_longitudinal_length_m_ &&
        max_lateral - min_lateral <= ramp_max_lateral_width_m_)
      {
        candidate_point_indices.insert(
          candidate_point_indices.end(), run.begin(), run.end());
      }
      run.clear();
    };

  for (std::size_t index = 0; index < latest_points_.size(); ++index) {
    const Point2D & point = latest_points_[index];
    if (!point.in_roi) {
      finish_run();
      continue;
    }
    if (!run.empty()) {
      const Point2D & previous = latest_points_[run.back()];
      if (point.scan_index != previous.scan_index + 1U) {
        finish_run();
      }
    }
    run.push_back(index);
  }
  finish_run();

  ramp_candidate_ = !candidate_point_indices.empty();
  if (!ramp_candidate_) {
    return;
  }

  const bool imu_confirms_ramp = use_imu_pitch_for_ramp_ &&
    std::abs(latest_imu_pitch_rad_) >= ramp_pitch_threshold_rad_;
  const bool camera_confirms_ramp = use_camera_ramp_classifier_ &&
    camera_ramp_detected_;
  traversable_ramp_ = allow_ramp_pass_with_lidar_only_ ||
    imu_confirms_ramp || camera_confirms_ramp;

  for (const std::size_t point_index : candidate_point_indices) {
    Point2D & point = latest_points_[point_index];
    point.terrain_class = traversable_ramp_ ?
      TerrainClass::TRAVERSABLE_RAMP : TerrainClass::RAMP_CANDIDATE;
    if (traversable_ramp_) {
      traversable_ramp_indices_.insert(point.scan_index);
    }
  }
}

void MotionDetectorNode::publishRampState(
  const std_msgs::msg::Header & header)
{
  (void)header;
  std_msgs::msg::Bool candidate_message;
  candidate_message.data = ramp_candidate_;
  ramp_candidate_pub_->publish(candidate_message);

  std_msgs::msg::Bool traversable_message;
  traversable_message.data = traversable_ramp_;
  traversable_ramp_pub_->publish(traversable_message);
}

double MotionDetectorNode::computeCurrentRoiCenterAngle() const
{
  double center_shift = 0.0;
  if (roi_config_.roi_mode == "adaptive_sector" ||
    roi_config_.roi_mode == "corridor")
  {
    center_shift = std::clamp(
      roi_config_.steering_gain * current_steering_angle_rad_,
      -roi_config_.max_roi_center_shift_rad,
      roi_config_.max_roi_center_shift_rad);
  }
  return normalizeAngle(
    roi_heading_rad_ + roi_config_.front_angle_offset_rad + center_shift);
}

bool MotionDetectorNode::isPointInRoi(const Point2D & point) const
{
  if (!roi_config_.use_roi_filter) {
    return true;
  }

  const double center_angle = computeCurrentRoiCenterAngle();
  if (roi_config_.roi_mode == "steering_warped_corridor") {
    return isPointInWarpedRoi(point);
  }
  if (roi_config_.roi_mode == "corridor") {
    return isPointInCorridorRoi(point, center_angle);
  }
  if (roi_config_.roi_mode == "curved_corridor") {
    if (!use_ackermann_roi_shape_) {
      return isPointInCorridorRoi(point, center_angle);
    }
    const double effective_steering = std::clamp(
      steering_sign_ * current_steering_angle_rad_,
      -max_steering_angle_rad_, max_steering_angle_rad_);
    if (std::abs(effective_steering) <= steering_deadband_rad_) {
      return isPointInCorridorRoi(point, center_angle);
    }

    const double curvature = std::tan(effective_steering) / wheelbase_m_;
    if (!std::isfinite(curvature) || std::abs(curvature) < 1.0e-6) {
      return isPointInCorridorRoi(point, center_angle);
    }

    double local_x = 0.0;
    double local_y = 0.0;
    pointToRoiLocal(point, center_angle, local_x, local_y);
    if (local_x < 0.0) {
      return false;
    }

    const double radius = 1.0 / curvature;
    const double point_radius = std::hypot(local_x, local_y - radius);
    const double lateral_error = std::abs(point_radius - std::abs(radius));
    const double arc_length = computeArcLengthInRoi(point);
    return std::isfinite(arc_length) && arc_length >= 0.0 &&
           arc_length <= currentRoiLength() &&
           lateral_error <= roi_config_.width_m * 0.5;
  }
  return isPointInSectorRoi(point, center_angle);
}

bool MotionDetectorNode::isPointInSectorRoi(
  const Point2D & point, const double center_angle) const
{
  double local_x = 0.0;
  double local_y = 0.0;
  pointToRoiLocal(point, center_angle, local_x, local_y);
  return std::hypot(local_x, local_y) <= currentRoiLength() &&
         std::abs(std::atan2(local_y, local_x)) <=
         roi_config_.angle_half_width_rad;
}

bool MotionDetectorNode::isPointInCorridorRoi(
  const Point2D & point, const double center_angle) const
{
  double local_x = 0.0;
  double local_y = 0.0;
  pointToRoiLocal(point, center_angle, local_x, local_y);
  return local_x >= 0.0 && local_x <= currentRoiLength() &&
         std::abs(local_y) <= roi_config_.width_m * 0.5;
}

double MotionDetectorNode::computeArcLengthInRoi(const Point2D & point) const
{
  if (roi_config_.roi_mode == "steering_warped_corridor") {
    return projectOntoWarpedCenterline(point);
  }
  const double center_angle = computeCurrentRoiCenterAngle();
  double local_x = 0.0;
  double local_y = 0.0;
  pointToRoiLocal(point, center_angle, local_x, local_y);

  if (roi_config_.roi_mode != "curved_corridor" || !use_ackermann_roi_shape_) {
    return roi_config_.roi_mode == "sector" ||
           roi_config_.roi_mode == "adaptive_sector" ?
           std::hypot(local_x, local_y) : local_x;
  }

  const double effective_steering = std::clamp(
    steering_sign_ * current_steering_angle_rad_,
    -max_steering_angle_rad_, max_steering_angle_rad_);
  if (std::abs(effective_steering) <= steering_deadband_rad_) {
    return local_x;
  }

  const double curvature = std::tan(effective_steering) / wheelbase_m_;
  if (!std::isfinite(curvature) || std::abs(curvature) < 1.0e-6) {
    return local_x;
  }

  const double radius = 1.0 / curvature;
  double theta = std::atan2(local_x, radius - local_y);
  if (radius < 0.0) {
    theta = normalizeAngle(theta - kPi);
  }
  return std::abs(radius) * std::abs(theta);
}

double MotionDetectorNode::normalizeAngle(const double angle_rad) const
{
  return std::remainder(angle_rad, 2.0 * kPi);
}

double MotionDetectorNode::shortestAngularDistance(
  const double a, const double b) const
{
  return normalizeAngle(a - b);
}

geometry_msgs::msg::Point MotionDetectorNode::computeDriveRoiStartMidpoint() const
{
  geometry_msgs::msg::Point apex;
  const double center_angle = computeCurrentRoiCenterAngle();
  const double cosine = std::cos(center_angle);
  const double sine = std::sin(center_angle);
  if (use_warped_rectangle_roi_ &&
    !warped_roi_geometry_.left_boundary.empty() &&
    !warped_roi_geometry_.right_boundary.empty())
  {
    const auto & left = warped_roi_geometry_.left_boundary.front();
    const auto & right = warped_roi_geometry_.right_boundary.front();
    const double midpoint_x = 0.5 * (left.x + right.x);
    const double midpoint_y = 0.5 * (left.y + right.y);
    apex.x = roi_origin_x_ + cosine * midpoint_x - sine * midpoint_y;
    apex.y = roi_origin_y_ + sine * midpoint_x + cosine * midpoint_y;
  } else {
    apex.x = roi_origin_x_ + normal_roi_start_x_m_ * cosine;
    apex.y = roi_origin_y_ + normal_roi_start_x_m_ * sine;
  }
  apex.z = 0.0;
  return apex;
}

double MotionDetectorNode::computeDriveRoiHeadingRad() const
{
  const double center_angle = computeCurrentRoiCenterAngle();
  if (use_warped_rectangle_roi_ && warped_roi_geometry_.centerline.size() >= 2U) {
    const auto & first = warped_roi_geometry_.centerline[0];
    const auto & second = warped_roi_geometry_.centerline[1];
    return normalizeAngle(center_angle + std::atan2(second.y - first.y, second.x - first.x));
  }
  return center_angle;
}

void MotionDetectorNode::publishRoiMarker(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(5U);
  marker_array.markers.push_back(createDeleteAllMarker(header));

  if (!enable_roi_marker_ || !roi_config_.use_roi_filter) {
    roi_marker_publisher_->publish(marker_array);
    return;
  }

  const double center_angle = computeCurrentRoiCenterAngle();
  auto make_line_marker = [&header, this](
      const int id, const std::string & marker_namespace,
      const float red, const float green, const float blue)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = marker_namespace;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 0.8F;
      marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
      return marker;
    };
  const std::string drive_prefix = lidar_role_ == "rear" ? "rear_" : "";

  const bool role_can_show_parking = !parking_show_current_role_only_ ||
    lidar_role_ == "front" || lidar_role_ == "rear";
  if (parking_side_roi_enable_ && is_parking_mode_ && role_can_show_parking) {
    const double half_angle = std::atan(
      std::tan(parking_side_base_half_angle_rad_) * parking_side_width_scale_);
    const geometry_msgs::msg::Point apex = parking_apex_from_drive_start_ ?
      computeDriveRoiStartMidpoint() : geometry_msgs::msg::Point{};
    const double drive_heading = parking_rotate_from_drive_heading_ ?
      computeDriveRoiHeadingRad() : 0.0;
    const std::string parking_prefix = lidar_role_ == "rear" ? "rear_" : "";
    auto add_side = [&](int id, double center, const std::string & ns) {
        auto marker = make_line_marker(
          id, ns, selected_side_ == (center > 0.0 ? "left" : "right") ? 1.0F : 0.2F,
          0.8F, 1.0F);
        marker.scale.x = selected_side_ == (center > 0.0 ? "left" : "right") ? 0.07 : 0.03;
        marker.points.push_back(apex);
        const int segments = std::max(4, parking_marker_segments_);
        for (int i = 0; i <= segments; ++i) {
          const double angle = center - half_angle + 2.0 * half_angle * i /
            static_cast<double>(segments);
          geometry_msgs::msg::Point p;
          p.x = apex.x + parking_side_search_length_m_ * std::cos(angle);
          p.y = apex.y + parking_side_search_length_m_ * std::sin(angle);
          marker.points.push_back(p);
        }
        marker.points.push_back(apex);
        marker_array.markers.push_back(std::move(marker));
      };
    add_side(20, drive_heading + kPi / 2.0, parking_prefix + "parking_left_roi");
    add_side(21, drive_heading - kPi / 2.0, parking_prefix + "parking_right_roi");
    if (selected_side_ != "unknown") {
      auto selected = marker_array.markers[selected_side_ == "left" ?
        marker_array.markers.size() - 2U : marker_array.markers.size() - 1U];
      selected.id = 22;
      selected.ns = parking_prefix + "selected_parking_side";
      selected.scale.x = 0.09;
      selected.color.r = 0.0F; selected.color.g = 1.0F; selected.color.b = 0.0F;
      marker_array.markers.push_back(std::move(selected));
    }
    auto mode_marker = marker_array.markers[marker_array.markers.size() - 2U];
    mode_marker.id = 23;
    mode_marker.ns = lidar_role_ == "rear" ? "rear_selected_parking_side" :
      (is_t_parking_mode_ ? "parking_t_mode_roi" : "paring_mode_roi");
    mode_marker.color.r = is_t_parking_mode_ ? 1.0F : 0.6F;
    mode_marker.color.g = 0.2F; mode_marker.color.b = is_t_parking_mode_ ? 0.2F : 1.0F;
    marker_array.markers.push_back(std::move(mode_marker));
  }

  if (roi_config_.roi_mode == "sector" ||
    roi_config_.roi_mode == "adaptive_sector")
  {
    auto marker = make_line_marker(1, drive_prefix + "drive_roi", 0.0F, 0.0F, 1.0F);
    constexpr std::size_t kArcSegments = 32U;
    marker.points.reserve(kArcSegments + 3U);
    geometry_msgs::msg::Point origin;
    origin.x = roi_origin_x_;
    origin.y = roi_origin_y_;
    origin.z = 0.0;
    marker.points.push_back(origin);
    for (std::size_t index = 0; index <= kArcSegments; ++index) {
      const double fraction = static_cast<double>(index) /
        static_cast<double>(kArcSegments);
      const double angle = center_angle - roi_config_.angle_half_width_rad +
        2.0 * roi_config_.angle_half_width_rad * fraction;
      geometry_msgs::msg::Point point;
      point.x = roi_origin_x_ + currentRoiLength() * std::cos(angle);
      point.y = roi_origin_y_ + currentRoiLength() * std::sin(angle);
      point.z = 0.0;
      marker.points.push_back(point);
    }
    marker.points.push_back(origin);
    marker_array.markers.push_back(std::move(marker));
    roi_marker_publisher_->publish(marker_array);
    return;
  }

  if (roi_config_.roi_mode == "steering_warped_corridor") {
    const double cosine = std::cos(center_angle);
    const double sine = std::sin(center_angle);
    auto to_global = [this, cosine, sine](const RoiPathSample & sample) {
        geometry_msgs::msg::Point point;
        point.x = roi_origin_x_ + cosine * sample.x - sine * sample.y;
        point.y = roi_origin_y_ + sine * sample.x + cosine * sample.y;
        point.z = 0.0;
        return point;
      };
    auto outer_marker = make_line_marker(
      1, drive_prefix + "drive_roi", 0.0F, 0.0F, 1.0F);
    outer_marker.points.reserve(warped_roi_geometry_.polygon.size() + 1U);
    for (const RoiPathSample & sample : warped_roi_geometry_.polygon) {
      outer_marker.points.push_back(to_global(sample));
    }
    if (!warped_roi_geometry_.polygon.empty()) {
      outer_marker.points.push_back(to_global(
          warped_roi_geometry_.polygon.front()));
    }
    marker_array.markers.push_back(std::move(outer_marker));

    auto interpolate_boundary = [this](
        const std::vector<RoiPathSample> & boundary,
        const double requested_arc) {
        const double arc = std::clamp(requested_arc, 0.0, currentRoiLength());
        if (boundary.size() < 2U) {
          return RoiPathSample{0.0, 0.0, 0.0, arc};
        }
        const double scaled_index = arc / currentRoiLength() *
          static_cast<double>(boundary.size() - 1U);
        const std::size_t start_index = std::min(
          static_cast<std::size_t>(std::floor(scaled_index)),
          boundary.size() - 2U);
        const double fraction = scaled_index - static_cast<double>(start_index);
        const RoiPathSample & start = boundary[start_index];
        const RoiPathSample & end = boundary[start_index + 1U];
        return RoiPathSample{
          start.x + fraction * (end.x - start.x),
          start.y + fraction * (end.y - start.y),
          start.heading + fraction * (end.heading - start.heading), arc};
      };
    auto add_warped_zone = [
      &marker_array, &make_line_marker, &interpolate_boundary, &to_global,
      this](
        const int id, const char * marker_namespace, const double arc,
        const float red, const float green, const float blue) {
        auto marker = make_line_marker(id, marker_namespace, red, green, blue);
        marker.points.reserve(2U);
        marker.points.push_back(to_global(interpolate_boundary(
              warped_roi_geometry_.left_boundary, arc)));
        marker.points.push_back(to_global(interpolate_boundary(
              warped_roi_geometry_.right_boundary, arc)));
        marker_array.markers.push_back(std::move(marker));
      };
    add_warped_zone(
      2, (drive_prefix + "drive_zone1_stop").c_str(), currentStopZone(), 1.0F, 0.0F, 0.0F);
    add_warped_zone(
      3, (drive_prefix + "drive_zone2_slow").c_str(), currentSlowZone(), 1.0F, 1.0F, 0.0F);
    add_warped_zone(
      4, (drive_prefix + "drive_zone3_caution").c_str(), currentCautionZone(), 0.0F, 0.0F, 1.0F);
    roi_marker_publisher_->publish(marker_array);
    return;
  }

  const double effective_steering = std::clamp(
    steering_sign_ * current_steering_angle_rad_,
    -max_steering_angle_rad_, max_steering_angle_rad_);
  double curvature = 0.0;
  if (roi_config_.roi_mode == "curved_corridor" &&
    use_ackermann_roi_shape_ &&
    std::abs(effective_steering) > steering_deadband_rad_)
  {
    curvature = std::tan(effective_steering) / wheelbase_m_;
    if (!std::isfinite(curvature) || std::abs(curvature) < 1.0e-6) {
      curvature = 0.0;
    }
  }

  const double cosine = std::cos(center_angle);
  const double sine = std::sin(center_angle);
  auto to_global = [this, cosine, sine](
      const double local_x, const double local_y) {
      geometry_msgs::msg::Point point;
      point.x = roi_origin_x_ + cosine * local_x - sine * local_y;
      point.y = roi_origin_y_ + sine * local_x + cosine * local_y;
      point.z = 0.0;
      return point;
    };
  auto edge_point = [this, curvature, &to_global](
      const double arc_length, const double side) {
      double path_x = arc_length;
      double path_y = 0.0;
      double heading = 0.0;
      if (std::abs(curvature) >= 1.0e-6) {
        heading = curvature * arc_length;
        path_x = std::sin(heading) / curvature;
        path_y = (1.0 - std::cos(heading)) / curvature;
      }
      const double half_width = roi_config_.width_m * 0.5;
      const double local_x = path_x - side * half_width * std::sin(heading);
      const double local_y = path_y + side * half_width * std::cos(heading);
      return to_global(local_x, local_y);
    };

  auto outer_marker = make_line_marker(1, drive_prefix + "drive_roi", 0.0F, 0.0F, 1.0F);
  const std::size_t segment_count = static_cast<std::size_t>(
    curved_roi_marker_segments_);
  outer_marker.points.reserve(2U * (segment_count + 1U) + 1U);
  for (std::size_t index = 0; index <= segment_count; ++index) {
    const double arc_length = currentRoiLength() *
      static_cast<double>(index) / static_cast<double>(segment_count);
    outer_marker.points.push_back(edge_point(arc_length, 1.0));
  }
  for (std::size_t index = segment_count + 1U; index-- > 0U;) {
    const double arc_length = currentRoiLength() *
      static_cast<double>(index) / static_cast<double>(segment_count);
    outer_marker.points.push_back(edge_point(arc_length, -1.0));
  }
  outer_marker.points.push_back(edge_point(0.0, 1.0));
  marker_array.markers.push_back(std::move(outer_marker));

  auto add_zone_boundary = [
    &marker_array, &make_line_marker, &edge_point, this](
      const int id, const char * marker_namespace, const double distance,
      const float red, const float green, const float blue)
    {
      auto marker = make_line_marker(id, marker_namespace, red, green, blue);
      const double bounded_distance = std::clamp(
        distance, 0.0, currentRoiLength());
      marker.points.reserve(2U);
      marker.points.push_back(edge_point(bounded_distance, 1.0));
      marker.points.push_back(edge_point(bounded_distance, -1.0));
      marker_array.markers.push_back(std::move(marker));
    };
  add_zone_boundary(2, (drive_prefix + "drive_zone1_stop").c_str(), currentStopZone(), 1.0F, 0.0F, 0.0F);
  add_zone_boundary(3, (drive_prefix + "drive_zone2_slow").c_str(), currentSlowZone(), 1.0F, 1.0F, 0.0F);
  add_zone_boundary(
    4, (drive_prefix + "drive_zone3_caution").c_str(), currentCautionZone(), 0.0F, 0.0F, 1.0F);

  roi_marker_publisher_->publish(marker_array);
}

RoiZoneCounts MotionDetectorNode::countRoiZonePoints() const
{
  RoiZoneCounts counts{0U, 0U, 0U};
  for (const Point2D & point : latest_points_) {
    if (!point.in_roi || !std::isfinite(point.roi_arc_length)) {
      continue;
    }
    if (traversable_ramp_indices_.count(point.scan_index) > 0U) {
      continue;
    }
    if (point.roi_arc_length < 0.0) {
      continue;
    }
    if (point.roi_arc_length <= currentStopZone()) {
      ++counts.stop_points;
    } else if (point.roi_arc_length <= currentSlowZone()) {
      ++counts.slow_points;
    } else if (point.roi_arc_length <= currentCautionZone()) {
      ++counts.caution_points;
    }
  }
  return counts;
}

void MotionDetectorNode::updateRiskState(const std_msgs::msg::Header & header)
{
  (void)header;
  if (!stop_on_roi_obstacle_) {
    stop_required_ = false;
    slow_required_ = false;
    roi_risk_level_ = 0;
    current_speed_limit_mps_ = default_speed_limit_mps_;
    stop_counter_ = 0;
    stop_clear_counter_ = 0;
    slow_counter_ = 0;
    slow_clear_counter_ = 0;
    return;
  }

  const RoiZoneCounts counts = countRoiZonePoints();
  static_obstacle_held_ = false;
  if (static_obstacle_hold_enable_) {
    for (const auto & track : tracks_) {
      if (track.motion_class == MotionClass::STATIC) {
        for (const auto & point : track.points) {
          if (point.in_roi) {static_obstacle_held_ = true; break;}
        }
      }
      if (static_obstacle_held_) {break;}
    }
  }
  int target_risk = 0;
  if (counts.stop_points >= static_cast<std::size_t>(min_stop_points_))
  {
    target_risk = 3;
  } else if (counts.slow_points >= static_cast<std::size_t>(min_slow_points_))
  {
    target_risk = 2;
  } else if (
    counts.caution_points >= static_cast<std::size_t>(min_caution_points_))
  {
    target_risk = 1;
  }

  if (ramp_candidate_ && !traversable_ramp_) {
    if (unknown_roi_object_policy_ == "stop") {
      target_risk = 3;
    } else if (unknown_roi_object_policy_ == "slow") {
      target_risk = 2;
    } else {
      target_risk = std::max(target_risk, 2);
    }
  }

  if (target_risk == 3) {
    stop_counter_ = std::min(stop_counter_ + 1, stop_confirm_frames_);
    stop_clear_counter_ = 0;
  } else {
    const int release_frames = static_obstacle_held_ ?
      std::max(stop_release_frames_, static_obstacle_release_frames_) : stop_release_frames_;
    stop_clear_counter_ = std::min(stop_clear_counter_ + 1, release_frames);
    stop_counter_ = 0;
  }
  if (stop_counter_ >= stop_confirm_frames_) {
    stop_required_ = true;
  }
  if (stop_clear_counter_ >= (static_obstacle_held_ ?
    std::max(stop_release_frames_, static_obstacle_release_frames_) : stop_release_frames_)) {
    stop_required_ = false;
  }

  if (target_risk >= 2) {
    slow_counter_ = std::min(slow_counter_ + 1, slow_confirm_frames_);
    slow_clear_counter_ = 0;
  } else {
    const int release_frames = static_obstacle_held_ ?
      std::max(slow_release_frames_, static_obstacle_release_frames_) : slow_release_frames_;
    slow_clear_counter_ = std::min(slow_clear_counter_ + 1, release_frames);
    slow_counter_ = 0;
  }
  if (slow_counter_ >= slow_confirm_frames_) {
    slow_required_ = true;
  }
  if (slow_clear_counter_ >= (static_obstacle_held_ ?
    std::max(slow_release_frames_, static_obstacle_release_frames_) : slow_release_frames_)) {
    slow_required_ = false;
  }

  roi_risk_level_ = stop_required_ ? 3 :
    (slow_required_ ? 2 : (target_risk == 1 ? 1 : 0));
  switch (roi_risk_level_) {
    case 3:
      current_speed_limit_mps_ = stop_speed_limit_mps_;
      break;
    case 2:
      current_speed_limit_mps_ = slow_speed_limit_mps_;
      break;
    case 1:
      current_speed_limit_mps_ = caution_speed_limit_mps_;
      break;
    default:
      current_speed_limit_mps_ = default_speed_limit_mps_;
      break;
  }
  if (ramp_candidate_ && !traversable_ramp_ && roi_risk_level_ < 3) {
    current_speed_limit_mps_ = std::min(
      current_speed_limit_mps_, ramp_candidate_speed_limit_mps_);
  }
}

void MotionDetectorNode::publishRiskState(
  const std_msgs::msg::Header & header)
{
  (void)header;
  evaluateParkingState();
  if (lidar_role_ == "rear" && !is_parking_mode_) {
    stop_required_ = false;
    slow_required_ = false;
    roi_risk_level_ = 0;
    current_speed_limit_mps_ = default_speed_limit_mps_;
  }
  if (lidar_role_ == "rear" && parking_done_) {
    stop_required_ = true;
    slow_required_ = false;
    roi_risk_level_ = 3;
    current_speed_limit_mps_ = stop_speed_limit_mps_;
  }
  updateDriveObstacleFlags();
  if (lidar_role_ == "front" && !tf_error_ && static_obstacle_in_zone1_) {
    zone1_obstacle_detected_ = true;
    stop_required_ = true;
    slow_required_ = false;
    roi_risk_level_ = 3;
    current_speed_limit_mps_ = stop_speed_limit_mps_;
  }
  if (lidar_role_ == "front" && !tf_error_ && !zone1_obstacle_detected_) {
    if (static_obstacle_in_zone2_ || static_obstacle_in_zone3_) {
      stop_required_ = false;
      slow_required_ = true;
      roi_risk_level_ = static_obstacle_in_zone2_ ? 2 : 1;
      current_speed_limit_mps_ = slow_speed_limit_mps_;
    } else if (zone3_obstacle_detected_) {
      slow_required_ = true;
      roi_risk_level_ = 1;
      current_speed_limit_mps_ = caution_speed_limit_mps_;
    }
  }
  std_msgs::msg::Bool stop_message;
  stop_message.data = stop_required_;
  stop_required_pub_->publish(stop_message);

  std_msgs::msg::Bool slow_message;
  slow_message.data = slow_required_;
  slow_required_pub_->publish(slow_message);

  std_msgs::msg::Int32 risk_message;
  risk_message.data = roi_risk_level_;
  roi_risk_level_pub_->publish(risk_message);

  std_msgs::msg::Float32 speed_message;
  speed_message.data = static_cast<float>(current_speed_limit_mps_);
  speed_limit_pub_->publish(speed_message);
  publishModeAndFinalState();
}

void MotionDetectorNode::updateModeState()
{
  const bool mission_t = mission_state_ == "PARKING_T_MODE" || mission_state_ == "T_PARKING";
  const bool mission_parallel = mission_state_ == "PARKING_PARALLEL_MODE" || mission_state_ == "PARALLEL_PARKING";
  const bool generic = parking_start_ || camera_parking_mode_ || gps_parking_mode_;
  is_t_parking_mode_ = mission_t || parking_t_request_;
  is_parallel_parking_mode_ = !is_t_parking_mode_ && (mission_parallel || parking_parallel_request_);
  is_parking_mode_ = is_t_parking_mode_ || is_parallel_parking_mode_ || generic;
  is_normal_drive_ = !is_parking_mode_;
  if (mission_state_ == "NORMAL_DRIVE" || mission_state_ == "DRIVE" || mission_state_ == "PARKING_EXIT") {
    if (!parking_start_ && !parking_t_request_ && !parking_parallel_request_ &&
      !camera_parking_mode_ && !gps_parking_mode_)
    {
      is_normal_drive_ = true;
      is_parking_mode_ = false;
      is_t_parking_mode_ = false;
      is_parallel_parking_mode_ = false;
      parking_done_ = false;
    }
  }
}

void MotionDetectorNode::evaluateParkingState()
{
  nearest_distance_m_ = std::numeric_limits<double>::infinity();
  std::size_t left_count = 0U, right_count = 0U;
  const double half_angle = std::atan(
    std::tan(parking_side_base_half_angle_rad_) * parking_side_width_scale_);
  const geometry_msgs::msg::Point parking_apex = computeDriveRoiStartMidpoint();
  const double drive_heading = computeDriveRoiHeadingRad();
  for (const auto & point : latest_points_) {
    nearest_distance_m_ = std::min(nearest_distance_m_, point.range);
    if (std::hypot(point.x - parking_apex.x, point.y - parking_apex.y) >
      parking_side_search_length_m_) {continue;}
    const double parking_angle = std::atan2(point.y - parking_apex.y, point.x - parking_apex.x);
    if (std::abs(shortestAngularDistance(parking_angle, drive_heading + kPi / 2.0)) <= half_angle) {++left_count;}
    if (std::abs(shortestAngularDistance(parking_angle, drive_heading - kPi / 2.0)) <= half_angle) {++right_count;}
  }
  if (!std::isfinite(nearest_distance_m_)) {nearest_distance_m_ = -1.0;}
  if (is_parking_mode_ && parking_side_roi_enable_) {
    const bool left_free = left_count <= static_cast<std::size_t>(parking_free_point_threshold_);
    const bool right_free = right_count <= static_cast<std::size_t>(parking_free_point_threshold_);
    std::string candidate = "unknown";
    if (left_free && !right_free) {candidate = "left";}
    else if (right_free && !left_free) {candidate = "right";}
    else if (left_free && right_free) {candidate = left_count <= right_count ? "left" : "right";}
    const bool blocked_all = left_count >= static_cast<std::size_t>(parking_blocked_point_threshold_) &&
      right_count >= static_cast<std::size_t>(parking_blocked_point_threshold_);
    if (blocked_all) {candidate = "unknown";}
    if (candidate == selected_side_) {++parking_selection_counter_;} else {parking_selection_counter_ = 1;}
    if (parking_selection_counter_ >= parking_confirm_count_) {selected_side_ = candidate;}
    side_obstacle_detected_ =
      (selected_side_ == "left" && left_count >= static_cast<std::size_t>(parking_blocked_point_threshold_)) ||
      (selected_side_ == "right" && right_count >= static_cast<std::size_t>(parking_blocked_point_threshold_));
    space_found_ = selected_side_ != "unknown";
    selected_slot_ = selected_side_ == "left" ? "A" : (selected_side_ == "right" ? "B" : (blocked_all ? "blocked_all" : "unknown"));
  } else {
    selected_side_ = "unknown"; selected_slot_ = "unknown"; space_found_ = false; parking_selection_counter_ = 0;
    side_obstacle_detected_ = false;
  }
  const double entry_x = is_t_parking_mode_ ? t_entry_x_m_ : parallel_entry_x_m_;
  const double lateral = is_t_parking_mode_ ? t_entry_lateral_offset_m_ : parallel_entry_lateral_offset_m_;
  rear_axle_entry_x_m_ = entry_x - rear_axle_offset_from_base_m_;
  rear_axle_entry_y_m_ = (selected_side_ == "left" ? 1.0 : -1.0) * lateral;
  // TODO(odom): replace this readiness proxy with rear-axle pose error and ready_x_tolerance_m.
  ready_to_start_ = is_parking_mode_ && space_found_ && selected_side_ != "unknown" && !stop_required_;
  if (lidar_role_ == "rear" && is_parking_mode_ && enable_rear_distance_done_ &&
    nearest_distance_m_ >= 0.0 && nearest_distance_m_ <= rear_done_distance_m_) {parking_done_ = true;}
}

void MotionDetectorNode::updateDriveObstacleFlags()
{
  std::size_t stop_count = 0U, red_yellow_count = 0U, yellow_blue_count = 0U;
  double nearest_stop = std::numeric_limits<double>::infinity();
  double nearest_red_yellow = std::numeric_limits<double>::infinity();
  double nearest_yellow_blue = std::numeric_limits<double>::infinity();
  const auto apex = computeDriveRoiStartMidpoint();
  const double heading = computeDriveRoiHeadingRad();
  const double cosine = std::cos(heading);
  const double sine = std::sin(heading);
  auto longitudinal_distance = [&](const Point2D & point) {
      return (point.x - apex.x) * cosine + (point.y - apex.y) * sine;
    };
  for (const auto & point : latest_points_) {
    if (!point.in_roi || traversable_ramp_indices_.count(point.scan_index) > 0U) {continue;}
    const double s = longitudinal_distance(point);
    if (s < 0.0 || s > caution_zone_m_) {continue;}
    if (s <= stop_zone_m_) {
      ++stop_count; nearest_stop = std::min(nearest_stop, s);
    } else if (s <= slow_zone_m_) {
      ++red_yellow_count; nearest_red_yellow = std::min(nearest_red_yellow, s);
    } else {
      ++yellow_blue_count; nearest_yellow_blue = std::min(nearest_yellow_blue, s);
    }
  }
  zone1_obstacle_detected_ = stop_count >= static_cast<std::size_t>(min_stop_points_);
  zone2_obstacle_detected_ = red_yellow_count >= static_cast<std::size_t>(min_slow_points_);
  zone3_obstacle_detected_ = yellow_blue_count >= static_cast<std::size_t>(min_caution_points_);
  nearest_stop_zone_distance_m_ = std::isfinite(nearest_stop) ? nearest_stop : -1.0;
  nearest_red_yellow_zone_distance_m_ = std::isfinite(nearest_red_yellow) ? nearest_red_yellow : -1.0;
  nearest_yellow_blue_zone_distance_m_ = std::isfinite(nearest_yellow_blue) ? nearest_yellow_blue : -1.0;
  static_obstacle_in_zone1_ = false;
  static_obstacle_in_zone2_ = false;
  static_obstacle_in_zone3_ = false;
  dynamic_obstacle_in_zone2_ = false;
  dynamic_obstacle_in_zone3_ = false;
  std::unordered_set<std::size_t> static_scan_indices;
  for (const auto & track : tracks_) {
    if (track.motion_class != MotionClass::STATIC) {continue;}
    for (const auto & point : track.points) {
      if (!point.in_roi ||
        traversable_ramp_indices_.count(point.scan_index) > 0U)
      {
        continue;
      }
      static_scan_indices.insert(point.scan_index);
      const double s = longitudinal_distance(point);
      if (s < 0.0 || s > caution_zone_m_) {continue;}
      if (s <= stop_zone_m_) {
        static_obstacle_in_zone1_ = true;
      } else if (s <= slow_zone_m_) {
        static_obstacle_in_zone2_ = true;
      } else {
        static_obstacle_in_zone3_ = true;
      }
    }
  }
  for (const auto & point : latest_points_) {
    if (!point.in_roi || traversable_ramp_indices_.count(point.scan_index) > 0U ||
      static_scan_indices.count(point.scan_index) > 0U)
    {
      continue;
    }
    const double s = longitudinal_distance(point);
    if (s > stop_zone_m_ && s <= slow_zone_m_) {
      dynamic_obstacle_in_zone2_ = true;
    } else if (s > slow_zone_m_ && s <= caution_zone_m_) {
      dynamic_obstacle_in_zone3_ = true;
    }
  }
}

std::pair<int, std::string> MotionDetectorNode::computeLidarDriveCommand() const
{
  if (tf_error_) {return {0, "tf_error_fail_safe"};}
  if (zone1_obstacle_detected_ || static_obstacle_in_zone1_) {
    return {0, "stop_zone_obstacle"};
  }
  if (zone2_obstacle_detected_ && dynamic_obstacle_in_zone2_) {
    return {1, "red_yellow_dynamic_slow"};
  }
  if (static_obstacle_in_zone2_) {return {2, "red_yellow_static_drive"};}
  if (zone3_obstacle_detected_ && dynamic_obstacle_in_zone3_) {
    return {2, "yellow_blue_dynamic_drive"};
  }
  if (static_obstacle_in_zone3_) {return {3, "yellow_blue_static_accel"};}
  return {2, "clear_drive"};
}

void MotionDetectorNode::updateStableLidarDrive()
{
  const rclcpp::Time current_time = now();
  const bool immediate_tf = tf_error_ && immediate_stop_on_tf_error_;
  const bool immediate_stop = raw_lidar_drive_ == 0 &&
    raw_drive_reason_ == "stop_zone_obstacle" && immediate_stop_on_zone0_;
  if (!lidar_drive_filter_enable_) {
    stable_lidar_drive_ = raw_lidar_drive_;
    stable_drive_since_ = current_time;
    pending_lidar_drive_ = stable_lidar_drive_;
    pending_drive_since_ = current_time;
    pending_drive_count_ = 0;
    stable_drive_reason_ = raw_drive_reason_;
    drive_reason_ = stable_drive_reason_;
    return;
  }
  if (immediate_tf || immediate_stop) {
    if (stable_lidar_drive_ != 0) {
      stable_lidar_drive_ = 0;
      stable_drive_since_ = current_time;
    }
    last_stop_time_ = current_time;
    pending_lidar_drive_ = 0;
    pending_drive_since_ = current_time;
    pending_drive_count_ = 0;
    stable_drive_reason_ = raw_drive_reason_;
    drive_reason_ = stable_drive_reason_;
    return;
  }
  const double stable_elapsed = std::max(0.0, (current_time - stable_drive_since_).seconds());
  if (stable_lidar_drive_ == 0) {
    const double stop_elapsed = last_stop_time_.nanoseconds() == 0 ?
      stable_elapsed : std::max(0.0, (current_time - last_stop_time_).seconds());
    if (stop_elapsed < hold_stop_duration_sec_) {
      stable_drive_reason_ = "holding_stop_for_1s";
      drive_reason_ = stable_drive_reason_;
      return;
    }
    if (stable_elapsed < min_hold_duration_sec_) {
      stable_drive_reason_ = "holding_stable_min_1s";
      drive_reason_ = stable_drive_reason_;
      return;
    }
  }
  if (raw_lidar_drive_ == stable_lidar_drive_) {
    pending_lidar_drive_ = stable_lidar_drive_;
    pending_drive_since_ = current_time;
    pending_drive_count_ = 0;
    stable_drive_reason_ = raw_drive_reason_;
    drive_reason_ = stable_drive_reason_;
    return;
  }
  if (raw_lidar_drive_ != pending_lidar_drive_) {
    pending_lidar_drive_ = raw_lidar_drive_;
    pending_drive_since_ = current_time;
    pending_drive_count_ = 1;
    stable_drive_reason_ = "pending_new_drive_state";
    drive_reason_ = stable_drive_reason_;
    return;
  }
  ++pending_drive_count_;
  const double pending_elapsed = std::max(0.0, (current_time - pending_drive_since_).seconds());
  if (pending_elapsed >= candidate_confirm_duration_sec_ &&
    stable_elapsed >= min_hold_duration_sec_)
  {
    stable_lidar_drive_ = pending_lidar_drive_;
    stable_drive_since_ = current_time;
    pending_drive_count_ = 0;
    stable_drive_reason_ = raw_drive_reason_;
  } else {
    stable_drive_reason_ = "holding_previous_until_1s_confirmed";
  }
  drive_reason_ = stable_drive_reason_;
}

void MotionDetectorNode::publishModeAndFinalState()
{
  const std::string mode = is_t_parking_mode_ ? "T_PARKING" :
    (is_parallel_parking_mode_ ? "PARALLEL_PARKING" : (is_parking_mode_ ? "PARKING_UNKNOWN" : "NORMAL_DRIVE"));
  std_msgs::msg::Bool b; std_msgs::msg::String s; std_msgs::msg::Float32 f; std_msgs::msg::Int32 i;
  b.data = is_t_parking_mode_; parking_t_mode_pub_->publish(b);
  b.data = is_parallel_parking_mode_; paring_mode_pub_->publish(b);
  s.data = mode; parking_mode_text_pub_->publish(s);
  b.data = space_found_; parking_space_found_pub_->publish(b);
  s.data = selected_side_; parking_selected_side_pub_->publish(s);
  s.data = selected_slot_; parking_selected_slot_pub_->publish(s);
  b.data = ready_to_start_; parking_ready_pub_->publish(b);
  f.data = rear_axle_entry_x_m_; parking_entry_x_pub_->publish(f);
  f.data = rear_axle_entry_y_m_; parking_entry_y_pub_->publish(f);
  if (parking_done_pub_) {b.data = parking_done_; parking_done_pub_->publish(b);}
  const auto drive_command = computeLidarDriveCommand();
  raw_lidar_drive_ = drive_command.first;
  raw_drive_reason_ = drive_command.second;
  updateStableLidarDrive();
  lidar_drive_ = stable_lidar_drive_;
  if (lidar_role_ == "front") {
    switch (stable_lidar_drive_) {
      case 0:
        stop_required_ = true; slow_required_ = false; roi_risk_level_ = 3;
        current_speed_limit_mps_ = stop_speed_limit_mps_;
        break;
      case 1:
        stop_required_ = false; slow_required_ = true; roi_risk_level_ = 2;
        current_speed_limit_mps_ = slow_speed_limit_mps_;
        break;
      case 3:
        stop_required_ = false; slow_required_ = false; roi_risk_level_ = 0;
        current_speed_limit_mps_ = accel_speed_limit_mps_;
        break;
      case 2:
      default:
        stop_required_ = false; slow_required_ = false; roi_risk_level_ = 0;
        current_speed_limit_mps_ = default_speed_limit_mps_;
        break;
    }
  }
  b.data = stop_required_; final_stop_pub_->publish(b);
  b.data = slow_required_; final_slow_pub_->publish(b);
  i.data = roi_risk_level_; final_risk_pub_->publish(i);
  f.data = current_speed_limit_mps_; final_speed_pub_->publish(f);
  std::ostringstream lidar_drive_text_stream;
  lidar_drive_text_stream << std::fixed << std::setprecision(2)
                          << static_cast<double>(stable_lidar_drive_);
  const std::string lidar_drive_text = lidar_drive_text_stream.str();
  if (lidar_drive_pub_) {
    f.data = static_cast<float>(stable_lidar_drive_);
    lidar_drive_pub_->publish(f);
  }
  if (lidar_drive_text_pub_) {
    s.data = lidar_drive_text;
    lidar_drive_text_pub_->publish(s);
  }
  const bool rear_ignored = lidar_role_ == "rear" && !is_parking_mode_;
  const int obstacle_zone = (zone1_obstacle_detected_ || static_obstacle_in_zone1_) ? 0 :
    ((zone2_obstacle_detected_ || static_obstacle_in_zone2_) ? 1 :
    ((zone3_obstacle_detected_ || static_obstacle_in_zone3_) ? 2 : -1));
  const std::string zone_text = (zone1_obstacle_detected_ || static_obstacle_in_zone1_) ? "stop" :
    ((static_obstacle_in_zone2_ || static_obstacle_in_zone3_) ? "non_stop_static" :
    (zone2_obstacle_detected_ ? "red_yellow" : (zone3_obstacle_detected_ ? "yellow_blue" : "clear")));
  const rclcpp::Time status_time = now();
  const double stable_drive_age_sec = std::max(
    0.0, (status_time - stable_drive_since_).seconds());
  const double pending_drive_age_sec = std::max(
    0.0, (status_time - pending_drive_since_).seconds());
  const double last_stop_age_sec = last_stop_time_.nanoseconds() == 0 ? -1.0 :
    std::max(0.0, (status_time - last_stop_time_).seconds());
  const char * drive_command_text = stable_lidar_drive_ == 0 ? "STOP" :
    (stable_lidar_drive_ == 1 ? "SLOW" : (stable_lidar_drive_ == 3 ? "ACCEL" : "DRIVE"));
  std::ostringstream text_stream;
  text_stream << std::fixed << std::setprecision(2)
              << "[LIDAR] role=" << lidar_role_ << " mode=" << mode
              << " raw=" << raw_lidar_drive_ << " stable=" << stable_lidar_drive_
              << " zone=" << zone_text << " drive=" << static_cast<double>(lidar_drive_)
              << " pending=" << pending_lidar_drive_ << " p_age=" << pending_drive_age_sec
              << "s s_age=" << stable_drive_age_sec << "s cmd=" << drive_command_text
              << " speed=" << current_speed_limit_mps_
              << " reason=" << drive_reason_
              << " ignored=" << (rear_ignored ? "true" : "false");
  s.data = text_stream.str(); final_status_text_pub_->publish(s);
  std::ostringstream json;
  json << "{\"lidar_role\":\"" << lidar_role_ << "\",\"mode\":\"" << mode
       << "\",\"parking_t_mode\":" << (is_t_parking_mode_ ? "true" : "false")
       << ",\"paring_mode\":" << (is_parallel_parking_mode_ ? "true" : "false")
       << ",\"parking_mode_active\":" << (is_parking_mode_ ? "true" : "false")
       << ",\"rear_ignored\":" << (rear_ignored ? "true" : "false")
       << ",\"front_distance_m\":" << (lidar_role_ == "front" ? nearest_distance_m_ : -1.0)
       << ",\"rear_distance_m\":" << (lidar_role_ == "rear" ? nearest_distance_m_ : -1.0)
       << ",\"nearest_distance_m\":" << nearest_distance_m_ << ",\"obstacle_zone\":" << obstacle_zone
       << ",\"final_stop_required\":" << (stop_required_ ? "true" : "false")
       << ",\"final_slow_required\":" << (slow_required_ ? "true" : "false")
       << ",\"final_risk_level\":" << roi_risk_level_ << ",\"final_speed_limit_mps\":" << current_speed_limit_mps_
       << ",\"selected_side\":\"" << selected_side_ << "\",\"selected_slot\":\"" << selected_slot_
       << "\",\"space_found\":" << (space_found_ ? "true" : "false") << ",\"ready_to_start\":" << (ready_to_start_ ? "true" : "false")
       << ",\"rear_axle_entry_x_m\":" << rear_axle_entry_x_m_ << ",\"rear_axle_entry_y_m\":" << rear_axle_entry_y_m_
       << ",\"parking_done\":" << (parking_done_ ? "true" : "false") << ",\"static_obstacle_held\":" << (static_obstacle_held_ ? "true" : "false")
       << ",\"zone1_obstacle_detected\":" << (zone1_obstacle_detected_ ? "true" : "false")
       << ",\"zone2_obstacle_detected\":" << (zone2_obstacle_detected_ ? "true" : "false")
       << ",\"zone3_obstacle_detected\":" << (zone3_obstacle_detected_ ? "true" : "false")
       << ",\"obstacle_in_stop_zone\":" << (zone1_obstacle_detected_ ? "true" : "false")
       << ",\"obstacle_in_red_yellow_zone\":" << (zone2_obstacle_detected_ ? "true" : "false")
       << ",\"obstacle_in_yellow_blue_zone\":" << (zone3_obstacle_detected_ ? "true" : "false")
       << ",\"dynamic_obstacle_in_red_yellow_zone\":" << (dynamic_obstacle_in_zone2_ ? "true" : "false")
       << ",\"dynamic_obstacle_in_yellow_blue_zone\":" << (dynamic_obstacle_in_zone3_ ? "true" : "false")
       << ",\"static_obstacle_in_zone1\":" << (static_obstacle_in_zone1_ ? "true" : "false")
       << ",\"static_obstacle_in_zone2\":" << (static_obstacle_in_zone2_ ? "true" : "false")
       << ",\"static_obstacle_in_zone3\":" << (static_obstacle_in_zone3_ ? "true" : "false")
       << ",\"static_obstacle_non_stop_zone\":" <<
          ((static_obstacle_in_zone2_ || static_obstacle_in_zone3_) ? "true" : "false")
       << ",\"static_obstacle_in_stop_zone\":" << (static_obstacle_in_zone1_ ? "true" : "false")
       << ",\"static_obstacle_in_red_yellow_zone\":" << (static_obstacle_in_zone2_ ? "true" : "false")
       << ",\"static_obstacle_in_yellow_blue_zone\":" << (static_obstacle_in_zone3_ ? "true" : "false")
       << ",\"nearest_stop_zone_distance_m\":" << nearest_stop_zone_distance_m_
       << ",\"nearest_red_yellow_zone_distance_m\":" << nearest_red_yellow_zone_distance_m_
       << ",\"nearest_yellow_blue_zone_distance_m\":" << nearest_yellow_blue_zone_distance_m_
       << ",\"tf_error\":" << (tf_error_ ? "true" : "false")
       << ",\"raw_lidar_drive\":" << raw_lidar_drive_
       << ",\"stable_lidar_drive\":" << stable_lidar_drive_
       << ",\"published_lidar_drive\":" << static_cast<double>(lidar_drive_) << ".0"
       << ",\"lidar_drive\":" << static_cast<double>(lidar_drive_) << ".0"
       << ",\"lidar_drive_text\":\"" << lidar_drive_text << "\""
       << ",\"pending_lidar_drive\":" << pending_lidar_drive_
       << ",\"pending_count\":" << pending_drive_count_
       << ",\"confirm_frames\":" << lidar_drive_confirm_frames_
       << ",\"raw_drive_reason\":\"" << raw_drive_reason_
       << "\",\"stable_drive_reason\":\"" << stable_drive_reason_
       << "\",\"drive_reason\":\"" << drive_reason_
       << "\",\"stable_drive_age_sec\":" << stable_drive_age_sec
       << ",\"pending_drive_age_sec\":" << pending_drive_age_sec
       << ",\"min_hold_duration_sec\":" << min_hold_duration_sec_
       << ",\"candidate_confirm_duration_sec\":" << candidate_confirm_duration_sec_
       << ",\"hold_stop_duration_sec\":" << hold_stop_duration_sec_
       << ",\"last_stop_age_sec\":" << last_stop_age_sec
       << ",\"drive_command_text\":\"" << drive_command_text << "\"}";
  s.data = json.str(); final_status_json_pub_->publish(s);
}

visualization_msgs::msg::Marker MotionDetectorNode::createPointMarker(
  const std_msgs::msg::Header & header,
  const MotionClass motion_class,
  const std::string & marker_namespace,
  const int32_t marker_id) const
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = point_marker_scale_;
  marker.scale.y = point_marker_scale_;
  marker.color.a = 1.0F;
  marker.color.r = motion_class == MotionClass::DYNAMIC ? 1.0F : 0.0F;
  marker.color.g = motion_class == MotionClass::STATIC ? 1.0F : 0.0F;
  marker.color.b = 0.0F;
  marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
  return marker;
}

visualization_msgs::msg::Marker MotionDetectorNode::createDeleteAllMarker(
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

const char * MotionDetectorNode::motionClassToString(
  const MotionClass motion_class) const
{
  return motion_class == MotionClass::STATIC ? "STATIC" : "DYNAMIC";
}

void MotionDetectorNode::applyEgoSpeedCompensation(double & vx, double & vy) const
{
  (void)vy;
  if (use_ego_speed_compensation_) {
    vx += ego_speed_mps_;
  }
}

int64_t MotionDetectorNode::logThrottleMilliseconds() const
{
  return static_cast<int64_t>(log_throttle_sec_ * 1000.0);
}

}  // namespace lidar_motion_detector

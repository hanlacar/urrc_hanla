#ifndef LIDAR_MOTION_DETECTOR__MOTION_DETECTOR_NODE_HPP_
#define LIDAR_MOTION_DETECTOR__MOTION_DETECTOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "lidar_motion_detector/types.hpp"

namespace lidar_motion_detector
{

class MotionDetectorNodeTest;

class MotionDetectorNode : public rclcpp::Node
{
public:
  explicit MotionDetectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class MotionDetectorNodeTest;

  void declareParameters();
  void loadParameters();
  void validateParameters();
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  bool lookupScanToTargetTransform(
    const sensor_msgs::msg::LaserScan & scan,
    geometry_msgs::msg::TransformStamped & transform);
  Point2D transformLaserPointToTarget(
    double laser_x,
    double laser_y,
    std::size_t scan_index,
    const geometry_msgs::msg::TransformStamped & transform) const;
  bool transformScanPoint(
    double range,
    double angle,
    std::size_t scan_index,
    const geometry_msgs::msg::TransformStamped & transform,
    Point2D & out_point) const;
  void forceSafetyStop();
  void preprocessScan(
    const sensor_msgs::msg::LaserScan & scan,
    std::vector<Point2D> & points);
  void clusterPoints(
    const std::vector<Point2D> & points,
    std::vector<Cluster> & clusters);
  void computeClusterProperties(Cluster & cluster);
  void updateTracks(
    const std::vector<Cluster> & clusters,
    const rclcpp::Time & stamp);
  void classifyTrack(Track & track);
  void removeStaleTracks();
  void publishMarkers(const std_msgs::msg::Header & header);
  void publishEmptyMarkers(const std_msgs::msg::Header & header);
  void onSteeringAngle(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void onEgoSpeed(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void onCameraRamp(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void updateEffectiveRoiLength();
  double currentRoiLength() const;
  double currentStopZone() const;
  double currentSlowZone() const;
  double currentCautionZone() const;
  void evaluateRampCandidates();
  void publishRampState(const std_msgs::msg::Header & header);
  double computeWarpedRoiAngle() const;
  void updateWarpedRoiGeometry();
  void pointToRoiLocal(
    const Point2D & point,
    double center_angle,
    double & local_x,
    double & local_y) const;
  bool isPointInWarpedRoi(const Point2D & point) const;
  double projectOntoWarpedCenterline(const Point2D & point) const;
  bool isLocalPointInPolygon(double x, double y) const;
  double computeCurrentRoiCenterAngle() const;
  bool isPointInRoi(const Point2D & point) const;
  bool isPointEligibleForDriveSafety(const Point2D & point) const;
  bool usesFrontSensorSafetyGeometry() const;
  double sensorOriginDistance(const Point2D & point) const;
  bool isPointInSectorRoi(const Point2D & point, double center_angle) const;
  bool isPointInCorridorRoi(const Point2D & point, double center_angle) const;
  double normalizeAngle(double angle_rad) const;
  double shortestAngularDistance(double a, double b) const;
  void publishRoiMarker(const std_msgs::msg::Header & header);
  geometry_msgs::msg::Point computeDriveRoiStartMidpoint() const;
  double computeDriveRoiHeadingRad() const;
  RoiZoneCounts countRoiZonePoints() const;
  void updateRiskState(const std_msgs::msg::Header & header);
  void publishRiskState(const std_msgs::msg::Header & header);
  void updateModeState();
  void evaluateParkingState();
  void publishModeAndFinalState();
  std::pair<int, std::string> computeLidarDriveCommand() const;
  void updateDriveObstacleFlags();
  void updateStableLidarDrive();
  double computeArcLengthInRoi(const Point2D & point) const;
  visualization_msgs::msg::Marker createPointMarker(
    const std_msgs::msg::Header & header,
    MotionClass motion_class,
    const std::string & marker_namespace,
    int32_t marker_id) const;
  visualization_msgs::msg::Marker createDeleteAllMarker(
    const std_msgs::msg::Header & header) const;
  const char * motionClassToString(MotionClass motion_class) const;
  void applyEgoSpeedCompensation(double & vx, double & vy) const;
  int64_t logThrottleMilliseconds() const;

  std::string input_scan_topic_;
  std::string static_points_topic_;
  std::string dynamic_points_topic_;
  std::string all_motion_points_topic_;
  std::string roi_marker_topic_;
  std::string stop_required_topic_;
  std::string slow_required_topic_;
  std::string roi_risk_level_topic_;
  std::string speed_limit_topic_;
  std::string steering_angle_topic_;
  std::string steering_angle_unit_;
  std::string ego_speed_topic_;
  std::string ego_speed_unit_;
  std::string ramp_candidate_topic_;
  std::string traversable_ramp_topic_;
  std::string imu_topic_;
  std::string camera_ramp_topic_;
  std::string unknown_roi_object_policy_;
  std::string source_scan_frame_;  // front_laser by default
  std::string target_frame_;
  std::string output_frame_;
  std::string active_output_frame_;
  std::string lidar_role_;
  std::string output_namespace_;
  std::string mission_state_{"NORMAL_DRIVE"};
  bool publish_lidar_drive_command_{false};

  double min_range_;
  double max_range_;
  RoiConfig roi_config_;
  double current_steering_angle_rad_;
  bool enable_roi_marker_;
  bool stop_on_roi_obstacle_;
  double roi_zone_step_m_;
  double stop_zone_m_;
  double slow_zone_m_;
  double caution_zone_m_;
  int min_stop_points_;
  int min_slow_points_;
  int min_caution_points_;
  int stop_confirm_frames_;
  int stop_release_frames_;
  int slow_confirm_frames_;
  int slow_release_frames_;
  double steering_sign_;
  double wheelbase_m_;
  double steering_deadband_rad_;
  double max_steering_angle_rad_;
  double default_speed_limit_mps_;
  double caution_speed_limit_mps_;
  double slow_speed_limit_mps_;
  double stop_speed_limit_mps_;
  int curved_roi_marker_segments_;
  bool use_speed_based_roi_length_;
  double min_roi_length_m_;
  double max_roi_length_m_;
  double roi_time_horizon_sec_;
  bool use_ratio_zones_;
  double stop_zone_ratio_;
  double slow_zone_ratio_;
  double caution_zone_ratio_;
  bool enable_ramp_candidate_filter_;
  bool allow_ramp_pass_with_lidar_only_;
  int ramp_min_continuous_points_;
  double ramp_max_lateral_width_m_;
  double ramp_min_longitudinal_length_m_;
  double ramp_candidate_speed_limit_mps_;
  bool use_imu_pitch_for_ramp_;
  double ramp_pitch_threshold_rad_;
  bool use_camera_ramp_classifier_;
  bool use_warped_rectangle_roi_;
  double warped_roi_angle_gain_;
  double max_warped_roi_angle_rad_;
  int warped_roi_segments_;
  double warped_roi_deadband_rad_;
  bool use_ackermann_roi_shape_;
  bool use_tf_transform_;
  bool use_lidar_tf_as_roi_origin_;
  double tf_lookup_timeout_sec_;
  double tf_warn_throttle_sec_;
  bool allow_no_tf_fallback_;
  double cluster_distance_threshold_;
  double cluster_distance_scale_;
  int min_cluster_points_;
  double max_cluster_width_;
  double track_match_distance_;
  int max_missed_frames_;
  double velocity_filter_alpha_;
  double dynamic_speed_threshold_;
  int dynamic_confirm_frames_;
  double static_speed_threshold_;
  int static_confirm_frames_;
  double point_marker_scale_;
  double marker_lifetime_sec_;
  double ego_speed_mps_;
  bool use_ego_speed_compensation_;
  double log_throttle_sec_;
  bool parking_start_{false};
  bool parking_t_request_{false};
  bool parking_parallel_request_{false};
  bool camera_parking_mode_{false};
  bool gps_parking_mode_{false};
  bool is_normal_drive_{true};
  bool is_parking_mode_{false};
  bool is_t_parking_mode_{false};
  bool is_parallel_parking_mode_{false};
  bool parking_side_roi_enable_{true};
  double parking_side_width_scale_{1.5};
  double parking_side_base_half_angle_rad_{0.5235987756};
  double parking_side_search_length_m_{2.5};
  int parking_blocked_point_threshold_{8};
  int parking_free_point_threshold_{3};
  int parking_confirm_count_{3};
  int parking_marker_segments_{24};
  bool parking_apex_from_drive_start_{true};
  bool parking_rotate_from_drive_heading_{true};
  bool parking_show_current_role_only_{true};
  double normal_roi_start_x_m_{0.0};
  int parking_selection_counter_{0};
  std::string selected_side_{"unknown"};
  std::string selected_slot_{"unknown"};
  bool space_found_{false};
  double t_entry_x_m_{0.5};
  double t_entry_lateral_offset_m_{0.75};
  double parallel_entry_x_m_{0.8};
  double parallel_entry_lateral_offset_m_{0.65};
  double rear_axle_offset_from_base_m_{-0.13};
  double rear_axle_entry_x_m_{0.0};
  double rear_axle_entry_y_m_{0.0};
  bool ready_to_start_{false};
  bool enable_rear_distance_done_{true};
  double rear_done_distance_m_{0.5};
  bool parking_done_{false};
  double nearest_distance_m_{-1.0};
  bool static_obstacle_hold_enable_{true};
  int static_obstacle_hold_frames_{3};
  int static_obstacle_release_frames_{3};
  bool static_obstacle_held_{false};
  bool side_obstacle_detected_{false};
  bool tf_error_{false};
  int lidar_drive_{1};
  std::string drive_reason_{"clear_drive"};
  std::string raw_drive_reason_{"clear_drive"};
  std::string stable_drive_reason_{"clear_drive"};
  int raw_lidar_drive_{2};
  int stable_lidar_drive_{2};
  int pending_lidar_drive_{2};
  int pending_drive_count_{0};
  bool lidar_drive_filter_enable_{true};
  int lidar_drive_confirm_frames_{5};
  int lidar_drive_release_frames_{5};
  int lidar_drive_default_{2};
  bool immediate_stop_on_zone0_{true};
  bool immediate_stop_on_tf_error_{true};
  double min_hold_duration_sec_{1.0};
  double candidate_confirm_duration_sec_{1.0};
  double hold_stop_duration_sec_{1.0};
  rclcpp::Time stable_drive_since_;
  rclcpp::Time pending_drive_since_;
  rclcpp::Time last_stop_time_;
  bool zone1_obstacle_detected_{false};
  bool zone2_obstacle_detected_{false};
  bool zone3_obstacle_detected_{false};
  bool static_obstacle_in_zone1_{false};
  bool static_obstacle_in_zone2_{false};
  bool static_obstacle_in_zone3_{false};
  bool dynamic_obstacle_in_zone2_{false};
  bool dynamic_obstacle_in_zone3_{false};
  double accel_speed_limit_mps_{1.2};
  double nearest_stop_zone_distance_m_{-1.0};
  double nearest_red_yellow_zone_distance_m_{-1.0};
  double nearest_yellow_blue_zone_distance_m_{-1.0};
  double nearest_forward_safety_distance_m_{-1.0};

  uint32_t next_track_id_{1U};
  bool stop_required_{false};
  bool slow_required_{false};
  int roi_risk_level_{0};
  double current_speed_limit_mps_{1.0};
  double effective_roi_length_m_{1.5};
  double latest_imu_pitch_rad_{0.0};
  bool camera_ramp_detected_{false};
  bool ramp_candidate_{false};
  bool traversable_ramp_{false};
  int stop_counter_{0};
  int stop_clear_counter_{0};
  int slow_counter_{0};
  int slow_clear_counter_{0};
  std::vector<Point2D> latest_points_;
  std::vector<Track> tracks_;
  std::unordered_set<std::size_t> traversable_ramp_indices_;
  WarpedRoiGeometry warped_roi_geometry_;
  geometry_msgs::msg::TransformStamped scan_to_target_transform_;
  // TF-derived scan-frame origin expressed in target_frame.  Front drive
  // safety uses this as the origin of its forward hemisphere.
  double scan_sensor_origin_x_{0.0};
  double scan_sensor_origin_y_{0.0};
  double roi_origin_x_{0.0};
  double roi_origin_y_{0.0};
  double roi_heading_rad_{0.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_speed_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_ramp_subscription_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> mode_subscriptions_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    static_points_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    dynamic_points_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    all_motion_points_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    roi_marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_required_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slow_required_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr roi_risk_level_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_candidate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr traversable_ramp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_t_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr paring_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parking_mode_text_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_space_found_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parking_selected_side_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parking_selected_slot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr parking_entry_x_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr parking_entry_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr final_status_text_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr final_status_json_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr final_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr final_slow_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr final_risk_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr final_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lidar_drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lidar_drive_text_pub_;
};

}  // namespace lidar_motion_detector

#endif  // LIDAR_MOTION_DETECTOR__MOTION_DETECTOR_NODE_HPP_

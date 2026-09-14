#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "lidar_motion_detector/drive_safety_geometry.hpp"
#include "lidar_motion_detector/motion_detector_node.hpp"

namespace lidar_motion_detector
{

class MotionDetectorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<MotionDetectorNode>();
    node_->lidar_role_ = "front";
    node_->min_stop_points_ = 1;
    node_->min_slow_points_ = 1;
    node_->min_caution_points_ = 1;
    node_->stop_confirm_frames_ = 1;
    node_->slow_confirm_frames_ = 1;
    node_->stop_release_frames_ = 1;
    node_->slow_release_frames_ = 1;
    node_->static_obstacle_hold_enable_ = true;
    node_->roi_config_.use_roi_filter = true;
    node_->roi_config_.roi_mode = "corridor";
    // The front scanner is mounted 0.73 m ahead of base_link.  Drive safety
    // distances and the hemisphere filter must use this TF-derived origin.
    node_->scan_sensor_origin_x_ = 0.73;
    node_->scan_sensor_origin_y_ = 0.0;
    node_->roi_origin_x_ = 0.73;
    node_->roi_origin_y_ = 0.0;
    node_->roi_heading_rad_ = 0.0;
    node_->normal_roi_start_x_m_ = 0.0;
    node_->current_steering_angle_rad_ = 0.0;
    node_->tf_error_ = false;
    node_->ramp_candidate_ = false;
    node_->traversable_ramp_ = false;
  }

  Point2D point(double x, double y, double arc_length, std::size_t index) const
  {
    return Point2D{
      x, y, std::hypot(x, y), std::atan2(y, x), index, true, arc_length,
      TerrainClass::OBSTACLE};
  }

  Point2D sensorPoint(
    double distance, double angle_rad, std::size_t index = 0U) const
  {
    return point(
      0.73 + distance * std::cos(angle_rad),
      distance * std::sin(angle_rad), distance, index);
  }

  void setPoints(const std::vector<Point2D> & points)
  {
    node_->latest_points_ = points;
  }

  void setStaticTrack(const std::vector<Point2D> & points)
  {
    Track track;
    track.motion_class = MotionClass::STATIC;
    track.points = points;
    node_->tracks_ = {track};
  }

  RoiZoneCounts countZones() const
  {
    return node_->countRoiZonePoints();
  }

  void updateRisk()
  {
    std_msgs::msg::Header header;
    node_->updateRiskState(header);
  }

  void updateDriveFlags()
  {
    node_->updateDriveObstacleFlags();
  }

  void setSteering(double steering_rad)
  {
    node_->current_steering_angle_rad_ = steering_rad;
  }

  bool stopRequired() const {return node_->stop_required_;}
  bool slowRequired() const {return node_->slow_required_;}
  bool zone1Detected() const {return node_->zone1_obstacle_detected_;}
  bool zone2Detected() const {return node_->zone2_obstacle_detected_;}
  bool zone3Detected() const {return node_->zone3_obstacle_detected_;}
  bool staticZone1Detected() const {return node_->static_obstacle_in_zone1_;}
  bool staticObstacleHeld() const {return node_->static_obstacle_held_;}
  double nearestStopDistance() const {return node_->nearest_stop_zone_distance_m_;}
  double nearestSlowDistance() const
  {
    return node_->nearest_red_yellow_zone_distance_m_;
  }
  double nearestCautionDistance() const
  {
    return node_->nearest_yellow_blue_zone_distance_m_;
  }

  void selectRearParkingRole()
  {
    node_->lidar_role_ = "rear";
    node_->is_parking_mode_ = true;
    // Keep this fixture's rear parking geometry independent of the front
    // scanner mount used by the drive-safety cases.
    node_->scan_sensor_origin_x_ = 0.0;
    node_->scan_sensor_origin_y_ = 0.0;
    node_->roi_origin_x_ = 0.0;
  }

  void evaluateParking()
  {
    node_->evaluateParkingState();
  }

  double nearestParkingDistance() const {return node_->nearest_distance_m_;}

  std::shared_ptr<MotionDetectorNode> node_;
};

constexpr double kTestPi = 3.14159265358979323846;

TEST(ForwardHemisphereTest, IncludesFrontPoint)
{
  EXPECT_TRUE(isInForwardHemisphere(1.13, 0.0, 0.73, 0.0));
}

TEST(ForwardHemisphereTest, IncludesFrontLeftAtEightyNineDegrees)
{
  constexpr double origin_x = 0.73;
  EXPECT_TRUE(isInForwardHemisphere(
      origin_x + std::cos(89.0 * kTestPi / 180.0),
      std::sin(89.0 * kTestPi / 180.0), origin_x, 0.0));
}

TEST(ForwardHemisphereTest, IncludesFrontRightAtMinusEightyNineDegrees)
{
  constexpr double origin_x = 0.73;
  EXPECT_TRUE(isInForwardHemisphere(
      origin_x + std::cos(-89.0 * kTestPi / 180.0),
      std::sin(-89.0 * kTestPi / 180.0), origin_x, 0.0));
}

TEST(ForwardHemisphereTest, ExcludesRearPoint)
{
  EXPECT_FALSE(isInForwardHemisphere(0.55, 0.0, 0.73, 0.0));
}

TEST(ForwardHemisphereTest, ExcludesRearLeftAtNinetyOneDegrees)
{
  constexpr double origin_x = 0.73;
  EXPECT_FALSE(isInForwardHemisphere(
      origin_x + std::cos(91.0 * kTestPi / 180.0),
      std::sin(91.0 * kTestPi / 180.0), origin_x, 0.0));
}

TEST(ForwardHemisphereTest, ExcludesRearRightAtMinusNinetyOneDegrees)
{
  constexpr double origin_x = 0.73;
  EXPECT_FALSE(isInForwardHemisphere(
      origin_x + std::cos(-91.0 * kTestPi / 180.0),
      std::sin(-91.0 * kTestPi / 180.0), origin_x, 0.0));
}

TEST(ForwardHemisphereTest, MountPositionBoundaryUsesSensorOrigin)
{
  EXPECT_FALSE(isInForwardHemisphere(0.72, 0.0, 0.73, 0.0));
  EXPECT_TRUE(isInForwardHemisphere(0.74, 0.0, 0.73, 0.0));
}

TEST(ForwardHemisphereTest, SensorRelativeRightAngleBoundary)
{
  constexpr double origin_x = 0.73;
  EXPECT_TRUE(isInForwardHemisphere(
      origin_x + std::cos(89.0 * kTestPi / 180.0),
      std::sin(89.0 * kTestPi / 180.0), origin_x, 0.0));
  EXPECT_FALSE(isInForwardHemisphere(
      origin_x + std::cos(91.0 * kTestPi / 180.0),
      std::sin(91.0 * kTestPi / 180.0), origin_x, 0.0));
  EXPECT_TRUE(isInForwardHemisphere(
      origin_x + std::cos(-89.0 * kTestPi / 180.0),
      std::sin(-89.0 * kTestPi / 180.0), origin_x, 0.0));
  EXPECT_FALSE(isInForwardHemisphere(
      origin_x + std::cos(-91.0 * kTestPi / 180.0),
      std::sin(-91.0 * kTestPi / 180.0), origin_x, 0.0));
}

TEST_F(MotionDetectorNodeTest, FrontPointInsideStopZoneRequiresStop)
{
  setPoints({sensorPoint(0.40, 0.0)});
  const auto counts = countZones();
  EXPECT_EQ(counts.stop_points, 1U);

  updateRisk();
  EXPECT_TRUE(stopRequired());
}

TEST_F(MotionDetectorNodeTest, FrontSensorStopBoundaryUsesEuclideanDistance)
{
  setPoints({sensorPoint(0.49, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(stopRequired());
  EXPECT_NEAR(nearestStopDistance(), 0.49, 1.0e-12);

  setPoints({sensorPoint(0.51, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_TRUE(slowRequired());
  EXPECT_NEAR(nearestSlowDistance(), 0.51, 1.0e-12);
}

TEST_F(MotionDetectorNodeTest, FrontSensorBeyondSlowZoneIsNotEmergencyStopOrSlow)
{
  setPoints({sensorPoint(0.90, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_TRUE(slowRequired());

  setPoints({sensorPoint(1.10, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_FALSE(slowRequired());
  EXPECT_NEAR(nearestCautionDistance(), 1.10, 1.0e-12);
}

TEST_F(MotionDetectorNodeTest, FrontSensorStopZoneIgnoresSteeringWarpProgress)
{
  setSteering(0.6);
  setPoints({sensorPoint(0.40, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(stopRequired());
  EXPECT_NEAR(nearestStopDistance(), 0.40, 1.0e-12);
}

TEST_F(MotionDetectorNodeTest, FrontSensorStopZoneUsesFullForwardSemicircle)
{
  constexpr double degree_to_rad = kTestPi / 180.0;
  setPoints({sensorPoint(0.40, 89.0 * degree_to_rad)});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(stopRequired());
  EXPECT_NEAR(nearestStopDistance(), 0.40, 1.0e-12);

  setPoints({sensorPoint(0.40, -89.0 * degree_to_rad)});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(stopRequired());

  setPoints({sensorPoint(0.40, 91.0 * degree_to_rad)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_DOUBLE_EQ(nearestStopDistance(), -1.0);

  setPoints({sensorPoint(0.40, -91.0 * degree_to_rad)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_DOUBLE_EQ(nearestStopDistance(), -1.0);
}

TEST_F(MotionDetectorNodeTest, PointsAtSensorOriginBoundaryAreClassifiedBySensorRelativeX)
{
  setPoints({point(0.72, 0.0, 0.01, 0U)});
  EXPECT_EQ(countZones().stop_points, 0U);

  setPoints({point(0.74, 0.0, 0.01, 1U)});
  EXPECT_EQ(countZones().stop_points, 1U);
}

TEST_F(MotionDetectorNodeTest, RearOfFrontSensorNeverRequiresStopOrSlow)
{
  setPoints({point(0.55, 0.0, 0.18, 0U)});
  const auto counts = countZones();
  EXPECT_EQ(counts.stop_points, 0U);
  EXPECT_EQ(counts.slow_points, 0U);

  updateRisk();
  EXPECT_FALSE(stopRequired());
  EXPECT_FALSE(slowRequired());
}

TEST_F(MotionDetectorNodeTest, PointsBehindFrontSensorDoNotEnterSlowOrCautionZones)
{
  setPoints({
      point(0.55, 0.0, 0.18, 0U),
      point(0.65, 0.0, 0.08, 1U)});
  const auto counts = countZones();
  EXPECT_EQ(counts.slow_points, 0U);
  EXPECT_EQ(counts.caution_points, 0U);

  updateDriveFlags();
  EXPECT_FALSE(zone2Detected());
  EXPECT_FALSE(zone3Detected());
}

TEST_F(MotionDetectorNodeTest, FrontPointInsideSlowZoneRequiresSlow)
{
  setPoints({point(1.33, 0.0, 0.60, 0U)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_TRUE(slowRequired());
  EXPECT_TRUE(zone2Detected());
}

TEST_F(MotionDetectorNodeTest, FrontSensorSupportDoesNotSetNearestStopDistance)
{
  setPoints({point(0.55, 0.0, 0.18, 0U)});
  updateDriveFlags();
  EXPECT_DOUBLE_EQ(nearestStopDistance(), -1.0);
  EXPECT_FALSE(zone1Detected());
}

TEST_F(MotionDetectorNodeTest, FrontSensorSupportStaticTrackDoesNotEnterStaticStopOrHold)
{
  const auto rear_point = point(0.55, 0.0, 0.18, 0U);
  setPoints({rear_point});
  setStaticTrack({rear_point});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(staticZone1Detected());
  EXPECT_FALSE(staticObstacleHeld());
}

TEST_F(MotionDetectorNodeTest, FrontStaticTrackEntersStaticStopAndHold)
{
  const auto front_point = sensorPoint(0.40, 0.0);
  setPoints({front_point});
  setStaticTrack({front_point});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(staticZone1Detected());
  EXPECT_TRUE(staticObstacleHeld());
}

TEST_F(MotionDetectorNodeTest, FrontObstacleAtPointFourSetsFinalStopState)
{
  setPoints({sensorPoint(0.40, 0.0)});
  updateRisk();
  updateDriveFlags();
  EXPECT_TRUE(stopRequired());
  EXPECT_TRUE(zone1Detected());
}

TEST_F(MotionDetectorNodeTest, FrontSensorRelativeFourTenthsMetersSetsNearestStopDistance)
{
  setPoints({sensorPoint(0.40, 0.0)});
  updateDriveFlags();
  EXPECT_TRUE(zone1Detected());
  EXPECT_NEAR(nearestStopDistance(), 0.40, 1.0e-12);
}

TEST_F(MotionDetectorNodeTest, FrontSensorBehindSupportLeavesFinalStopClear)
{
  setPoints({point(0.55, 0.0, 0.18, 0U)});
  updateRisk();
  updateDriveFlags();
  EXPECT_FALSE(stopRequired());
  EXPECT_FALSE(zone1Detected());
  EXPECT_DOUBLE_EQ(nearestStopDistance(), -1.0);
}

TEST_F(MotionDetectorNodeTest, RearLidarRetainsParkingAndZoneBehavior)
{
  selectRearParkingRole();
  setPoints({point(-0.20, 0.0, 0.20, 0U)});
  const auto counts = countZones();
  EXPECT_EQ(counts.stop_points, 1U);

  evaluateParking();
  EXPECT_NEAR(nearestParkingDistance(), 0.20, 1.0e-12);
}

}  // namespace lidar_motion_detector

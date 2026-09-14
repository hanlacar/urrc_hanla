#ifndef LIDAR_MOTION_DETECTOR__DRIVE_SAFETY_GEOMETRY_HPP_
#define LIDAR_MOTION_DETECTOR__DRIVE_SAFETY_GEOMETRY_HPP_

#include <cmath>

namespace lidar_motion_detector
{

constexpr double kHalfPi = 1.57079632679489661923;

// Point and sensor-origin coordinates must already be transformed into
// base_link.  The forward direction is the vehicle's +x axis, so the vector
// from the TF-derived sensor origin (rather than base_link origin) is used.
inline bool isInForwardHemisphere(
  const double x_base,
  const double y_base,
  const double sensor_origin_x_base = 0.0,
  const double sensor_origin_y_base = 0.0)
{
  if (!std::isfinite(x_base) || !std::isfinite(y_base) ||
    !std::isfinite(sensor_origin_x_base) ||
    !std::isfinite(sensor_origin_y_base))
  {
    return false;
  }

  const double dx = x_base - sensor_origin_x_base;
  const double dy = y_base - sensor_origin_y_base;
  return dx > 0.0 && std::abs(std::atan2(dy, dx)) <= kHalfPi;
}

}  // namespace lidar_motion_detector

#endif  // LIDAR_MOTION_DETECTOR__DRIVE_SAFETY_GEOMETRY_HPP_

#pragma once

#include "drone_city_nav/types.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

struct LidarPoseMotionCompensationResult {
  Point2 position{};
  Point2 applied_shift{};
  double pose_lag_s{0.0};
  double latency_s{0.0};
  double signed_time_offset_s{0.0};
  double applied_shift_m{0.0};
  bool applied{false};
};

[[nodiscard]] inline bool finiteLidarMotionPoint(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] inline double sanitizedLidarMotionSeconds(const double seconds) noexcept {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return 0.0;
  }
  return std::clamp(seconds, 0.0, 1.0);
}

[[nodiscard]] inline LidarPoseMotionCompensationResult
compensateLidarPoseForLatency(const Point2 pose_position, const Point2 velocity,
                              const bool enabled, const bool velocity_valid,
                              const double pose_lag_s,
                              const double lidar_pose_latency_s) noexcept {
  LidarPoseMotionCompensationResult result{};
  result.position = pose_position;
  result.pose_lag_s = sanitizedLidarMotionSeconds(pose_lag_s);
  result.latency_s = sanitizedLidarMotionSeconds(lidar_pose_latency_s);

  if (!enabled || !velocity_valid || !finiteLidarMotionPoint(pose_position) ||
      !finiteLidarMotionPoint(velocity)) {
    return result;
  }

  result.signed_time_offset_s = result.pose_lag_s - result.latency_s;
  result.applied_shift = Point2{velocity.x * result.signed_time_offset_s,
                                velocity.y * result.signed_time_offset_s};
  result.position.x += result.applied_shift.x;
  result.position.y += result.applied_shift.y;
  result.applied_shift_m = std::hypot(result.applied_shift.x, result.applied_shift.y);
  result.applied = result.applied_shift_m > 0.0;
  return result;
}

} // namespace drone_city_nav

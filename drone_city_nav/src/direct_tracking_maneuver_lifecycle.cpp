#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

} // namespace

DirectTrackingManeuverLifecycle::DirectTrackingManeuverLifecycle(
    const DirectTrackingManeuverConfig& config)
    : config_{config} {
  if (!(config_.bearing_change_threshold_rad > 0.0) ||
      config_.bearing_change_threshold_rad > std::acos(-1.0) ||
      !std::isfinite(config_.minimum_closing_speed_mps) ||
      !(config_.closing_recovery_speed_mps > config_.minimum_closing_speed_mps) ||
      !(config_.no_closing_duration_s > 0.0) ||
      !(config_.minimum_reseed_interval_s >= 0.0)) {
    throw std::invalid_argument{"invalid direct tracking maneuver configuration"};
  }
}

DirectTrackingManeuverUpdate DirectTrackingManeuverLifecycle::update(
    const DirectTrackingManeuverObservation& observation) noexcept {
  DirectTrackingManeuverUpdate result{.reseed_generation = reseed_generation_};
  if (!observation.active || observation.stamp_ns <= 0 ||
      observation.line_of_sight_generation == 0U ||
      !finite(observation.interceptor_position) ||
      !finite(observation.interceptor_velocity) ||
      !finite(observation.target_position) || !finite(observation.target_velocity)) {
    resetEpisode();
    return result;
  }

  const double relative_x =
      observation.target_position.x - observation.interceptor_position.x;
  const double relative_y =
      observation.target_position.y - observation.interceptor_position.y;
  const double relative_z =
      observation.target_position.z - observation.interceptor_position.z;
  const double range_m = std::hypot(std::hypot(relative_x, relative_y), relative_z);
  const double horizontal_range_m = std::hypot(relative_x, relative_y);
  if (!(range_m > 1.0e-6) || !(horizontal_range_m > 1.0e-6)) {
    resetEpisode();
    return result;
  }

  const double bearing_rad = std::atan2(relative_y, relative_x);
  if (line_of_sight_generation_ != observation.line_of_sight_generation) {
    resetEpisode();
    line_of_sight_generation_ = observation.line_of_sight_generation;
    reference_bearing_rad_ = bearing_rad;
    reference_bearing_valid_ = true;
  }

  result.bearing_change_rad =
      reference_bearing_valid_
          ? std::remainder(bearing_rad - reference_bearing_rad_, 2.0 * std::acos(-1.0))
          : 0.0;
  const double relative_velocity_x =
      observation.interceptor_velocity.x - observation.target_velocity.x;
  const double relative_velocity_y =
      observation.interceptor_velocity.y - observation.target_velocity.y;
  const double relative_velocity_z =
      observation.interceptor_velocity.z - observation.target_velocity.z;
  result.closing_speed_mps =
      (relative_x * relative_velocity_x + relative_y * relative_velocity_y +
       relative_z * relative_velocity_z) /
      range_m;

  if (result.closing_speed_mps >= config_.closing_recovery_speed_mps) {
    no_closing_since_ns_ = 0;
    no_closing_reseeded_ = false;
  } else if (result.closing_speed_mps <= config_.minimum_closing_speed_mps) {
    if (no_closing_since_ns_ == 0) {
      no_closing_since_ns_ = observation.stamp_ns;
    }
    result.no_closing_duration_s =
        static_cast<double>(observation.stamp_ns - no_closing_since_ns_) * 1.0e-9;
  } else {
    no_closing_since_ns_ = 0;
  }

  const double since_reseed_s =
      last_reseed_stamp_ns_ > 0
          ? static_cast<double>(observation.stamp_ns - last_reseed_stamp_ns_) * 1.0e-9
          : config_.minimum_reseed_interval_s;
  if (since_reseed_s < config_.minimum_reseed_interval_s) {
    return result;
  }

  if (std::abs(result.bearing_change_rad) >= config_.bearing_change_threshold_rad) {
    result.reason = DirectTrackingReseedReason::kBearingChange;
    reference_bearing_rad_ = bearing_rad;
  } else if (!no_closing_reseeded_ && no_closing_since_ns_ > 0 &&
             result.no_closing_duration_s >= config_.no_closing_duration_s) {
    result.reason = DirectTrackingReseedReason::kNoClosing;
    no_closing_reseeded_ = true;
  }
  if (result.reason != DirectTrackingReseedReason::kNone) {
    if (result.reason == DirectTrackingReseedReason::kBearingChange &&
        no_closing_since_ns_ > 0) {
      no_closing_since_ns_ = observation.stamp_ns;
    }
    ++reseed_generation_;
    last_reseed_stamp_ns_ = observation.stamp_ns;
    result.reseed_generation = reseed_generation_;
    result.reseed_requested = true;
  }
  return result;
}

void DirectTrackingManeuverLifecycle::resetEpisode() noexcept {
  line_of_sight_generation_ = 0U;
  no_closing_since_ns_ = 0;
  last_reseed_stamp_ns_ = 0;
  reference_bearing_rad_ = 0.0;
  reference_bearing_valid_ = false;
  no_closing_reseeded_ = false;
}

const char*
directTrackingReseedReasonName(const DirectTrackingReseedReason reason) noexcept {
  switch (reason) {
    case DirectTrackingReseedReason::kNone:
      return "none";
    case DirectTrackingReseedReason::kBearingChange:
      return "bearing_change";
    case DirectTrackingReseedReason::kNoClosing:
      return "no_closing";
  }
  return "unknown";
}

} // namespace drone_city_nav

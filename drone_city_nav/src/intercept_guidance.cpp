#include "drone_city_nav/intercept_guidance.hpp"

#include <algorithm>
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

[[nodiscard]] double norm(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] Vec3 subtract(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] Point3 extrapolate(const Point3& position, const Vec3& velocity,
                                 const double time_s) noexcept {
  return Point3{position.x + velocity.x * time_s, position.y + velocity.y * time_s,
                position.z + velocity.z * time_s};
}

} // namespace

InterceptGuidance::InterceptGuidance(const InterceptGuidanceConfig& config)
    : config_{config} {
  if (!(config_.far_prediction_horizon_s >= config_.ahead_prediction_horizon_s) ||
      !(config_.ahead_prediction_horizon_s >= 0.0) ||
      !(config_.minimum_target_speed_mps >= 0.0) ||
      !(config_.ahead_enter_m > config_.ahead_exit_m) ||
      !(config_.ahead_corridor_enter_m > 0.0) ||
      !(config_.ahead_corridor_exit_m >= config_.ahead_corridor_enter_m) ||
      !(config_.horizon_smoothing_time_constant_s > 0.0)) {
    throw std::invalid_argument{"invalid intercept guidance configuration"};
  }
}

void InterceptGuidance::resetPredictionState() noexcept {
  smoothed_prediction_horizon_s_.reset();
  previous_update_stamp_ns_.reset();
  ahead_mode_ = false;
}

InterceptGuidanceResult InterceptGuidance::update(const TimedVehicleState& interceptor,
                                                  const TimedVehicleState& target,
                                                  const std::int64_t now_ns) {
  InterceptGuidanceResult result;
  if (!target.position_valid || target.stamp_ns <= 0 || now_ns <= 0 ||
      !finite(target.position)) {
    resetPredictionState();
    return result;
  }

  result.observed_position = target.position;
  result.observed_velocity =
      target.velocity_valid && finite(target.velocity) ? target.velocity : Vec3{};
  result.observation_stamp_ns = target.stamp_ns;
  result.valid = true;
  result.predicted_position = target.position;
  result.prediction_age_s =
      static_cast<double>(std::max<std::int64_t>(0, now_ns - target.stamp_ns)) * 1.0e-9;

  if (!target.velocity_valid || !finite(target.velocity)) {
    resetPredictionState();
    return result;
  }
  result.target_speed_mps = norm(target.velocity);
  if (result.target_speed_mps < config_.minimum_target_speed_mps) {
    resetPredictionState();
    return result;
  }

  const Vec3 direction{target.velocity.x / result.target_speed_mps,
                       target.velocity.y / result.target_speed_mps,
                       target.velocity.z / result.target_speed_mps};
  if (interceptor.position_valid && finite(interceptor.position)) {
    const Vec3 relative = subtract(interceptor.position, target.position);
    result.ahead_m = dot(relative, direction);
    const double relative_norm_squared = dot(relative, relative);
    result.cross_track_m = std::sqrt(
        std::max(0.0, relative_norm_squared - result.ahead_m * result.ahead_m));

    if (ahead_mode_) {
      if (result.ahead_m <= config_.ahead_exit_m ||
          result.cross_track_m >= config_.ahead_corridor_exit_m) {
        ahead_mode_ = false;
      }
    } else if (result.ahead_m >= config_.ahead_enter_m &&
               result.cross_track_m <= config_.ahead_corridor_enter_m) {
      ahead_mode_ = true;
    }
  } else {
    ahead_mode_ = false;
  }

  result.mode =
      ahead_mode_ ? InterceptGuidanceMode::kAheadLead : InterceptGuidanceMode::kFarLead;
  const double desired_horizon_s = ahead_mode_ ? config_.ahead_prediction_horizon_s
                                               : config_.far_prediction_horizon_s;
  if (!smoothed_prediction_horizon_s_ || !previous_update_stamp_ns_ ||
      now_ns <= *previous_update_stamp_ns_) {
    smoothed_prediction_horizon_s_ = desired_horizon_s;
  } else {
    const double elapsed_s =
        static_cast<double>(now_ns - *previous_update_stamp_ns_) * 1.0e-9;
    const double alpha =
        1.0 - std::exp(-elapsed_s / config_.horizon_smoothing_time_constant_s);
    *smoothed_prediction_horizon_s_ +=
        alpha * (desired_horizon_s - *smoothed_prediction_horizon_s_);
  }
  previous_update_stamp_ns_ = now_ns;
  result.prediction_horizon_s = *smoothed_prediction_horizon_s_;
  result.predicted_position =
      extrapolate(target.position, target.velocity,
                  result.prediction_age_s + result.prediction_horizon_s);
  if (!finite(result.predicted_position)) {
    resetPredictionState();
    result.mode = InterceptGuidanceMode::kDirect;
    result.predicted_position = target.position;
    result.prediction_horizon_s = 0.0;
  }
  return result;
}

const char* interceptGuidanceModeName(const InterceptGuidanceMode mode) noexcept {
  switch (mode) {
    case InterceptGuidanceMode::kDirect:
      return "direct";
    case InterceptGuidanceMode::kFarLead:
      return "far_lead";
    case InterceptGuidanceMode::kAheadLead:
      return "ahead_lead";
  }
  return "unknown";
}

} // namespace drone_city_nav

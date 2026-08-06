#include "drone_city_nav/intercept_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

[[nodiscard]] Vec3 rotateHorizontal(const Vec3& velocity,
                                    const double angle_rad) noexcept {
  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  return Vec3{velocity.x * cosine - velocity.y * sine,
              velocity.x * sine + velocity.y * cosine, velocity.z};
}

[[nodiscard]] double horizontalDistance(const Point3& first,
                                        const Point3& second) noexcept {
  return std::hypot(first.x - second.x, first.y - second.y);
}

[[nodiscard]] std::optional<double>
horizontalInterceptTime(const Point3& interceptor, const Point3& target,
                        const Vec3& target_velocity,
                        const double interceptor_speed_mps) noexcept {
  const double relative_x = target.x - interceptor.x;
  const double relative_y = target.y - interceptor.y;
  const double a = target_velocity.x * target_velocity.x +
                   target_velocity.y * target_velocity.y -
                   interceptor_speed_mps * interceptor_speed_mps;
  const double b =
      2.0 * (relative_x * target_velocity.x + relative_y * target_velocity.y);
  const double c = relative_x * relative_x + relative_y * relative_y;
  if (c <= 1.0e-12) {
    return 0.0;
  }
  constexpr double kEpsilon{1.0e-9};
  if (std::abs(a) <= kEpsilon) {
    if (std::abs(b) <= kEpsilon) {
      return std::nullopt;
    }
    const double solution = -c / b;
    return solution >= 0.0 ? std::optional<double>{solution} : std::nullopt;
  }
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return std::nullopt;
  }
  const double root = std::sqrt(discriminant);
  const double first = (-b - root) / (2.0 * a);
  const double second = (-b + root) / (2.0 * a);
  double best = std::numeric_limits<double>::infinity();
  if (first >= 0.0) {
    best = first;
  }
  if (second >= 0.0) {
    best = std::min(best, second);
  }
  return std::isfinite(best) ? std::optional<double>{best} : std::nullopt;
}

} // namespace

TargetVerticalPrediction
predictTargetVerticalMotion(const double initial_z_m, const double initial_velocity_mps,
                            const double elapsed_s, const double deceleration_mps2,
                            const FlightEnvelopeConfig& flight_envelope) noexcept {
  if (!std::isfinite(initial_z_m) || !std::isfinite(initial_velocity_mps) ||
      !std::isfinite(elapsed_s) || elapsed_s < 0.0 ||
      !std::isfinite(deceleration_mps2) || !(deceleration_mps2 > 0.0)) {
    return {};
  }
  const double speed_mps = std::abs(initial_velocity_mps);
  const double stopping_time_s = speed_mps / deceleration_mps2;
  const double motion_time_s = std::min(elapsed_s, stopping_time_s);
  const double signed_deceleration_mps2 =
      std::copysign(deceleration_mps2, initial_velocity_mps);
  const double predicted_z_m =
      initial_z_m + initial_velocity_mps * motion_time_s -
      0.5 * signed_deceleration_mps2 * motion_time_s * motion_time_s;
  const std::optional<double> bounded_z_m =
      clampToFlightEnvelope(predicted_z_m, flight_envelope);
  if (!bounded_z_m.has_value()) {
    return {};
  }
  double remaining_velocity_mps = std::copysign(
      std::max(0.0, speed_mps - deceleration_mps2 * elapsed_s), initial_velocity_mps);
  const bool envelope_limited = std::abs(*bounded_z_m - predicted_z_m) > 1.0e-9;
  if (envelope_limited &&
      ((*bounded_z_m <= flight_envelope.minimum_target_z_m &&
        remaining_velocity_mps < 0.0) ||
       (*bounded_z_m >= std::nextafter(flight_envelope.maximum_target_z_m,
                                       flight_envelope.minimum_target_z_m) &&
        remaining_velocity_mps > 0.0))) {
    remaining_velocity_mps = 0.0;
  }
  return TargetVerticalPrediction{.z_m = *bounded_z_m,
                                  .velocity_mps = remaining_velocity_mps,
                                  .envelope_limited = envelope_limited,
                                  .valid = true};
}

InterceptGuidance::InterceptGuidance(const InterceptGuidanceConfig& config)
    : config_{config} {
  if (!(config_.interceptor_speed_mps > 0.0) ||
      !(config_.minimum_prediction_horizon_s >= 0.0) ||
      !(config_.maximum_prediction_horizon_s >= config_.minimum_prediction_horizon_s) ||
      !(config_.ahead_maximum_prediction_horizon_s >=
        config_.minimum_prediction_horizon_s) ||
      config_.ahead_maximum_prediction_horizon_s >
          config_.maximum_prediction_horizon_s ||
      config_.fallback_prediction_horizon_s < config_.minimum_prediction_horizon_s ||
      config_.fallback_prediction_horizon_s > config_.maximum_prediction_horizon_s ||
      !(config_.minimum_target_speed_mps >= 0.0) ||
      !(config_.ahead_enter_m > config_.ahead_exit_m) ||
      !(config_.ahead_corridor_enter_m > 0.0) ||
      !(config_.ahead_corridor_exit_m >= config_.ahead_corridor_enter_m) ||
      !(config_.horizon_smoothing_time_constant_s > 0.0) ||
      !std::isfinite(config_.prediction_heading_offset_rad) ||
      std::abs(config_.prediction_heading_offset_rad) > std::acos(-1.0) ||
      !(config_.hypothesis_zero_distance_m >= 0.0) ||
      !(config_.hypothesis_full_distance_m > config_.hypothesis_zero_distance_m) ||
      !(config_.maximum_hypothesis_lateral_offset_m >= 0.0) ||
      !(config_.target_vertical_deceleration_mps2 > 0.0) ||
      !clampToFlightEnvelope(config_.target_flight_envelope.minimum_target_z_m,
                             config_.target_flight_envelope)
           .has_value()) {
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
  result.configured_heading_offset_rad = config_.prediction_heading_offset_rad;
  result.valid = true;
  result.predicted_position = target.position;
  result.prediction_age_s =
      static_cast<double>(std::max<std::int64_t>(0, now_ns - target.stamp_ns)) * 1.0e-9;

  if (!target.velocity_valid || !finite(target.velocity)) {
    resetPredictionState();
    const std::optional<double> bounded_z =
        clampToFlightEnvelope(target.position.z, config_.target_flight_envelope);
    if (!bounded_z.has_value()) {
      result.valid = false;
      return result;
    }
    result.predicted_position.z = *bounded_z;
    result.vertical_prediction_limited =
        std::abs(*bounded_z - target.position.z) > 1.0e-9;
    return result;
  }
  Point3 current_target =
      extrapolate(target.position, result.observed_velocity, result.prediction_age_s);
  const TargetVerticalPrediction current_vertical = predictTargetVerticalMotion(
      target.position.z, result.observed_velocity.z, result.prediction_age_s,
      config_.target_vertical_deceleration_mps2, config_.target_flight_envelope);
  if (!current_vertical.valid) {
    resetPredictionState();
    result.valid = false;
    return result;
  }
  current_target.z = current_vertical.z_m;
  Vec3 current_velocity = result.observed_velocity;
  current_velocity.z = current_vertical.velocity_mps;
  result.vertical_prediction_limited = current_vertical.envelope_limited;
  result.target_speed_mps = norm(current_velocity);
  if (result.target_speed_mps < config_.minimum_target_speed_mps) {
    resetPredictionState();
    result.predicted_position = current_target;
    return result;
  }

  Vec3 hypothesis_velocity = current_velocity;
  if (interceptor.position_valid && finite(interceptor.position)) {
    const double distance_m = horizontalDistance(interceptor.position, current_target);
    const double blend = std::clamp(
        (distance_m - config_.hypothesis_zero_distance_m) /
            (config_.hypothesis_full_distance_m - config_.hypothesis_zero_distance_m),
        0.0, 1.0);
    result.effective_heading_offset_rad = config_.prediction_heading_offset_rad * blend;
    hypothesis_velocity =
        rotateHorizontal(current_velocity, result.effective_heading_offset_rad);
  }

  const Vec3 direction{current_velocity.x / result.target_speed_mps,
                       current_velocity.y / result.target_speed_mps,
                       current_velocity.z / result.target_speed_mps};
  if (interceptor.position_valid && finite(interceptor.position)) {
    const Vec3 relative = subtract(interceptor.position, current_target);
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

  const std::optional<double> intercept_time =
      interceptor.position_valid && finite(interceptor.position)
          ? horizontalInterceptTime(interceptor.position, current_target,
                                    hypothesis_velocity, config_.interceptor_speed_mps)
          : std::nullopt;
  result.analytic_intercept_time_s =
      intercept_time.value_or(config_.fallback_prediction_horizon_s);
  double desired_horizon_s =
      std::clamp(result.analytic_intercept_time_s, config_.minimum_prediction_horizon_s,
                 config_.maximum_prediction_horizon_s);
  if (ahead_mode_) {
    desired_horizon_s =
        std::min(desired_horizon_s, config_.ahead_maximum_prediction_horizon_s);
  }
  result.mode = ahead_mode_ ? InterceptGuidanceMode::kAheadIntercept
                            : InterceptGuidanceMode::kAnalyticIntercept;
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
      extrapolate(current_target, hypothesis_velocity, result.prediction_horizon_s);
  const Point3 baseline_prediction =
      extrapolate(current_target, current_velocity, result.prediction_horizon_s);
  const double hypothesis_delta_x = result.predicted_position.x - baseline_prediction.x;
  const double hypothesis_delta_y = result.predicted_position.y - baseline_prediction.y;
  result.hypothesis_lateral_offset_m =
      std::hypot(hypothesis_delta_x, hypothesis_delta_y);
  if (result.hypothesis_lateral_offset_m >
          config_.maximum_hypothesis_lateral_offset_m &&
      result.hypothesis_lateral_offset_m > 1.0e-9) {
    const double scale = config_.maximum_hypothesis_lateral_offset_m /
                         result.hypothesis_lateral_offset_m;
    result.predicted_position.x = baseline_prediction.x + hypothesis_delta_x * scale;
    result.predicted_position.y = baseline_prediction.y + hypothesis_delta_y * scale;
    result.hypothesis_lateral_offset_m = config_.maximum_hypothesis_lateral_offset_m;
  }
  const TargetVerticalPrediction predicted_vertical = predictTargetVerticalMotion(
      target.position.z, result.observed_velocity.z,
      result.prediction_age_s + result.prediction_horizon_s,
      config_.target_vertical_deceleration_mps2, config_.target_flight_envelope);
  if (!predicted_vertical.valid) {
    resetPredictionState();
    result.valid = false;
    return result;
  }
  result.predicted_position.z = predicted_vertical.z_m;
  result.vertical_prediction_limited =
      result.vertical_prediction_limited || predicted_vertical.envelope_limited;
  if (!finite(result.predicted_position)) {
    resetPredictionState();
    result.mode = InterceptGuidanceMode::kDirect;
    result.predicted_position = target.position;
    result.predicted_position.z =
        clampToFlightEnvelope(target.position.z, config_.target_flight_envelope)
            .value_or(config_.target_flight_envelope.minimum_target_z_m);
    result.prediction_horizon_s = 0.0;
  }
  return result;
}

const char* interceptGuidanceModeName(const InterceptGuidanceMode mode) noexcept {
  switch (mode) {
    case InterceptGuidanceMode::kDirect:
      return "direct";
    case InterceptGuidanceMode::kAnalyticIntercept:
      return "analytic_intercept";
    case InterceptGuidanceMode::kAheadIntercept:
      return "ahead_intercept";
  }
  return "unknown";
}

} // namespace drone_city_nav

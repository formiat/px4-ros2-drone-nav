#include "drone_city_nav/radar_target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool validOwnship(const TimedVehicleState& state) noexcept {
  return state.position_valid && state.velocity_valid && state.heading_valid &&
         state.stamp_ns > 0 && finite(state.position) && finite(state.velocity) &&
         std::isfinite(state.heading_rad);
}

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] double normalizedAngle(const double angle_rad) noexcept {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] std::int64_t absoluteDifference(const std::int64_t first,
                                              const std::int64_t second) noexcept {
  return first >= second ? first - second : second - first;
}

} // namespace

RadarOwnshipHistory::RadarOwnshipHistory(const RadarOwnshipHistoryConfig& config)
    : config_{config} {
  if (!(config_.retention_ns > 0) || config_.maximum_extrapolation_ns < 0 ||
      config_.maximum_extrapolation_ns > config_.retention_ns) {
    throw std::invalid_argument{"invalid radar ownship history configuration"};
  }
}

bool RadarOwnshipHistory::add(const TimedVehicleState& state) {
  if (!validOwnship(state) ||
      (!samples_.empty() && state.stamp_ns < samples_.back().stamp_ns)) {
    return false;
  }
  if (!samples_.empty() && state.stamp_ns == samples_.back().stamp_ns) {
    samples_.back() = state;
  } else {
    samples_.push_back(state);
  }
  prune(state.stamp_ns);
  return true;
}

void RadarOwnshipHistory::prune(const std::int64_t newest_stamp_ns) {
  while (samples_.size() > 1U &&
         newest_stamp_ns - samples_.front().stamp_ns > config_.retention_ns) {
    samples_.pop_front();
  }
}

std::optional<TimedVehicleState>
RadarOwnshipHistory::sample(const std::int64_t stamp_ns) const noexcept {
  if (stamp_ns <= 0 || samples_.empty()) {
    return std::nullopt;
  }
  const auto upper =
      std::lower_bound(samples_.begin(), samples_.end(), stamp_ns,
                       [](const TimedVehicleState& state, const std::int64_t stamp) {
                         return state.stamp_ns < stamp;
                       });
  if (upper != samples_.end() && upper->stamp_ns == stamp_ns) {
    return *upper;
  }
  if (upper == samples_.begin() || upper == samples_.end()) {
    const TimedVehicleState& nearest =
        upper == samples_.begin() ? samples_.front() : samples_.back();
    const std::int64_t error_ns = absoluteDifference(stamp_ns, nearest.stamp_ns);
    if (error_ns > config_.maximum_extrapolation_ns) {
      return std::nullopt;
    }
    const double delta_s = static_cast<double>(stamp_ns - nearest.stamp_ns) * 1.0e-9;
    TimedVehicleState result = nearest;
    result.position.x += result.velocity.x * delta_s;
    result.position.y += result.velocity.y * delta_s;
    result.position.z += result.velocity.z * delta_s;
    result.stamp_ns = stamp_ns;
    return result;
  }

  const TimedVehicleState& to = *upper;
  const TimedVehicleState& from = *(upper - 1);
  const double ratio = static_cast<double>(stamp_ns - from.stamp_ns) /
                       static_cast<double>(to.stamp_ns - from.stamp_ns);
  const double heading_delta = normalizedAngle(to.heading_rad - from.heading_rad);
  TimedVehicleState result = from;
  result.position = Point3{from.position.x + ratio * (to.position.x - from.position.x),
                           from.position.y + ratio * (to.position.y - from.position.y),
                           from.position.z + ratio * (to.position.z - from.position.z)};
  result.velocity = Vec3{from.velocity.x + ratio * (to.velocity.x - from.velocity.x),
                         from.velocity.y + ratio * (to.velocity.y - from.velocity.y),
                         from.velocity.z + ratio * (to.velocity.z - from.velocity.z)};
  result.heading_rad = normalizedAngle(from.heading_rad + ratio * heading_delta);
  result.stamp_ns = stamp_ns;
  return result;
}

RadarTargetTracker::RadarTargetTracker(const RadarTargetTrackerConfig& config)
    : config_{config} {
  if (!(config_.maximum_update_interval_s > 0.0) ||
      !(config_.position_correction_gain > 0.0) ||
      config_.position_correction_gain > 1.0 ||
      !(config_.velocity_correction_gain > 0.0) ||
      config_.velocity_correction_gain > 1.0 ||
      !(config_.high_rate_velocity_correction_gain > 0.0) ||
      config_.high_rate_velocity_correction_gain > 1.0 ||
      !(config_.maximum_ownship_stamp_error_s >= 0.0) || config_.track_id == 0U) {
    throw std::invalid_argument{"invalid radar target tracker configuration"};
  }
}

void RadarTargetTracker::reset() noexcept {
  previous_.reset();
}

RadarTrackEstimate RadarTargetTracker::update(
    const TimedVehicleState& ownship, const RadarDetectionSample& detection,
    const std::int64_t measurement_stamp_ns, const std::uint64_t scan_sequence,
    const RadarTrackerUpdateMode update_mode) {
  const double velocity_correction_gain =
      update_mode == RadarTrackerUpdateMode::kTrack
          ? config_.high_rate_velocity_correction_gain
          : config_.velocity_correction_gain;
  RadarTrackEstimate result{
      .stamp_ns = measurement_stamp_ns,
      .track_id = config_.track_id,
      .source_scan_sequence = scan_sequence,
      .source_detection_id = detection.detection_id,
      .velocity_correction_gain = velocity_correction_gain,
  };
  if (!validOwnship(ownship) || measurement_stamp_ns <= 0 ||
      static_cast<double>(absoluteDifference(ownship.stamp_ns, measurement_stamp_ns)) *
              1.0e-9 >
          config_.maximum_ownship_stamp_error_s) {
    return result;
  }
  const std::optional<Point3> measured_position =
      radarDetectionPositionMap(ownship, detection);
  const std::optional<Vec3> line_of_sight =
      radarDetectionLineOfSightMap(ownship, detection);
  if (!measured_position.has_value() || !line_of_sight.has_value()) {
    return result;
  }
  result.position = *measured_position;
  result.position_valid = true;
  result.measurement_count =
      previous_.has_value() ? previous_->measurement_count + 1U : 1U;

  if (!previous_.has_value() || !previous_->position_valid ||
      measurement_stamp_ns <= previous_->stamp_ns) {
    previous_ = result;
    return result;
  }
  const double delta_s =
      static_cast<double>(measurement_stamp_ns - previous_->stamp_ns) * 1.0e-9;
  if (delta_s > config_.maximum_update_interval_s) {
    result.measurement_count = 1U;
    previous_ = result;
    return result;
  }

  Vec3 velocity{};
  if (previous_->velocity_valid) {
    const Point3 predicted{previous_->position.x + previous_->velocity.x * delta_s,
                           previous_->position.y + previous_->velocity.y * delta_s,
                           previous_->position.z + previous_->velocity.z * delta_s};
    const Vec3 innovation{measured_position->x - predicted.x,
                          measured_position->y - predicted.y,
                          measured_position->z - predicted.z};
    result.position = Point3{
        predicted.x + config_.position_correction_gain * innovation.x,
        predicted.y + config_.position_correction_gain * innovation.y,
        predicted.z + config_.position_correction_gain * innovation.z,
    };
    velocity = Vec3{
        previous_->velocity.x + velocity_correction_gain * innovation.x / delta_s,
        previous_->velocity.y + velocity_correction_gain * innovation.y / delta_s,
        previous_->velocity.z + velocity_correction_gain * innovation.z / delta_s,
    };
  } else {
    velocity = Vec3{(result.position.x - previous_->position.x) / delta_s,
                    (result.position.y - previous_->position.y) / delta_s,
                    (result.position.z - previous_->position.z) / delta_s};
  }
  const double absolute_target_radial_velocity =
      detection.radial_velocity_mps + dot(ownship.velocity, *line_of_sight);
  const double radial_error =
      absolute_target_radial_velocity - dot(velocity, *line_of_sight);
  velocity.x += line_of_sight->x * radial_error;
  velocity.y += line_of_sight->y * radial_error;
  velocity.z += line_of_sight->z * radial_error;
  if (!finite(velocity)) {
    previous_ = result;
    return result;
  }
  result.velocity = velocity;
  result.velocity_valid = true;
  previous_ = result;
  return result;
}

} // namespace drone_city_nav

#include "drone_city_nav/dynamic_agent_lidar_state.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] Point3 extrapolate(const Point3& position, const Vec3& velocity,
                                 const double delta_s) noexcept {
  return Point3{position.x + velocity.x * delta_s, position.y + velocity.y * delta_s,
                position.z + velocity.z * delta_s};
}

} // namespace

DynamicAgentLidarState::DynamicAgentLidarState(DynamicAgentLidarStateConfig config)
    : config_{std::move(config)} {
  if (!(config_.tracked_agent_radius_m > 0.0) ||
      !(config_.tracked_agent_vertical_tolerance_m >= 0.0) ||
      !(config_.tracked_agent_maximum_age_s > 0.0) ||
      !(config_.cooperative_peer_horizontal_margin_m >= 0.0) ||
      !(config_.cooperative_peer_vertical_margin_m >= 0.0) ||
      !(config_.cooperative_alignment_extrapolation_s >= 0.0) ||
      (config_.cooperative_enabled && config_.own_vehicle_id.empty())) {
    throw std::invalid_argument{"invalid dynamic-agent lidar configuration"};
  }
  if (config_.cooperative_enabled) {
    peer_store_ = std::make_unique<CooperativePeerStore>(config_.own_vehicle_id,
                                                         config_.peer_store);
  }
}

void DynamicAgentLidarState::updateTrackedAgent(const Point3& position,
                                                const Vec3& velocity,
                                                const bool position_valid,
                                                const bool velocity_valid,
                                                const std::int64_t stamp_ns) noexcept {
  tracked_agent_ = TrackedAgentState{
      .position = position,
      .velocity = velocity,
      .stamp_ns = stamp_ns,
      .position_valid = position_valid,
      .velocity_valid = velocity_valid,
  };
}

CooperativePeerUpdateStatus DynamicAgentLidarState::updateCooperativeIntent(
    const CooperativeFlightIntentData& intent, const std::int64_t now_ns) {
  if (!peer_store_) {
    return CooperativePeerUpdateStatus::kInvalid;
  }
  return peer_store_->update(intent, now_ns);
}

DynamicAgentLidarFilterPlan
DynamicAgentLidarState::makeFilterPlan(const std::int64_t now_ns,
                                       const std::int64_t acquisition_stamp_ns) {
  DynamicAgentLidarFilterPlan result;
  const auto maximum_track_age_ns = static_cast<std::int64_t>(
      std::llround(config_.tracked_agent_maximum_age_s * kNanosecondsPerSecond));
  if (tracked_agent_.position_valid && tracked_agent_.stamp_ns > 0 && now_ns > 0 &&
      now_ns >= tracked_agent_.stamp_ns &&
      now_ns - tracked_agent_.stamp_ns <= maximum_track_age_ns &&
      acquisition_stamp_ns > 0) {
    const double delta_s =
        static_cast<double>(acquisition_stamp_ns - tracked_agent_.stamp_ns) /
        kNanosecondsPerSecond;
    const Point3 position =
        tracked_agent_.velocity_valid
            ? extrapolate(tracked_agent_.position, tracked_agent_.velocity, delta_s)
            : tracked_agent_.position;
    result.tracked_agent_exclusions.push_back(DynamicAgentLidarVolume{
        .position = position,
        .radius_m = config_.tracked_agent_radius_m,
        .lower_extent_m = config_.tracked_agent_vertical_tolerance_m,
        .upper_extent_m = config_.tracked_agent_vertical_tolerance_m,
    });
  }

  if (!peer_store_ || acquisition_stamp_ns <= 0) {
    return result;
  }
  const double maximum_extrapolation_s = config_.cooperative_alignment_extrapolation_s;
  for (const CooperativeFlightIntentData& intent : peer_store_->activeIntents(now_ns)) {
    Point3 position{};
    if (const std::optional<CooperativeTrajectorySample> sample =
            sampleCooperativeTrajectory(intent, acquisition_stamp_ns);
        sample.has_value()) {
      position = sample->position;
    } else {
      const double delta_s =
          static_cast<double>(acquisition_stamp_ns - intent.stamp_ns) /
          kNanosecondsPerSecond;
      if (std::abs(delta_s) > maximum_extrapolation_s) {
        continue;
      }
      position = extrapolate(intent.current_position, intent.current_velocity, delta_s);
    }
    result.cooperative_memory_exclusions.push_back(DynamicAgentLidarVolume{
        .position = position,
        .radius_m =
            intent.footprint_radius_m + config_.cooperative_peer_horizontal_margin_m,
        .lower_extent_m = intent.footprint_lower_extent_m +
                          config_.cooperative_peer_vertical_margin_m,
        .upper_extent_m = intent.footprint_upper_extent_m +
                          config_.cooperative_peer_vertical_margin_m,
    });
  }
  return result;
}

} // namespace drone_city_nav

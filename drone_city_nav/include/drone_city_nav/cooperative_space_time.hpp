#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace drone_city_nav {

struct CooperativeSpaceTimeConfig {
  double prediction_horizon_s{5.0};
  double desired_minimum_separation_m{5.0};
  double spatial_transition_s{1.5};
  double minimum_spatial_offset_m{0.5};
  double maximum_spatial_offset_m{5.0};
  double minimum_time_shift_s{0.25};
  double maximum_time_shift_s{2.0};
  double sample_period_s{0.1};
  double spatial_margin_m{0.25};
  double incumbent_hysteresis_m{0.25};
};

struct CooperativeSpaceTimeDecision {
  bool valid{false};
  bool active{false};
  bool changed{false};
  CooperativeManeuver maneuver{CooperativeManeuver::kKeep};
  Vec3 preferred_acceleration_direction{};
  double lateral_offset_m{0.0};
  double vertical_offset_m{0.0};
  double time_shift_s{0.0};
  double predicted_minimum_separation_m{0.0};
  double time_to_minimum_s{0.0};
  double integrated_separation_shortfall_m2_s{0.0};
  std::size_t evaluated_candidate_count{0U};
};

[[nodiscard]] bool
cooperativeSpaceTimeConfigIsValid(const CooperativeSpaceTimeConfig& config) noexcept;

[[nodiscard]] CooperativeSpaceTimeDecision optimizeCooperativeSpaceTime(
    const CooperativeFlightIntentData& ownship,
    std::span<const CooperativeConflictPeer> conflicting_peers, std::int64_t now_ns,
    const CooperativeSpaceTimeConfig& config = {},
    std::optional<CooperativeManeuver> incumbent = std::nullopt);

} // namespace drone_city_nav

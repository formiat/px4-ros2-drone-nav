#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace drone_city_nav {

struct CooperativeChannelCoordinationConfig {
  double reservation_time_margin_s{0.5};
  double same_path_entry_headway_s{1.0};
  double lateral_separation_tolerance_m{0.1};
  CooperativeConflictConfig conflict{};
};

struct CooperativeChannelDecision {
  bool active{false};
  double lateral_offset_m{0.0};
  std::int64_t entry_not_before_ns{0};
  bool yield_before_entry{false};
  bool conflict_zone_only{false};
  std::string yield_to_vehicle_id;
};

[[nodiscard]] double
channelLateralSeparationM(const CooperativeChannelUse& first,
                          const CooperativeChannelUse& second) noexcept;

[[nodiscard]] CooperativeChannelDecision coordinateCooperativeChannel(
    const CooperativeFlightIntentData& ownship,
    std::span<const CooperativeFlightIntentData> peers,
    const CooperativeChannelCoordinationConfig& config = {}) noexcept;

} // namespace drone_city_nav

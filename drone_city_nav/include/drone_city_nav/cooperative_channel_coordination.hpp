#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"

#include <span>
#include <string>

namespace drone_city_nav {

struct CooperativeChannelCoordinationConfig {
  double reservation_time_margin_s{0.5};
  double same_lane_entry_headway_s{1.0};
  CooperativeConflictConfig conflict{};
};

struct CooperativeChannelDecision {
  bool active{false};
  std::size_t lane_index{0U};
  std::size_t lane_count{0U};
  bool yield_before_entry{false};
  bool conflict_zone_only{false};
  std::string yield_to_vehicle_id;
};

[[nodiscard]] std::size_t assignCooperativeChannelLane(int direction_sign,
                                                       std::size_t lane_count) noexcept;

[[nodiscard]] CooperativeChannelDecision coordinateCooperativeChannel(
    const CooperativeFlightIntentData& ownship,
    std::span<const CooperativeFlightIntentData> peers,
    const CooperativeChannelCoordinationConfig& config = {}) noexcept;

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/types.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <span>

namespace drone_city_nav {

struct MppiDebugMarkerInput {
  std_msgs::msg::Header header;
  std::span<const mppi::State> horizon;
  std::span<const mppi::State> previous_horizon;
  std::span<const mppi::State> execution_horizon;
  std::span<const mppi::RouteSample3D> global_route;
  mppi::State initial_state{};
  mppi::State target{};
  Point3 mission_start{};
  Point3 mission_goal{};
  mppi::RiskTier selected_tier{mppi::RiskTier::kCollision};
};

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildMppiDebugMarkers(const MppiDebugMarkerInput& input);

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <span>
#include <string>

namespace drone_city_nav {

struct MppiDebugMarkerInput {
  std_msgs::msg::Header header;
  std::span<const mppi::State> horizon;
  std::span<const mppi::State> previous_horizon;
  std::span<const mppi::State> execution_horizon;
  std::span<const mppi::RouteSample3D> global_route;
  std::span<const PassageTraversalEdge> passage_traversals;
  std::span<const PassageTraversalId> selected_passage_traversal_ids;
  mppi::State initial_state{};
  mppi::State target{};
  Point3 mission_start{};
  Point3 mission_goal{};
  Point3 observed_tracking_target{};
  Point3 predicted_tracking_target{};
  Point3 resolved_tracking_target{};
  bool tracking_objective_active{false};
  mppi::RiskTier selected_tier{mppi::RiskTier::kCollision};
};

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildMppiDebugMarkers(const MppiDebugMarkerInput& input);

} // namespace drone_city_nav

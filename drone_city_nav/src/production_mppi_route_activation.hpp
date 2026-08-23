#pragma once

#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include "production_mppi_node.hpp"

namespace drone_city_nav {

struct ProductionRouteActivationResult3D {
  ProductionMppiPreparedEsdf prepared{};
  StaticRouteCandidateValidation validation{};
  RouteActivationAssessment3D assessment{};
  RouteProposalReplacementAssessment3D replacement{};
  mppi::StaticRouteHandoffResult handoff{};
  StaticRouteActivationStatus activation_status{
      StaticRouteActivationStatus::kNotAttempted};
  std::uint64_t snapshot_pose_revision{0U};
  std::uint64_t snapshot_raw_revision{0U};
  std::uint64_t required_objective_sample{0U};
  bool world_compatible{false};
  bool generation_matches{false};
  bool objective_matches{false};
  bool snapshot_current{false};
  bool observed_world_rebased{false};
  bool publication_world_advanced{false};
  bool activated{false};
};

} // namespace drone_city_nav

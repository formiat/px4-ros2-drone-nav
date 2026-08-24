#pragma once

#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include "production_mppi_node.hpp"

namespace drone_city_nav {

struct ProductionRouteActivationSnapshot3D {
  std::optional<ProductionMppiPreparedEsdf> resident_world;
  std::shared_ptr<const ExecutionRouteSnapshot3D> execution_snapshot;
  ProductionMppiNavigation navigation{};
  ProductionMppiAppliedControl applied_control{};
  ProductionMppiExecutionHorizonOwner execution_horizon_owner{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::uint64_t minimum_tracking_route_mission_epoch{0U};
  std::uint64_t minimum_tracking_route_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

struct ProductionRouteActivationResult3D {
  ProductionMppiPreparedEsdf prepared{};
  ProductionMaterializedRouteProposal3D proposal{};
  StaticRouteCandidateValidation validation{};
  RouteActivationAssessment3D assessment{};
  RouteProposalReplacementAssessment3D replacement{};
  mppi::StaticRouteHandoffResult handoff{};
  RouteSpliceCertificationResult3D splice{};
  StaticRouteActivationStatus activation_status{
      StaticRouteActivationStatus::kNotAttempted};
  std::uint64_t candidate_generation{0U};
  std::uint64_t snapshot_pose_revision{0U};
  std::uint64_t snapshot_raw_revision{0U};
  std::uint64_t required_objective_sample{0U};
  bool world_compatible{false};
  bool generation_matches{false};
  bool objective_matches{false};
  bool snapshot_current{false};
  bool observed_world_rebased{false};
  bool publication_world_advanced{false};
  bool certified_pending{false};

  [[nodiscard]] bool executionGeometryValid() const noexcept;
  [[nodiscard]] bool readyForArbitration() const noexcept;
};

} // namespace drone_city_nav

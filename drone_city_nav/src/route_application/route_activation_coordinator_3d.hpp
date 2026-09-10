#pragma once

#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/dynamic_handoff_validator_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/route_risk_policy_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstdint>
#include <memory>
#include <optional>

#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"
#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_materializer_3d.hpp"
#include "route_trajectory_compiler_3d.hpp"

namespace drone_city_nav {

struct ProductionRouteActivationSnapshot3D {
  std::shared_ptr<const WorldSnapshot3D> resident_world;
  std::shared_ptr<const CommittedExecutionAuthority3D> execution_authority;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_route;
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::uint64_t minimum_tracking_route_mission_epoch{0U};
  std::uint64_t minimum_tracking_route_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

struct PendingRoutePublicationCurrentness3D {
  bool resident_world_current{false};
  bool objective_current{false};
  bool raw_world_current{false};
  bool execution_base_current{false};
  bool pending_current{false};
  bool candidate_world_coherent{false};
};

// Pending publication only reserves an execution transaction. The planning
// tick recertifies it against the latest raw world before it can become an
// execution owner, so raw snapshot churn is diagnostic rather than a
// transaction-base invalidation here.
[[nodiscard]] bool pendingRoutePublicationBaseCurrent3D(
    const PendingRoutePublicationCurrentness3D& currentness) noexcept;

struct RouteActivationCoordinatorConfig3D {
  RouteTrajectoryCompilerConfig3D trajectory_compiler{};
  StaticRouteExtensionConfig route_extension{};
  RouteSuccessorImprovementConfig3D successor_improvement{};
  FlightEnvelopeConfig flight_envelope{};
  RouteTrackingPolicy3D route_tracking{};
  SweptFootprintConfig physical_footprint{};
  CertifiedRouteSpliceConfig3D certified_splice{};
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  RouteRiskPolicy3D route_risk{};
  DynamicHandoffValidator3D dynamic_handoff_validator;
  double cruise_speed_mps{0.0};
  double maximum_control_feedback_age_ms{0.0};
};

// One immutable preparation transaction. Every identity and resource used by
// compilation and certification remains owned through the resulting pending
// draft; no callback or provider is consulted by the coordinator.
struct RouteActivationPreparationRequest3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionRouteMaterialization3D materialization{};
  ProductionRouteActivationSnapshot3D snapshot{};
  StaticRoutePlanningLatencyStats planning_latency{};
  // Whether the search that produced the candidate has settled on its world:
  // it converged, or it has been given the grace it gets. A blocked route's
  // replacement that costs far more than the route it replaces is activated
  // only once the search has settled; before that the candidate is the first
  // route the search found, not the best it can find.
  bool search_settled{true};

  [[nodiscard]] bool valid() const noexcept;
};

struct PreparedRouteActivation3D {
  ProductionRouteActivationSnapshot3D snapshot{};
  std::shared_ptr<const ExecutionPlan3D> execution_base;
  std::optional<PendingCertifiedRoute3D> pending_draft;
  ProductionRouteActivationResult3D result{};
};

struct RouteActivationCommitResult3D {
  ProductionRouteActivationResult3D result{};
};

// The caller captures these values while holding its world/objective commit
// boundary and keeps that boundary locked for the complete commit call.
struct RouteActivationCommitContext3D {
  std::shared_ptr<const WorldSnapshot3D> resident_world;
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::uint64_t minimum_tracking_route_mission_epoch{0U};
  std::uint64_t minimum_tracking_route_sample_sequence{0U};
};

class RouteActivationCoordinator3D final {
public:
  explicit RouteActivationCoordinator3D(
      const RouteActivationCoordinatorConfig3D& config);

  RouteActivationCoordinator3D(const RouteActivationCoordinator3D&) = delete;
  RouteActivationCoordinator3D& operator=(const RouteActivationCoordinator3D&) = delete;
  RouteActivationCoordinator3D(RouteActivationCoordinator3D&&) = delete;
  RouteActivationCoordinator3D& operator=(RouteActivationCoordinator3D&&) = delete;

  [[nodiscard]] PreparedRouteActivation3D
  prepare(RouteActivationPreparationRequest3D request) const;

  [[nodiscard]] RouteActivationCommitResult3D
  commit(PreparedRouteActivation3D prepared,
         const RouteActivationCommitContext3D& context,
         ExecutionSupervisor3D& execution_supervisor) const;

private:
  RouteActivationCoordinatorConfig3D config_{};
  RouteTrajectoryCompiler3D trajectory_compiler_;
};

} // namespace drone_city_nav

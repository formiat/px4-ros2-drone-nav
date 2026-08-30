#pragma once

#include "drone_city_nav/mppi/static_route_handoff.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"

#include "production_mppi_node.hpp"

namespace drone_city_nav {

[[nodiscard]] ProductionCompiledRouteCandidate3D
makeCompiledRouteCandidate3D(MaterializedRoute3D materialized,
                             RouteCompilationResult3D compilation);

struct ProductionRouteActivationSnapshot3D {
  std::shared_ptr<const WorldSnapshot3D> resident_world;
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

struct PendingRoutePublicationCurrentness3D {
  bool resident_world_current{false};
  bool objective_current{false};
  bool raw_world_current{false};
  bool execution_base_current{false};
  bool candidate_world_coherent{false};
};

// Pending publication only reserves an execution transaction. The planning
// tick recertifies it against the latest raw world before it can become an
// execution owner, so raw snapshot churn is diagnostic rather than a
// transaction-base invalidation here.
[[nodiscard]] bool pendingRoutePublicationBaseCurrent3D(
    const PendingRoutePublicationCurrentness3D& currentness) noexcept;

} // namespace drone_city_nav

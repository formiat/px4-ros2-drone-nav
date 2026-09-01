#include <memory>

#include "production_mppi_node.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

ProductionRouteActivationSnapshot3D
ProductionMppiNode::captureRouteActivationSnapshot3D() {
  ProductionRouteActivationSnapshot3D snapshot;
  {
    const auto lock = evidence_boundary_.evidenceWithInput();
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    const RouteExecutionManagerSnapshot3D execution = execution_supervisor_.snapshot();
    snapshot.execution_authority = execution.authority;
    snapshot.pending_route = execution.pending;
    snapshot.raw_world = world_pipeline_->latestRawWorld();
    snapshot.resident_world = resident.world();
    snapshot.navigation = navigation_;
    const std::shared_ptr<const ProductionNavigationObjectiveState> objective_state =
        navigationObjectiveState();
    if (objective_state != nullptr) {
      snapshot.objective = objective_state->objective;
      snapshot.minimum_tracking_route_mission_epoch =
          objective_state->minimum_tracking_route_mission_epoch;
      snapshot.minimum_tracking_route_sample_sequence =
          objective_state->minimum_tracking_route_sample_sequence;
    }
  }
  snapshot.stamp_ns = get_clock()->now().nanoseconds();
  return snapshot;
}

} // namespace drone_city_nav

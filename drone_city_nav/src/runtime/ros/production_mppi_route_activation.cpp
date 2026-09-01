#include <memory>

#include "production_mppi_node.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

ProductionRouteActivationSnapshot3D
ProductionMppiNode::captureRouteActivationSnapshot3D() {
  ProductionRouteActivationSnapshot3D snapshot;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    const RouteExecutionManagerSnapshot3D execution = execution_supervisor_.snapshot();
    snapshot.execution_authority = execution.authority;
    snapshot.pending_route = execution.pending;
    snapshot.raw_world = world_pipeline_->latestRawWorld();
    snapshot.resident_world = resident.world();
    snapshot.navigation = navigation_;
    snapshot.objective = navigationObjective();
  }
  snapshot.minimum_tracking_route_mission_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  snapshot.minimum_tracking_route_sample_sequence =
      minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire);
  snapshot.stamp_ns = get_clock()->now().nanoseconds();
  return snapshot;
}

} // namespace drone_city_nav

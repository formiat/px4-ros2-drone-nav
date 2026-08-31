#include <cinttypes>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"
#include "route_activation_coordinator_3d.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

ProductionRouteActivationSnapshot3D
ProductionMppiNode::captureRouteActivationSnapshot3D() {
  ProductionRouteActivationSnapshot3D snapshot;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    snapshot.execution_authority = route_execution_manager_.authority();
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

ProductionRouteActivationResult3D
ProductionMppiNode::commitRouteActivation3D(PreparedRouteActivation3D prepared) {
  const MaterializedRoute3D& candidate = prepared.result.materialized;
  const RouteAdmissionReport3D& report = prepared.result.admission;
  if (candidate.candidate_generation != 0U &&
      (!report.trajectory_validation.valid() ||
       prepared.result.proposal.trajectory == nullptr)) {
    RCLCPP_WARN(
        get_logger(),
        "COMPILED_TRAJECTORY valid=false reason=%s sample_index=%zu "
        "route_generation=%" PRIu64,
        compiledTrajectoryFailureReason3DName(report.trajectory_validation.reason),
        report.trajectory_validation.sample_index, candidate.candidate_generation);
  }
  RouteActivationCommitResult3D committed;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_};
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    committed = route_activation_coordinator_->commit(
        std::move(prepared),
        RouteActivationCommitContext3D{
            .resident_world = resident.world(),
            .objective = navigationObjective(),
            .raw_world = world_pipeline_->latestRawWorld(),
            .minimum_tracking_route_mission_epoch =
                minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire),
            .minimum_tracking_route_sample_sequence =
                minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire),
        },
        route_execution_manager_);
  }
  latest_route_pipeline_event_.store(
      std::make_shared<const ProductionRouteActivationResult3D>(committed.result),
      std::memory_order_release);
  return std::move(committed.result);
}

} // namespace drone_city_nav

#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_stop_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cinttypes>
#include <cmath>
#include <memory>
#include <string_view>

#include "production_mppi_node_execution_internal.hpp"
#include "raw_world_ingress_ros_3d.hpp"

namespace drone_city_nav {

ProductionMppiExecutionPublication
ProductionMppiNode::publishStopExecution(const ProductionMppiExecutionCycle& cycle,
                                         const ProductionMppiExecutionReason reason) {
  ProductionMppiExecutionPublication publication;
  if (cycle.evidence.execution_dynamics == nullptr ||
      cycle.controller.result == nullptr ||
      cycle.controller.resultRef().controls.empty()) {
    return publication;
  }
  const auto evidence_lock = evidence_boundary_.evidenceWithLatestLidar();
  // A route-directed cycle derives no validation world of its own: the route
  // owns one, and a stop is exactly the answer to a route whose world can no
  // longer be trusted. Derive the newest observed evidence here, the way a
  // hold derives it.
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world =
      cycle.evidence.direct_observed_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world =
      cycle.evidence.direct_static_world;
  const WorldSnapshot3D& world = cycle.controller.worldRef();
  if (observed_raw_world == nullptr && static_world == nullptr) {
    if (config_.world.use_static_map && world.static_occupancy != nullptr) {
      static_world = VersionedStaticWorld3D::captureOwned(
          navigationWorldCertificate3D(world), world.static_occupancy);
    } else if (const RawWorldIngressSnapshot3D world_input =
                   raw_world_ingress_->snapshot();
               world_input.latest_raw_world != nullptr) {
      const Point3 body_position{cycle.evidence.exact_initial_state.x,
                                 cycle.evidence.exact_initial_state.y,
                                 cycle.evidence.exact_initial_state.z};
      observed_raw_world = world_input.latest_raw_world->deriveRouteEvidence(
          proprioceptiveContactSeed3D(
              body_position, cycle.evidence.exact_previous_control,
              config_.world.physical_footprint,
              std::addressof(world_input.latest_raw_world->occupancy()))
              .value_or(proprioceptiveContactSeed3D(
                  body_position, cycle.evidence.exact_previous_control,
                  config_.world.physical_footprint, 0.0)),
          world.launch_support_contact);
    }
  }
  const ExecutionStopPreparation3D prepared =
      execution_supervisor_.prepareStop(ExecutionStopRequest3D{
          .cycle_source_plan = cycle.route.execution.source_snapshot,
          .execution_input = cycle.evidence.execution_input,
          .latest_lidar_evidence = cycle.evidence.latest_lidar_evidence,
          .observed_raw_world = observed_raw_world,
          .static_world = static_world,
          .validation_policy = cycle.evidence.selected_policy != nullptr
                                   ? cycle.evidence.selected_policy
                                   : config_.execution.validation_policy,
          .exact_initial_state = cycle.evidence.exact_initial_state,
          .exact_previous_control = cycle.evidence.exact_previous_control,
          .finite_horizon_config = config_.execution.finite_horizon,
          .minimum_control_count = cycle.controller.resultRef().controls.size(),
          .now_ns = cycle.controller.now_ns,
      });
  const StopExecution3D* const stop = prepared.stopExecution();
  if (!prepared.prepared() || stop == nullptr || stop->horizon == nullptr) {
    // A resident stop that still holds the vehicle is the answer, not a
    // failure: it is already the trajectory this call would have produced.
    if (prepared.status != ExecutionStopStatus3D::kResidentStopCurrent &&
        prepared.status != ExecutionStopStatus3D::kAtRest) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STOP_EXECUTION published=false status=%s transition=%.*s "
          "detail=%.*s certification=%.*s certification_dynamics=%s "
          "certification_path=%s speed_mps=%.2f replacement_failure=%s",
          executionStopStatus3DName(prepared.status),
          static_cast<int>(
              executionRouteTransitionStatus3DName(prepared.transition_status).size()),
          executionRouteTransitionStatus3DName(prepared.transition_status).data(),
          static_cast<int>(
              executionRouteTransitionDetail3DName(prepared.transition_detail).size()),
          executionRouteTransitionDetail3DName(prepared.transition_detail).data(),
          static_cast<int>(
              stopCertificationStatus3DName(prepared.certification.status).size()),
          stopCertificationStatus3DName(prepared.certification.status).data(),
          motionDynamicsConsistency3DName(prepared.certification.dynamics_consistency),
          finiteExecutionPathStatus3DName(
              prepared.certification.path_validation_status),
          prepared.initial_speed_mps, productionMppiExecutionReasonName(reason));
    }
    return publication;
  }

  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, stop->valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  // A stop follows no route, so it constrains none of the route metadata a
  // route-directed horizon carries.
  horizon.route_constrained = false;
  if (stop->observed_raw_world != nullptr) {
    horizon.obstacle_revision = stop->observed_raw_world->version().revision;
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, stop->horizon->states, stop->horizon->controls,
          cycle.evidence.exact_previous_control, stop->control_interval_ns,
          config_.control.mppi.dynamics)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STOP_EXECUTION published=false status=horizon_encoding_"
                         "rejected speed_mps=%.2f",
                         prepared.initial_speed_mps);
    return publication;
  }
  const std::shared_ptr<const ExecutionPlan3D> expected = prepared.expectedPlan();
  // The evidence boundary is already held for the whole preparation, so the
  // stop commits through the same boundary the way a hold does instead of
  // re-entering it through the snapshot commit.
  if (expected == nullptr || commitAndPublishExecutionHorizon(
                                 cycle, horizon,
                                 ExecutionHorizonLeaseCandidate3D{
                                     .kind = ExecutionHorizonCommitKind3D::kTransition,
                                     .expected_authority = prepared.expected_authority,
                                     .expected_plan = expected,
                                     .certification_plan = expected,
                                     .progress_preparation = nullptr,
                                     .transition = prepared.transition,
                                     .expected_pending = nullptr,
                                     .owner = {},
                                     .expected_horizon_producer_instance_id = 0U,
                                     .execution_input = cycle.evidence.execution_input,
                                     .stationary_capture_rearm_intent = false,
                                 }) != ProductionMppiHorizonCommitStatus::kPublished) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STOP_EXECUTION published=false status=publication_commit_"
                         "rejected speed_mps=%.2f",
                         prepared.initial_speed_mps);
    return publication;
  }

  publication.horizon = stop->horizon->states;
  publication.mode = ProductionMppiExecutionMode::kPlanned;
  publication.reason = reason;
  publication.planned_control_count = stop->horizon->controls.size();
  publication.nominal_prefix_control_count =
      stop->horizon->nominal_prefix_control_count;
  publication.arrival_control_count = stop->horizon->arrival_control_count;
  publication.first_control = stop->horizon->controls.front();
  publication.first_control_available = true;
  publication.latest_lidar_obstacle_sequence =
      cycle.evidence.latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count =
      cycle.evidence.latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms =
      cycle.evidence.latest_lidar_obstacle_age_ms;
  publication.latest_lidar_obstacle_fresh = cycle.evidence.latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      cycle.evidence.latest_lidar_obstacle_receive_time_fallback;
  publication.terminal_rest_state = true;
  publication.published = true;
  RCLCPP_WARN(get_logger(),
              "STOP_EXECUTION published=true trajectory_revision=%" PRIu64
              " snapshot_version=%" PRIu64 " speed_mps=%.2f stop_distance_m=%.2f "
              "controls=%zu rest=(%.2f,%.2f,%.2f) replacement_failure=%s",
              stop->trajectory_revision, prepared.preparedPlan()->version,
              prepared.initial_speed_mps, prepared.stop_distance_m,
              stop->horizon->controls.size(), stop->rest_position.x,
              stop->rest_position.y, stop->rest_position.z,
              productionMppiExecutionReasonName(reason));
  return publication;
}

} // namespace drone_city_nav

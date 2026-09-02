#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include <cinttypes>
#include <memory>
#include <optional>
#include <string_view>

#include "production_mppi_node_execution_internal.hpp"

namespace drone_city_nav {

std::optional<ProductionMppiExecutionPublication>
ProductionMppiNode::retainActiveFinitePath(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason replacement_failure_reason,
    bool* const physically_rejected) {
  const ProductionRouteExecutionSelection3D& route_execution = cycle.route.execution;
  const ExecutionRetentionResult3D prepared =
      execution_supervisor_.prepareRetention(ExecutionRetentionRequest3D{
          .lifecycle_source_plan = route_execution.source_snapshot,
          .lifecycle_event = route_execution.lifecycle_event,
          .lifecycle_observed_raw_world = route_execution.lifecycle_observed_raw_world,
          .execution_input = cycle.evidence.execution_input,
          .latest_lidar_evidence = cycle.evidence.latest_lidar_evidence,
          .exact_initial_state = cycle.evidence.exact_initial_state,
          .exact_previous_control = cycle.evidence.exact_previous_control,
          .finite_horizon_config = config_.execution.finite_horizon,
          .now_ns = cycle.controller.now_ns,
          .lidar_validation_now_ns = cycle.evidence.lidar_validation_now_ns,
      });
  const std::shared_ptr<const ExecutionPlan3D> expected = prepared.expectedPlan();
  const std::uint64_t expected_version = expected != nullptr ? expected->version : 0U;
  if (!prepared.prepared()) {
    if (physically_rejected != nullptr) {
      const auto physical = [](const FiniteExecutionPathStatus3D status) {
        return status == FiniteExecutionPathStatus3D::kRawCollision ||
               status == FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
      };
      *physically_rejected = physical(prepared.trajectory_validation.status) ||
                             physical(prepared.rebuild_validation.status) ||
                             physical(prepared.actual_state_validation.status);
    }
    const std::string_view certification_status =
        finiteExecutionCertificationStatus3DName(prepared.certification.status);
    const std::string_view adherence_status = finiteExecutionRouteAdherenceStatus3DName(
        prepared.certification.route_adherence_status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_RETENTION prepared=false kind=%s stage=%s "
        "snapshot_version=%" PRIu64 " replacement_failure_reason=%s "
        "trajectory_validation=%s actual_state_validation=%s "
        "rebuild_validation=%s certification=%.*s route_adherence=%.*s "
        "route_adherence_failure_distance_m=%.3f",
        executionRetentionKind3DName(prepared.kind),
        executionRetentionStatus3DName(prepared.status), expected_version,
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(prepared.trajectory_validation.status),
        mppi::finiteExecutionPathStatusName(prepared.actual_state_validation.status),
        mppi::finiteExecutionPathStatusName(prepared.rebuild_validation.status),
        static_cast<int>(certification_status.size()), certification_status.data(),
        static_cast<int>(adherence_status.size()), adherence_status.data(),
        prepared.certification.route_adherence_failure_distance_m);
    return std::nullopt;
  }

  const ExecutionRouteTransitionResult3D& transition = *prepared.transition;
  const FiniteExecutionState3D* const retained_route_execution =
      transition.next->finiteExecution();
  const DirectTrackingFiniteExecution3D* const retained_direct_execution =
      transition.next->directTrackingExecution();
  const FiniteExecutionState3D* const resident_route_execution =
      expected->finiteExecution();
  const DirectTrackingFiniteExecution3D* const resident_direct_execution =
      expected->directTrackingExecution();
  const mppi::FiniteHorizon* retained_horizon{nullptr};
  std::int64_t retained_valid_until_ns{0};
  std::int64_t retained_control_interval_ns{0};
  if (retained_route_execution != nullptr) {
    retained_horizon = retained_route_execution->horizon.get();
    retained_valid_until_ns = retained_route_execution->valid_until_ns;
    retained_control_interval_ns = retained_route_execution->control_interval_ns;
  } else if (retained_direct_execution != nullptr) {
    retained_horizon = retained_direct_execution->horizon.get();
    retained_valid_until_ns = retained_direct_execution->valid_until_ns;
    retained_control_interval_ns = retained_direct_execution->control_interval_ns;
  }
  const mppi::FiniteHorizon* resident_horizon{nullptr};
  if (resident_route_execution != nullptr) {
    resident_horizon = resident_route_execution->horizon.get();
  } else if (resident_direct_execution != nullptr) {
    resident_horizon = resident_direct_execution->horizon.get();
  }
  const bool prepared_kind_valid =
      (prepared.kind == ExecutionRetentionKind3D::kRoute &&
       retained_route_execution != nullptr && resident_route_execution != nullptr) ||
      (prepared.kind == ExecutionRetentionKind3D::kDirectTracking &&
       retained_direct_execution != nullptr && resident_direct_execution != nullptr);
  if (!prepared_kind_valid || retained_horizon == nullptr ||
      resident_horizon == nullptr || retained_horizon->controls.empty() ||
      resident_horizon->controls.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_RETENTION prepared=false kind=%s "
                         "stage=prepared_horizon_unavailable snapshot_version=%" PRIu64,
                         executionRetentionKind3DName(prepared.kind), expected_version);
    return std::nullopt;
  }

  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, retained_valid_until_ns, ProductionMppiExecutionMode::kPlanned,
      ProductionMppiExecutionReason::kNone);
  if (prepared.kind == ExecutionRetentionKind3D::kRoute) {
    const CertifiedRouteSuffix3D* const retained_route = transition.next->route();
    if (retained_route == nullptr ||
        !production_mppi_execution_detail::bindHorizonRouteMetadata(horizon,
                                                                    *retained_route)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "EXECUTION_RETENTION prepared=false kind=route "
                           "stage=route_metadata_rejected snapshot_version=%" PRIu64,
                           transition.next->version);
      return std::nullopt;
    }
    if (retained_route_execution->observed_raw_world != nullptr) {
      horizon.obstacle_revision =
          retained_route_execution->observed_raw_world->version().revision;
    }
  } else if (prepared.kind == ExecutionRetentionKind3D::kDirectTracking) {
    horizon.route_constrained = false;
    horizon.route_target.x = retained_direct_execution->target.x;
    horizon.route_target.y = retained_direct_execution->target.y;
    horizon.route_target.z = retained_direct_execution->target.z;
  } else {
    return std::nullopt;
  }
  if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
          horizon, retained_horizon->states, retained_horizon->controls,
          cycle.evidence.exact_previous_control, retained_control_interval_ns)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_RETENTION prepared=false kind=%s "
                         "stage=horizon_encoding_rejected snapshot_version=%" PRIu64,
                         executionRetentionKind3DName(prepared.kind),
                         transition.next->version);
    return std::nullopt;
  }

  if (commitExecutionSnapshotHorizon(cycle, expected, transition, horizon, nullptr,
                                     expected, nullptr, prepared.expected_authority) !=
      ProductionMppiHorizonCommitStatus::kPublished) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_RETENTION prepared=false kind=%s "
                         "stage=publication_commit_rejected snapshot_version=%" PRIu64,
                         executionRetentionKind3DName(prepared.kind), expected_version);
    return std::nullopt;
  }
  const mppi::FiniteHorizon& reported_horizon = *retained_horizon;
  ProductionMppiExecutionPublication retained;
  retained.horizon = reported_horizon.states;
  retained.mode = ProductionMppiExecutionMode::kPlanned;
  retained.reason = replacement_failure_reason;
  retained.planned_control_count = reported_horizon.controls.size();
  retained.nominal_prefix_control_count = reported_horizon.nominal_prefix_control_count;
  retained.arrival_control_count = reported_horizon.arrival_control_count;
  retained.arrival_shaping_attempts = prepared.arrival_shaping_attempts;
  retained.first_control = reported_horizon.controls.front();
  retained.first_control_available = true;
  retained.latest_lidar_obstacle_sequence =
      cycle.evidence.latest_lidar_evidence->sequence();
  retained.latest_lidar_obstacle_hit_count =
      cycle.evidence.latest_lidar_obstacle_points.size();
  retained.latest_lidar_obstacle_age_ms = cycle.evidence.latest_lidar_obstacle_age_ms;
  retained.latest_lidar_obstacle_fresh = cycle.evidence.latest_lidar_obstacle_fresh;
  retained.latest_lidar_obstacle_receive_time_fallback =
      cycle.evidence.latest_lidar_obstacle_receive_time_fallback;
  retained.retained_previous_finite_path = true;
  retained.resident_owner_continues = false;
  retained.terminal_rest_state = true;
  retained.published = true;

  const std::string_view braking_event_name =
      prepared.braking_event.has_value()
          ? routeLifecycleEventKind3DName(*prepared.braking_event)
          : std::string_view{"none"};
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "EXECUTION_RETENTION retained=true recertified=true kind=%s published=true "
      "snapshot_version=%" PRIu64 " trajectory_revision=%" PRIu64
      " braking_event=%.*s actual_state_validation=%s "
      "trajectory_validation=%s",
      executionRetentionKind3DName(prepared.kind), transition.next->version,
      prepared.prepared_trajectory_revision,
      static_cast<int>(braking_event_name.size()), braking_event_name.data(),
      mppi::finiteExecutionPathStatusName(prepared.actual_state_validation.status),
      mppi::finiteExecutionPathStatusName(prepared.trajectory_validation.status));
  return retained;
}

} // namespace drone_city_nav

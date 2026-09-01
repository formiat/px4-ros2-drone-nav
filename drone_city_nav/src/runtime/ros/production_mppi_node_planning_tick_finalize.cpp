#include "production_mppi_node_planning_tick_finalize.hpp"

#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "navigation_diagnostics_sink.hpp"
#include "production_mppi_diagnostics_snapshot.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

} // namespace

void ProductionMppiNode::finalizePlanningTick(
    const ProductionMppiPlanningTickFinalization& finalization) {
  const mppi::MppiTickInput& input = finalization.input;
  mppi::MppiTickResult& result = finalization.result;
  const std::shared_ptr<const WorldSnapshot3D>& world = finalization.world;
  const ProductionRouteExecutionSelection3D& route_execution =
      finalization.route_execution;
  const std::shared_ptr<const VersionedExecutionInput3D>& execution_input =
      finalization.execution_input;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence =
      finalization.latest_lidar_evidence;
  const ProductionMppiNavigation& navigation = finalization.navigation;
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>>& execution_mppi_route =
      finalization.execution_mppi_route;
  const std::shared_ptr<const std::vector<PassageTraversalId>>&
      execution_selected_passage_traversal_ids =
          finalization.execution_selected_passage_traversal_ids;
  const std::shared_ptr<const ProductionNavigationObjective>& objective =
      finalization.objective;
  const ProductionMppiPredictionError& prediction = finalization.prediction;
  const MppiLivenessResult& liveness = finalization.liveness;
  const DirectTrackingManeuverUpdate& direct_tracking_maneuver =
      finalization.direct_tracking_maneuver;
  const MppiSpeedPolicyResult& speed_policy = finalization.speed_policy;
  const RouteProgressUpdate3D& route_progress = finalization.route_progress;
  const MppiEligibleRolloutUpdate& no_eligible_recovery =
      finalization.no_eligible_recovery;
  const MissionGoalCaptureResult& goal_capture = finalization.goal_capture;
  const MppiRolloutBudgetDecision& rollout_budget = finalization.rollout_budget;
  const ProductionMppiCooperativeUpdate& cooperative = finalization.cooperative;
  const ProductionMppiNonCooperativeUpdate& noncooperative =
      finalization.noncooperative;
  const RouteProgressProjection3D& route_projection = finalization.route_projection;
  const Point3& mission_goal = finalization.mission_goal;
  const std::string& target_source = finalization.target_source;
  const std::uint64_t route_generation = finalization.route_generation;
  const std::uint64_t memory_sequence = finalization.memory_sequence;
  const std::int64_t now_ns = finalization.now_ns;
  const double pose_age_ms = finalization.pose_age_ms;
  const double esdf_age_ms = finalization.esdf_age_ms;
  const double observation_age_ms = finalization.observation_age_ms;
  const double control_feedback_age_ms = finalization.control_feedback_age_ms;
  const double snapshot_ms = finalization.snapshot_ms;
  const RouteExecutionStatus3D route_execution_status =
      finalization.route_execution_status;
  const ProductionMppiPlanningState planning_state = finalization.planning_state;
  const ProductionMppiPreviousControlSource previous_control_source =
      finalization.previous_control_source;
  const mppi::RiskTier route_required_risk_tier = finalization.route_required_risk_tier;
  const bool route_usable = finalization.route_usable;
  const bool direct_tracking_interception = finalization.direct_tracking_interception;
  const bool local_route_stop_is_terminal = finalization.local_route_stop_is_terminal;
  const bool pose_predicted = finalization.pose_predicted;

  if (result.cooperative_acquisition_reseeded) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_SEPARATION_ACQUISITION_RESEED available=%s "
                "positive_progress=%s backward_fallback=%s candidate_index=%zu "
                "head_progress_m=%.3f terminal_progress_m=%.3f separation_gain_m=%.3f "
                "route_generation=%" PRIu64,
                result.cooperative_acquisition_available ? "true" : "false",
                result.cooperative_acquisition_positive_progress ? "true" : "false",
                result.cooperative_acquisition_backward_fallback ? "true" : "false",
                result.cooperative_acquisition_candidate_index,
                static_cast<double>(result.cooperative_acquisition_head_progress_m),
                static_cast<double>(result.cooperative_acquisition_terminal_progress_m),
                static_cast<double>(result.cooperative_acquisition_separation_gain_m),
                route_generation);
  }
  if (result.cooperative_release_reseeded) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_SEPARATION_RELEASE_RESEED route_generation=%" PRIu64,
                route_generation);
  }
  if (result.noncooperative_acquisition_reseeded) {
    RCLCPP_INFO(
        get_logger(),
        "NONCOOPERATIVE_SEPARATION_ACQUISITION_RESEED available=%s "
        "candidate_index=%zu maneuver=%s minimum_separation_m=%.3f "
        "separation_gain_m=%.3f head_progress_m=%.3f terminal_progress_m=%.3f "
        "lifecycle_generation=%" PRIu64,
        result.noncooperative_acquisition_available ? "true" : "false",
        result.noncooperative_acquisition_candidate_index,
        mppi::nonCooperativeManeuverName(result.noncooperative_acquisition_maneuver),
        static_cast<double>(result.noncooperative_acquisition_minimum_separation_m),
        static_cast<double>(result.noncooperative_acquisition_separation_gain_m),
        static_cast<double>(result.noncooperative_acquisition_head_progress_m),
        static_cast<double>(result.noncooperative_acquisition_terminal_progress_m),
        noncooperative.avoidance.lifecycle_generation);
  }
  if (result.noncooperative_release_reseeded) {
    RCLCPP_INFO(get_logger(),
                "NONCOOPERATIVE_SEPARATION_RELEASE_RESEED "
                "lifecycle_generation=%" PRIu64,
                noncooperative.avoidance.lifecycle_generation);
  }
  ++tick_sequence_;
  ProductionMppiExecutionPublication execution = publishExecutionHorizon(
      input, result, *world, route_execution, objective, execution_input,
      latest_lidar_evidence, finalization.offboard_session,
      finalization.offboard_session_receive_stamp_ns, planning_state, now_ns);
  const std::shared_ptr<const ExecutionPlan3D> committed_execution_snapshot =
      execution_supervisor_.plan();
  const CertifiedRouteSuffix3D* const committed_route =
      committed_execution_snapshot != nullptr ? committed_execution_snapshot->route()
                                              : nullptr;
  const bool committed_direct_owner =
      committed_execution_snapshot != nullptr &&
      committed_execution_snapshot->directTrackingExecution() != nullptr;
  const bool committed_execution_owner =
      committed_execution_snapshot != nullptr &&
      (committed_execution_snapshot->finiteExecution() != nullptr ||
       committed_direct_owner ||
       committed_execution_snapshot->stationaryHold() != nullptr);
  const bool raw_invalidation_active =
      committed_route != nullptr &&
      route_execution_status == RouteExecutionStatus3D::kRawCollision;
  const bool finite_braking_tail_active =
      execution.retained_previous_finite_path && execution.terminal_rest_state;
  const RollingRouteTelemetryObservation3D rolling_route{
      .route_generation =
          committed_route != nullptr ? committed_route->identity.generation : 0U,
      .continuity_id = committed_route != nullptr ? committed_route->continuity_id : 0U,
      .geometry_revision = committed_route != nullptr
                               ? committed_route->geometry->compiled_trajectory_revision
                               : 0U,
      .endpoint_semantics =
          committed_execution_snapshot != nullptr
              ? effectiveRouteEndpointSemantics3D(
                    executionRouteEndpointSemantics3D(*committed_execution_snapshot),
                    raw_invalidation_active, finite_braking_tail_active)
              : RouteEndpointSemantics3D::kContinuation,
      .route_remaining_m = committed_route != nullptr
                               ? committed_route->remainingM()
                               : std::numeric_limits<double>::infinity(),
      .speed_mps = routeSpeed3D(
          Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz}),
      .resident_route_available = committed_route != nullptr || committed_direct_owner,
      .execution_owner_available = committed_execution_owner,
      .endpoint_limiter_active =
          speed_policy.active_limiter == MppiSpeedLimiter::kRouteEndpoint,
      .raw_invalidation_active = raw_invalidation_active,
      .finite_braking_tail_active = finite_braking_tail_active,
      .nominal_reseeded = result.nominal_reseeded,
      .no_executable_route_hold =
          planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold,
  };
  diagnostics_sink_->recordTick(result, planning_state, execution,
                                liveness.reseed_requested ||
                                    route_progress.local_reseed_requested,
                                rolling_route);

  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  std::optional<ProductionMppiRvizSnapshot> rviz;
  if (now_ns - last_rviz_stamp_ns_ >= config_.diagnostics.rviz_period_ns) {
    std::shared_ptr<const std::vector<mppi::RouteSample3D>> rviz_route =
        route_usable ? execution_mppi_route : nullptr;
    if (direct_tracking_interception) {
      const std::vector<Point3> direct_points{
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          mission_goal,
      };
      rviz_route = mppi::adaptRouteVisualization3D(sampleRoute3D(
          direct_points,
          std::max(0.5, distance3D(direct_points.front(), direct_points.back())),
          speed_policy.reference_speed_mps));
    }
    rviz = ProductionMppiRvizSnapshot{
        .candidate_horizon = result.horizon,
        .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                         : std::vector<mppi::State>{},
        .execution_horizon = execution.horizon,
        .route = std::move(rviz_route),
        .passage_traversals = world->topology_passage_traversals,
        .selected_passage_traversal_ids = execution_selected_passage_traversal_ids,
    };
    last_rviz_stamp_ns_ = now_ns;
  }

  {
    const auto lock = evidence_boundary_.input();
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
  mppi::MppiTickResult diagnostic_result = std::move(result);
  diagnostic_result.horizon.clear();
  diagnostic_result.controls.clear();
  static_cast<void>(diagnostics_sink_->enqueue(ProductionMppiDiagnosticsSnapshot{
      .input = input,
      .result = std::move(diagnostic_result),
      .world = world,
      .world_build = finalization.world_build,
      .route_pipeline = finalization.route_pipeline,
      .execution_route = route_execution.route,
      .stability = stability,
      .prediction = prediction,
      .liveness = liveness,
      .direct_tracking_maneuver = direct_tracking_maneuver,
      .speed_policy = speed_policy,
      .route_progress = route_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = goal_capture,
      .execution = std::move(execution),
      .planning_state = planning_state,
      .rviz = std::move(rviz),
      .objective = objective,
      .target_source = target_source,
      .tick_sequence = tick_sequence_,
      .memory_sequence = memory_sequence,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .observation_age_ms = observation_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .route_station_m = route_projection.station_m,
      .route_remaining_m = route_projection.remaining_m,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .rolling_route = rolling_route,
      .route_projection_valid = route_projection.valid,
      .local_route_stop_is_terminal = local_route_stop_is_terminal,
      .liveness_reseed_requested = liveness.reseed_requested,
      .pose_predicted = pose_predicted,
      .previous_control_source = previous_control_source,
      .rollout_budget = rollout_budget,
      .cooperative = cooperative,
      .noncooperative = noncooperative,
      .route_required_risk_tier = route_required_risk_tier,
  }));
}

} // namespace drone_city_nav

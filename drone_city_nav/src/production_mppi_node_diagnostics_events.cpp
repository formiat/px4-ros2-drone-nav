#include <cinttypes>
#include <cstddef>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::logDiagnosticsEvents(
    const ProductionMppiDiagnosticsSnapshot& snapshot,
    const ConstrainedRouteObservation& route_constraint) {
  const std::shared_ptr<const ProductionNavigationObjective>& objective =
      snapshot.objective;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const mppi::MppiTickInput& input = snapshot.input;
  const ProductionMppiPreparedEsdf& esdf = snapshot.esdf;
  const MppiLivenessResult& liveness = snapshot.liveness;

  if (snapshot.liveness_reseed_requested) {
    RCLCPP_WARN(get_logger(),
                "MPPI_LIVENESS_RESEED generation=%" PRIu64
                " observation_age_s=%.3f actual_displacement_m=%.3f "
                "along_route_progress_m=%.3f route_progress_used=%s speed_mps=%.3f "
                "predicted_head_progress_m=%.3f predicted_terminal_progress_m=%.3f",
                liveness.reseed_generation, liveness.observation_age_s,
                liveness.actual_displacement_m, liveness.actual_route_progress_m,
                liveness.used_route_progress ? "true" : "false",
                liveness.actual_speed_mps, liveness.predicted_head_progress_m,
                liveness.predicted_terminal_progress_m);
  }
  if (snapshot.route_progress.local_reseed_requested) {
    RCLCPP_WARN(get_logger(),
                "PERSISTENT_ROUTE_LOCAL_RESEED route_generation=%" PRIu64
                " reseed_generation=%" PRIu64
                " observation_age_s=%.3f along_route_progress_m=%.3f "
                "predicted_head_progress_m=%.3f",
                esdf.route_generation, snapshot.route_progress.local_reseed_generation,
                snapshot.route_progress.observation_age_s,
                snapshot.route_progress.progress_m,
                snapshot.route_progress.predicted_head_progress_m);
  }
  if (snapshot.route_progress.stalled) {
    RCLCPP_WARN(get_logger(),
                "PERSISTENT_ROUTE_STALL route_generation=%" PRIu64 " reason=%s"
                " stall_generation=%" PRIu64
                " observation_age_s=%.3f along_route_progress_m=%.3f "
                "predicted_head_progress_m=%.3f",
                esdf.route_generation,
                routeProgressAction3DName(snapshot.route_progress.action),
                snapshot.route_progress.stall_generation,
                snapshot.route_progress.observation_age_s,
                snapshot.route_progress.progress_m,
                snapshot.route_progress.predicted_head_progress_m);
  }
  if (snapshot.no_eligible_recovery.route_replan_requested) {
    const bool route_replan_enabled =
        optional_constraints_.no_eligible_route_replan_enabled;
    RCLCPP_WARN(get_logger(),
                "MPPI_NO_ELIGIBLE_RECOVERY action=%s policy_enabled=%s"
                " recovery_generation=%" PRIu64 " phase=%s route_generation=%" PRIu64,
                route_replan_enabled ? "release_persistent_route"
                                     : "retain_persistent_route",
                route_replan_enabled ? "true" : "false",
                snapshot.no_eligible_recovery.no_eligible_recovery_generation,
                mppiNoEligiblePhaseName(snapshot.no_eligible_recovery.phase),
                esdf.route_generation);
  }
  if (snapshot.planning_state == ProductionMppiPlanningState::kPlanned &&
      snapshot.esdf_age_ms > maximum_esdf_age_ms_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_STALE_WORLD action=continue_resident_esdf "
                         "esdf_age_ms=%.1f warning_age_ms=%.1f revision=%" PRIu64,
                         snapshot.esdf_age_ms, maximum_esdf_age_ms_, esdf.revision);
  }
  if (snapshot.cooperative.yield.active) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "COOPERATIVE_PASSAGE_YIELD vehicle='%s' passage='%s' offset_m=%.2f "
        "offset_interval_m=[%.2f,%.2f] status=%s hold=%s queue_hold=%s "
        "hold_station_m=%.2f maximum_speed_mps=%.2f entry_not_before_ns=%" PRId64,
        vehicle_id_.c_str(), snapshot.cooperative.passage.passage_traversal_id.c_str(),
        snapshot.cooperative.passage.lateral_offset_m,
        snapshot.cooperative.passage.minimum_lateral_offset_m,
        snapshot.cooperative.passage.maximum_lateral_offset_m,
        cooperativePassageYieldStatusName(snapshot.cooperative.yield.status),
        snapshot.cooperative.yield.hold_at_entry ? "true" : "false",
        snapshot.cooperative.yield.queue_hold_station_active ? "true" : "false",
        snapshot.cooperative.yield.hold_station_m,
        snapshot.cooperative.yield.maximum_speed_mps,
        snapshot.cooperative.yield.entry_not_before_ns);
  }
  if (snapshot.goal_capture.newly_latched) {
    RCLCPP_INFO(get_logger(),
                "MISSION_GOAL_CAPTURE state=latched goal=(%.2f, %.2f, %.2f) "
                "distance_m=%.3f action=position_hold",
                mission_goal.x, mission_goal.y, mission_goal.z,
                snapshot.goal_capture.distance_m);
  }

  const bool constraint_transition =
      !last_route_constraint_observation_.has_value() ||
      last_route_constraint_observation_->route_generation !=
          route_constraint.route_generation ||
      last_route_constraint_observation_->phase != route_constraint.phase ||
      last_route_constraint_observation_->span_available !=
          route_constraint.span_available ||
      (route_constraint.span_available &&
       last_route_constraint_observation_->span_index != route_constraint.span_index);
  const bool constraint_event_relevant =
      route_constraint.span_available ||
      (last_route_constraint_observation_.has_value() &&
       last_route_constraint_observation_->span_available);
  if (constraint_transition && constraint_event_relevant) {
    RCLCPP_INFO(
        get_logger(),
        "ROUTE_CONSTRAINT_EVENT route_generation=%" PRIu64
        " phase=%s span_index=%zd span_count=%zu station_m=%.2f "
        "span_station_m=(%.2f,%.2f) distance_m=(entry:%.2f,exit:%.2f) "
        "entry=(%.2f,%.2f,%.2f) exit=(%.2f,%.2f,%.2f) "
        "z=(actual:%.2f,reference:%.2f,min:%.2f,max:%.2f,error:%.2f,ok:%s) "
        "free_space=(left:%.2f,right:%.2f,lateral_width:%.2f,vertical_height:%.2f) "
        "constraint=(lateral:%s,vertical:%s) "
        "cross_track_m=%.2f speed_mps=%.2f vz_mps=%.2f reference_speed_mps=%.2f "
        "execution_mode=%s execution_reason=%s",
        route_constraint.route_generation,
        constrainedRoutePhaseName(route_constraint.phase).data(),
        route_constraint.span_available
            ? static_cast<std::ptrdiff_t>(route_constraint.span_index)
            : static_cast<std::ptrdiff_t>(-1),
        route_constraint.span_count, route_constraint.station_m,
        route_constraint.begin_station_m, route_constraint.end_station_m,
        route_constraint.distance_to_entry_m, route_constraint.distance_to_exit_m,
        route_constraint.entry_position.x, route_constraint.entry_position.y,
        route_constraint.entry_position.z, route_constraint.exit_position.x,
        route_constraint.exit_position.y, route_constraint.exit_position.z,
        input.initial_state.z, route_constraint.reference_z_m, route_constraint.min_z_m,
        route_constraint.max_z_m, route_constraint.vertical_error_m,
        route_constraint.within_vertical_window ? "true" : "false",
        route_constraint.lateral_free_left_m, route_constraint.lateral_free_right_m,
        route_constraint.lateral_width_m, route_constraint.vertical_height_m,
        route_constraint.lateral_constrained ? "true" : "false",
        route_constraint.vertical_constrained ? "true" : "false",
        route_constraint.cross_track_error_m,
        route_constraint.actual_horizontal_speed_mps,
        route_constraint.actual_vertical_speed_mps,
        route_constraint.reference_speed_mps,
        productionMppiExecutionModeName(snapshot.execution.mode),
        productionMppiExecutionReasonName(snapshot.execution.reason));
  }
  last_route_constraint_observation_ = route_constraint;
}

} // namespace drone_city_nav

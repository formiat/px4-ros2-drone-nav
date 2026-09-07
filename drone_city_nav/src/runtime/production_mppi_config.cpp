#include "production_mppi_config.hpp"

#include "drone_city_nav/sensor_braking_contract_3d.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

bool ProductionMppiConfig::valid() const noexcept {
  const bool feasibility_budget_valid =
      !planning.persistent_planner.feasibility_first_enabled ||
      (std::isfinite(planning.persistent_planner.maximum_feasibility_compute_time_ms) &&
       planning.persistent_planner.maximum_feasibility_compute_time_ms > 0.0 &&
       planning.persistent_planner.maximum_feasibility_compute_time_ms <
           planning.persistent_planner.maximum_compute_time_ms);
  const double planning_tick_period_s = 1.0 / planning.tick_rate_hz;
  const bool planning_phase_valid =
      std::isfinite(planning.planning_tick_phase_offset_s) &&
      planning.planning_tick_phase_offset_s >= 0.0 &&
      planning.planning_tick_phase_offset_s < planning_tick_period_s;
  const bool cooperative_valid =
      (!planning.cooperative_traffic_enabled || !planning.vehicle_id.empty()) &&
      passageVolumeConfigIsValid(planning.cooperative_passage_volume) &&
      planning.cooperative_passage_route.desired_center_separation_m > 0.0 &&
      planning.cooperative_passage_route.directional_offset_fraction >= 0.0 &&
      planning.cooperative_passage_route.directional_offset_fraction <= 1.0 &&
      planning.cooperative_passage_route.preferred_transition_length_m > 0.0 &&
      planning.cooperative_passage_route.minimum_transition_length_m > 0.0 &&
      planning.cooperative_passage_route.minimum_transition_length_m <=
          planning.cooperative_passage_route.preferred_transition_length_m &&
      planning.cooperative_passage_timing.minimum_prediction_speed_mps > 0.0 &&
      planning.cooperative_passage_timing.maximum_prediction_horizon_s > 0.0 &&
      planning.cooperative_passage_yield.stopping_buffer_m >= 0.0 &&
      planning.cooperative_passage_yield.reaction_latency_s >= 0.0 &&
      planning.cooperative_passage_yield.maximum_braking_acceleration_mps2 > 0.0;
  const bool noncooperative_valid = !planning.noncooperative_avoidance_enabled ||
                                    (!planning.cooperative_traffic_enabled &&
                                     !planning.topics.noncooperative_tracks.empty());
  const bool route_extension_valid =
      staticRouteExtensionConfigValid(planning.static_route_extension) &&
      planning.route_successor_improvement.valid() &&
      futureRouteConnectorConfig3DValid(planning.future_route_connector) &&
      certifiedRouteSpliceConfig3DValid(planning.certified_route_splice);
  const bool route_search_retry_valid =
      planning.static_route_search_retry.minimum_pose_change_m > 0.0 &&
      planning.static_route_search_retry.minimum_objective_change_m > 0.0 &&
      planning.static_route_search_retry.minimum_retry_interval_s > 0.0;
  const bool rollout_budget_valid =
      control.mppi.rollouts > 0U &&
      control.rollout_budget.direct_tracking_rollouts > 0U &&
      control.rollout_budget.open_static_rollouts > 0U &&
      control.rollout_budget.direct_tracking_rollouts <=
          control.rollout_budget.open_static_rollouts &&
      control.rollout_budget.open_static_rollouts <= control.mppi.rollouts &&
      control.rollout_budget.minimum_reduced_clearance_m > 0.0F &&
      control.rollout_budget.maximum_world_age_ms > 0.0 &&
      control.rollout_budget.maximum_tracking_age_ms > 0.0;
  const bool objective_valid = !planning.mission_waypoints.empty() &&
                               planning.dynamic_objective_replan_distance_m > 0.0 &&
                               planning.dynamic_objective_replan_period_s > 0.0 &&
                               planning.tracking_objective_ray_sample_spacing_m > 0.0 &&
                               planning.tracking_capture_radius_m > 0.0 &&
                               planning.static_tracking_esdf_refresh_margin_m >= 0.0;
  const bool mission_waypoints_valid =
      objective_valid &&
      std::ranges::all_of(planning.mission_waypoints, [this](const Point3& waypoint) {
        return insideFlightEnvelope(waypoint, world.flight_envelope);
      });

  return std::isfinite(planning.tick_rate_hz) && planning.tick_rate_hz > 0.0 &&
         std::isfinite(diagnostics.rviz_rate_hz) && diagnostics.rviz_rate_hz > 0.0 &&
         std::isfinite(diagnostics.info_rate_hz) && diagnostics.info_rate_hz > 0.0 &&
         std::isfinite(diagnostics.file_rate_hz) && diagnostics.file_rate_hz > 0.0 &&
         std::isfinite(diagnostics.flush_period_s) &&
         diagnostics.flush_period_s > 0.0 && planning.deadline_ms > 0.0 &&
         execution.maximum_control_feedback_age_ms > 0.0 &&
         execution.latest_lidar_obstacle_maximum_age_ms > 0.0 &&
         std::isfinite(execution.stationary_hold_validity_s) &&
         execution.stationary_hold_validity_s >= 0.2 &&
         std::isfinite(planning.constrained_route_speed_limit_mps) &&
         planning.constrained_route_speed_limit_mps >= 0.0F &&
         std::isfinite(diagnostics.route_constraint_distance_m) &&
         diagnostics.route_constraint_distance_m >= 0.0 &&
         std::isfinite(planning.persistent_planner.minimum_horizontal_step_m) &&
         planning.persistent_planner.minimum_horizontal_step_m > 0.0 &&
         std::isfinite(planning.persistent_planner.minimum_vertical_step_m) &&
         planning.persistent_planner.minimum_vertical_step_m > 0.0 &&
         std::isfinite(planning.persistent_planner.goal_tolerance_m) &&
         planning.persistent_planner.goal_tolerance_m >= 0.0 &&
         planning.persistent_planner.maximum_extracted_path_nodes >= 2U &&
         planning.persistent_planner.maximum_shortcut_checks > 0U &&
         std::isfinite(planning.persistent_planner.maximum_compute_time_ms) &&
         planning.persistent_planner.maximum_compute_time_ms > 0.0 &&
         feasibility_budget_valid && planning_phase_valid &&
         std::isfinite(planning.route_sampling_step_m) &&
         planning.route_sampling_step_m > 0.0 &&
         std::isfinite(planning.route_completion_tolerance_m) &&
         planning.route_completion_tolerance_m > 0.0 &&
         std::isfinite(planning.static_esdf_route_lookahead_m) &&
         planning.static_esdf_route_lookahead_m > 0.0 &&
         planning.static_route_geometry.maximum_shortcut_length_m > 0.0 &&
         planning.static_route_geometry.sparse_deviation_tolerance_m >= 0.0 &&
         planning.static_route_geometry.maximum_shortcut_turn_increase_rad >= 0.0 &&
         planning.static_route_geometry.shortcut_validation_batch_size > 0U &&
         planning.static_route_geometry.corner_smoothing_distance_m >= 0.0 &&
         planning.static_route_geometry.corner_curve_samples >= 2U &&
         trackingErrorTubeConfig3DIsValid(control.tracking_error_tube) &&
         world.physical_footprint.sweep_step_m > 0.0 &&
         world.physical_footprint.radius_m >= 0.0 &&
         world.physical_footprint.lower_extent_m >= 0.0 &&
         world.physical_footprint.upper_extent_m >= 0.0 &&
         world.physical_footprint.perimeter_samples > 0U &&
         world.physical_footprint.radial_rings > 0U &&
         world.physical_footprint.axial_samples >= 2U &&
         control.mppi.costs.clearance_preference_weight >= 0.0F &&
         control.mppi.costs.route_progress_integral_weight >= 0.0F &&
         control.mppi.costs.obstacle_approach_weight >= 0.0F &&
         control.mppi.risk.tube_response_time_s > 0.0F &&
         control.mppi.risk.stopping_response_time_s >= 0.0F &&
         control.mppi.risk.stopping_deceleration_mps2 > 0.0F &&
         stoppingCapabilityIsValid(control.speed_policy.stopping_capability) &&
         sensorBrakingContract3DIsValid(control.speed_policy.sensor_braking_contract,
                                        control.speed_policy.stopping_capability) &&
         control.speed_policy.stopping_capability
                 .maximum_commanded_horizontal_deceleration_mps2 <=
             static_cast<double>(
                 control.mppi.dynamics.maximum_horizontal_acceleration_mps2) &&
         execution.finite_horizon.stopping_capability
                 .guaranteed_horizontal_deceleration_mps2 <=
             static_cast<double>(
                 control.mppi.dynamics.maximum_horizontal_acceleration_mps2) &&
         execution.finite_horizon.stopping_capability
                 .guaranteed_vertical_deceleration_mps2 <=
             static_cast<double>(
                 control.mppi.dynamics.maximum_vertical_acceleration_mps2) &&
         world.no_static_3d_esdf_update_rate_hz > 0.0 &&
         localObservedEsdfWindow3DIsValid(world.no_static_3d_esdf_window) &&
         rollout_budget_valid && mission_waypoints_valid && cooperative_valid &&
         noncooperative_valid && route_extension_valid && route_search_retry_valid &&
         navigationAngularDerivativeConfigIsValid(
             control.navigation_angular_derivative) &&
         execution.validation_policy != nullptr &&
         planning.persistent_planner.time_model.valid() &&
         std::isfinite(planning.persistent_planner.minimum_continuous_turn_alignment) &&
         planning.persistent_planner.minimum_continuous_turn_alignment >= -1.0 &&
         planning.persistent_planner.minimum_continuous_turn_alignment <= 1.0;
}

} // namespace drone_city_nav

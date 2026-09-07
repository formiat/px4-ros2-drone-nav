#include "production_mppi_execution_diagnostics.hpp"

#include "drone_city_nav/json_output.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/rolling_route_telemetry_3d.hpp"

#include <limits>
#include <sstream>

#include "production_mppi_execution_control.hpp"

namespace drone_city_nav::detail {

std::string executionInfoFields(const ProductionMppiExecutionPublication& execution) {
  std::ostringstream fields;
  fields << " execution_mode=" << productionMppiExecutionModeName(execution.mode)
         << " execution_reason=" << productionMppiExecutionReasonName(execution.reason)
         << " execution_published=" << (execution.published ? "true" : "false")
         << " retained_previous_finite_path="
         << (execution.retained_previous_finite_path ? "true" : "false")
         << " resident_owner_continues="
         << (execution.resident_owner_continues ? "true" : "false")
         << " terminal_rest_state="
         << (execution.terminal_rest_state ? "true" : "false")
         << " planned_controls=" << execution.planned_control_count
         << " nominal_prefix_controls=" << execution.nominal_prefix_control_count
         << " arrival_controls=" << execution.arrival_control_count
         << " arrival_shaping_attempts=" << execution.arrival_shaping_attempts
         << " first_control_available="
         << (execution.first_control_available ? "true" : "false") << " first_control=("
         << execution.first_control.ax << ',' << execution.first_control.ay << ','
         << execution.first_control.az << ')' << " finite_path_validation_backoff="
         << (execution.finite_path_validation_backoff ? "true" : "false")
         << " finite_path_validation_status="
         << mppi::finiteExecutionPathStatusName(execution.finite_path_validation_status)
         << " finite_path_first_failed_validation_status="
         << mppi::finiteExecutionPathStatusName(
                execution.finite_path_first_failed_validation_status)
         << " finite_path_rejected_precondition="
         << execution.finite_path_rejected_precondition
         << " latest_lidar_obstacle_fresh="
         << (execution.latest_lidar_obstacle_fresh ? "true" : "false")
         << " latest_lidar_obstacle_receive_time_fallback="
         << (execution.latest_lidar_obstacle_receive_time_fallback ? "true" : "false")
         << " latest_lidar_obstacle_sequence="
         << execution.latest_lidar_obstacle_sequence
         << " latest_lidar_obstacle_age_ms=" << execution.latest_lidar_obstacle_age_ms
         << " latest_lidar_obstacle_hits=" << execution.latest_lidar_obstacle_hit_count
         << " latest_lidar_path_validation_backoff="
         << (execution.latest_lidar_path_validation_backoff ? "true" : "false");
  return fields.str();
}

std::string executionJsonFields(const ProductionMppiExecutionPublication& execution) {
  JsonOutputStream fields;
  fields << ",\"execution_mode\":\"" << productionMppiExecutionModeName(execution.mode)
         << "\",\"execution_reason\":\""
         << productionMppiExecutionReasonName(execution.reason) << '"'
         << ",\"execution_published\":" << (execution.published ? "true" : "false")
         << ",\"retained_previous_finite_path\":"
         << (execution.retained_previous_finite_path ? "true" : "false")
         << ",\"resident_owner_continues\":"
         << (execution.resident_owner_continues ? "true" : "false")
         << ",\"terminal_rest_state\":"
         << (execution.terminal_rest_state ? "true" : "false")
         << ",\"planned_control_count\":" << execution.planned_control_count
         << ",\"nominal_prefix_control_count\":"
         << execution.nominal_prefix_control_count
         << ",\"arrival_control_count\":" << execution.arrival_control_count
         << ",\"arrival_shaping_attempts\":" << execution.arrival_shaping_attempts
         << ",\"first_control_available\":"
         << (execution.first_control_available ? "true" : "false")
         << ",\"first_control_ax_mps2\":" << execution.first_control.ax
         << ",\"first_control_ay_mps2\":" << execution.first_control.ay
         << ",\"first_control_az_mps2\":" << execution.first_control.az
         << ",\"finite_path_validation_backoff\":"
         << (execution.finite_path_validation_backoff ? "true" : "false")
         << ",\"finite_path_validation_status\":\""
         << mppi::finiteExecutionPathStatusName(execution.finite_path_validation_status)
         << '\"' << ",\"finite_path_first_failed_validation_status\":\""
         << mppi::finiteExecutionPathStatusName(
                execution.finite_path_first_failed_validation_status)
         << '\"' << ",\"finite_path_rejected_precondition\":\""
         << execution.finite_path_rejected_precondition << '\"'
         << ",\"latest_lidar_obstacle_fresh\":"
         << (execution.latest_lidar_obstacle_fresh ? "true" : "false")
         << ",\"latest_lidar_obstacle_receive_time_fallback\":"
         << (execution.latest_lidar_obstacle_receive_time_fallback ? "true" : "false")
         << ",\"latest_lidar_obstacle_sequence\":"
         << execution.latest_lidar_obstacle_sequence
         << ",\"latest_lidar_obstacle_age_ms\":"
         << execution.latest_lidar_obstacle_age_ms
         << ",\"latest_lidar_obstacle_hit_count\":"
         << execution.latest_lidar_obstacle_hit_count
         << ",\"latest_lidar_path_validation_backoff\":"
         << (execution.latest_lidar_path_validation_backoff ? "true" : "false");
  return fields.str();
}

std::string
rollingRouteInfoFields(const RollingRouteTelemetryObservation3D& observation) {
  std::ostringstream fields;
  fields << " route_endpoint_semantics="
         << routeEndpointSemantics3DName(observation.endpoint_semantics)
         << " route_continuity_id=" << observation.continuity_id
         << " route_geometry_revision=" << observation.geometry_revision
         << " route_speed_mps=" << observation.speed_mps << " resident_route_available="
         << (observation.resident_route_available ? "true" : "false")
         << " execution_owner_available="
         << (observation.execution_owner_available ? "true" : "false")
         << " endpoint_limiter_active="
         << (observation.endpoint_limiter_active ? "true" : "false")
         << " raw_invalidation_active="
         << (observation.raw_invalidation_active ? "true" : "false")
         << " finite_braking_tail_active="
         << (observation.finite_braking_tail_active ? "true" : "false");
  return fields.str();
}

std::string
rollingRouteJsonFields(const RollingRouteTelemetryObservation3D& observation) {
  JsonOutputStream fields;
  fields << ",\"route_endpoint_semantics\":\""
         << routeEndpointSemantics3DName(observation.endpoint_semantics) << '"'
         << ",\"route_continuity_id\":" << observation.continuity_id
         << ",\"route_geometry_revision\":" << observation.geometry_revision
         << ",\"route_speed_mps\":" << observation.speed_mps
         << ",\"resident_route_available\":"
         << (observation.resident_route_available ? "true" : "false")
         << ",\"execution_owner_available\":"
         << (observation.execution_owner_available ? "true" : "false")
         << ",\"endpoint_limiter_active\":"
         << (observation.endpoint_limiter_active ? "true" : "false")
         << ",\"raw_invalidation_active\":"
         << (observation.raw_invalidation_active ? "true" : "false")
         << ",\"finite_braking_tail_active\":"
         << (observation.finite_braking_tail_active ? "true" : "false");
  return fields.str();
}

std::string executedHorizonClearanceJsonFields(
    const std::optional<ExecutedHorizonClearance3D>& clearance) {
  JsonOutputStream fields;
  const bool constrained = clearance.has_value() && clearance->constrained();
  fields << ",\"executed_horizon_clearance_available\":"
         << (clearance.has_value() && clearance->available ? "true" : "false")
         << ",\"executed_horizon_minimum_clearance_m\":"
         << (clearance.has_value() ? clearance->minimum_clearance_m
                                   : std::numeric_limits<double>::infinity())
         << ",\"executed_horizon_constrained_clearance_m\":"
         << (constrained ? clearance->constrainedClearanceM()
                         : std::numeric_limits<double>::infinity())
         << ",\"executed_horizon_distance_to_constraint_m\":"
         << (constrained ? clearance->distanceToConstraintM()
                         : std::numeric_limits<double>::infinity())
         << ",\"executed_horizon_constrained_samples\":[";
  if (constrained) {
    bool first = true;
    for (const ConstrainedHorizonSample3D& sample : clearance->constrained_samples) {
      fields << (first ? "" : ",") << '[' << sample.distance_m << ','
             << sample.clearance_m << ']';
      first = false;
    }
  }
  fields << ']';
  return fields.str();
}

std::string rolloutCostTermsJson(const mppi::RolloutCostTerms& terms) {
  JsonOutputStream json;
  json << "{\"soft_cost\":" << terms.soft_cost
       << ",\"head_progress\":" << terms.head_progress
       << ",\"progress\":" << terms.progress
       << ",\"route_progress_integral\":" << terms.route_progress_integral
       << ",\"speed_tracking\":" << terms.speed_tracking
       << ",\"overspeed\":" << terms.overspeed
       << ",\"guide_deviation\":" << terms.guide_deviation
       << ",\"altitude_tracking\":" << terms.altitude_tracking
       << ",\"acceleration\":" << terms.acceleration << ",\"jerk\":" << terms.jerk
       << ",\"yaw_change\":" << terms.yaw_change
       << ",\"dynamic_aircraft\":" << terms.dynamic_aircraft
       << ",\"maneuver_preference\":" << terms.maneuver_preference
       << ",\"clearance_preference\":" << terms.clearance_preference
       << ",\"obstacle_approach\":" << terms.obstacle_approach
       << ",\"stopping_deficit\":" << terms.stopping_deficit
       << ",\"terminal\":" << terms.terminal
       << ",\"minimum_clearance_m\":" << terms.minimum_clearance_m
       << ",\"contact_distance_m\":" << terms.contact_distance_m
       << ",\"head_speed_mps\":" << terms.head_speed_mps
       << ",\"collision\":" << (terms.collision ? "true" : "false")
       << ",\"altitude_envelope_violation\":"
       << (terms.altitude_envelope_violation ? "true" : "false") << '}';
  return json.str();
}

} // namespace drone_city_nav::detail

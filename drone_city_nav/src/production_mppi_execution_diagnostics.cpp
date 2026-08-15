#include "production_mppi_execution_diagnostics.hpp"

#include <sstream>

#include "production_mppi_node.hpp"

namespace drone_city_nav::detail {

std::string executionInfoFields(const ProductionMppiExecutionPublication& execution) {
  std::ostringstream fields;
  fields << " execution_mode=" << productionMppiExecutionModeName(execution.mode)
         << " execution_reason=" << productionMppiExecutionReasonName(execution.reason)
         << " execution_published=" << (execution.published ? "true" : "false")
         << " retained_previous_finite_path="
         << (execution.retained_previous_finite_path ? "true" : "false")
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
         << " latest_lidar_obstacle_fresh="
         << (execution.latest_lidar_obstacle_fresh ? "true" : "false")
         << " latest_lidar_obstacle_sequence="
         << execution.latest_lidar_obstacle_sequence
         << " latest_lidar_obstacle_age_ms=" << execution.latest_lidar_obstacle_age_ms
         << " latest_lidar_obstacle_hits=" << execution.latest_lidar_obstacle_hit_count
         << " latest_lidar_path_validation_backoff="
         << (execution.latest_lidar_path_validation_backoff ? "true" : "false");
  return fields.str();
}

std::string executionJsonFields(const ProductionMppiExecutionPublication& execution) {
  std::ostringstream fields;
  fields << ",\"execution_mode\":\"" << productionMppiExecutionModeName(execution.mode)
         << "\",\"execution_reason\":\""
         << productionMppiExecutionReasonName(execution.reason) << '"'
         << ",\"execution_published\":" << (execution.published ? "true" : "false")
         << ",\"retained_previous_finite_path\":"
         << (execution.retained_previous_finite_path ? "true" : "false")
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
         << ",\"latest_lidar_obstacle_fresh\":"
         << (execution.latest_lidar_obstacle_fresh ? "true" : "false")
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

} // namespace drone_city_nav::detail

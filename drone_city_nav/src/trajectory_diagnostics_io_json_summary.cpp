#include "trajectory_diagnostics_io_internal.hpp"

namespace drone_city_nav {

using namespace trajectory_diagnostics_io_detail;

std::string
finalTrajectoryDiagnosticsSummaryJson(const TrajectoryPlannerStats& stats,
                                      const TrajectoryShapeDiagnostics& shape) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "{" << trajectoryTimingDiagnosticsJsonFields(stats);
  stream << ",\"trajectory_quality\":\"" << trajectoryQualityName(stats.quality)
         << "\"";
  appendJsonSize(stream, "corridor_parallel_workers_used",
                 stats.corridor.parallel_workers_used);
  appendJsonBool(stream, "corridor_samples_reused", stats.corridor.samples_reused);
  appendJsonSize(stream, "corridor_reused_samples", stats.corridor.reused_samples);
  appendJsonUint64(stream, "corridor_route_fingerprint",
                   stats.corridor.route_fingerprint);
  appendJsonUint64(stream, "corridor_config_fingerprint",
                   stats.corridor.config_fingerprint);
  appendJsonUint64(stream, "corridor_grid_cells_hash",
                   stats.corridor.prohibited_grid_fingerprint.cells_hash);
  appendJsonUint64(stream, "corridor_grid_inflated_hash",
                   stats.corridor.prohibited_grid_fingerprint.inflated_hash);
  appendJsonNumber(stream, "corridor_sample_build_duration_ms",
                   stats.corridor.sample_build_duration_ms);
  appendJsonNumber(stream, "corridor_raycast_duration_ms",
                   stats.corridor.raycast_duration_ms);
  appendJsonNumber(stream, "corridor_lateral_limit_duration_ms",
                   stats.corridor.lateral_limit_duration_ms);
  appendJsonNumber(stream, "corridor_clearance_field_build_ms",
                   stats.corridor.clearance_field_build_duration_ms);
  appendJsonBool(stream, "clearance_field_reused_by_corridor",
                 stats.corridor.clearance_field_reused);
  appendJsonBool(stream, "corridor_clearance_field_cache_hit",
                 stats.corridor.clearance_field_cache_hit);
  stream << "," << trajectoryOptimizerDiagnosticsJsonFields(stats);
  stream << "," << turnSmoothingDiagnosticsJsonFields(stats);
  stream << "," << speedProfileConstraintDiagnosticsJsonFields(stats);
  appendJsonSize(stream, "trajectory_shape_segment_count", shape.segment_count);
  appendJsonNumber(stream, "trajectory_shape_max_curvature_jump_1pm",
                   shape.max_curvature_jump_1pm);
  appendJsonNumber(stream, "trajectory_shape_max_heading_delta_rad",
                   shape.max_heading_delta_rad);
  appendJsonNumber(stream, "trajectory_shape_max_offset_delta_m",
                   shape.max_offset_delta_m);
  stream << "}";
  return stream.str();
}

std::string trajectoryPlannerDiagnosticsJson(const std::uint64_t planner_path_id,
                                             const std::uint64_t path_stamp_ns,
                                             const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "{\"planner_path_id\":" << planner_path_id;
  appendJsonUint64(stream, "path_stamp_ns", path_stamp_ns);
  stream << ",\"trajectory_status\":\"" << trajectoryPlannerStatusName(stats.status)
         << "\"";
  stream << ",\"trajectory_quality\":\"" << trajectoryQualityName(stats.quality)
         << "\"";
  appendJsonSize(stream, "trajectory_input_points", stats.input_points);
  appendJsonSize(stream, "trajectory_compact_segments", stats.compact_segments);
  appendJsonSize(stream, "trajectory_line_segments", stats.line_segments);
  appendJsonSize(stream, "trajectory_arc_segments", stats.arc_segments);
  appendJsonSize(stream, "trajectory_samples", stats.samples);
  appendJsonNumber(stream, "trajectory_length_m", stats.length_m);
  appendJsonNumber(stream, "curvature_min_1pm", stats.curvature_min_1pm);
  appendJsonNumber(stream, "curvature_max_1pm", stats.curvature_max_1pm);
  appendJsonNumber(stream, "curvature_mean_abs_1pm", stats.curvature_mean_abs_1pm);
  appendJsonNumber(stream, "speed_profile_min_mps", stats.speed_profile_min_mps);
  appendJsonNumber(stream, "speed_profile_max_mps", stats.speed_profile_max_mps);
  appendJsonNumber(stream, "speed_profile_mean_mps", stats.speed_profile_mean_mps);
  appendJsonNumber(stream, "planning_speed_profile_min_mps",
                   stats.speed_profile_min_mps);
  appendJsonNumber(stream, "planning_speed_profile_max_mps",
                   stats.speed_profile_max_mps);
  appendJsonNumber(stream, "planning_speed_profile_mean_mps",
                   stats.speed_profile_mean_mps);
  appendJsonSize(stream, "speed_profile_curvature_limited_samples",
                 stats.speed_profile_curvature_limited_samples);
  appendJsonSize(stream, "planning_speed_profile_curvature_limited_samples",
                 stats.speed_profile_curvature_limited_samples);
  appendJsonUint64(stream, "planning_speed_profile_construction_config_fingerprint",
                   stats.speed_profile_construction_config_fingerprint);
  appendJsonUint64(stream, "planning_runtime_speed_policy_config_fingerprint",
                   stats.runtime_speed_policy_config_fingerprint);
  appendJsonUint64(stream, "planning_runtime_velocity_control_config_fingerprint",
                   stats.runtime_velocity_control_config_fingerprint);
  stream << "," << speedProfileConstraintDiagnosticsJsonFields(stats);
  stream << "," << trajectoryTimingDiagnosticsJsonFields(stats);
  appendJsonSize(stream, "corridor_input_points", stats.corridor.input_points);
  appendJsonSize(stream, "corridor_samples", stats.corridor.samples);
  appendJsonSize(stream, "corridor_route_prohibited_samples",
                 stats.corridor.route_prohibited_samples);
  appendJsonSize(stream, "corridor_center_recovered_samples",
                 stats.corridor.center_recovered_samples);
  appendJsonSize(stream, "corridor_center_unrecoverable_samples",
                 stats.corridor.center_unrecoverable_samples);
  appendJsonSize(stream, "corridor_outside_grid_samples",
                 stats.corridor.outside_grid_samples);
  appendJsonSize(stream, "corridor_lateral_limited_samples",
                 stats.corridor.lateral_limited_samples);
  appendJsonNumber(stream, "corridor_width_min_m", stats.corridor.min_width_m);
  appendJsonNumber(stream, "corridor_width_mean_m", stats.corridor.mean_width_m);
  appendJsonNumber(stream, "corridor_width_max_m", stats.corridor.max_width_m);
  appendJsonNumber(stream, "corridor_clearance_min_m", stats.corridor.min_clearance_m);
  appendJsonNumber(stream, "corridor_clearance_mean_m",
                   stats.corridor.mean_clearance_m);
  appendJsonNumber(stream, "corridor_clearance_max_m", stats.corridor.max_clearance_m);
  appendJsonNumber(stream, "corridor_center_recovery_max_m",
                   stats.corridor.max_center_recovery_m);
  appendJsonNumber(stream, "corridor_lateral_reduction_max_m",
                   stats.corridor.max_lateral_bound_reduction_m);
  appendJsonSize(stream, "corridor_parallel_workers_used",
                 stats.corridor.parallel_workers_used);
  appendJsonBool(stream, "corridor_samples_reused", stats.corridor.samples_reused);
  appendJsonSize(stream, "corridor_reused_samples", stats.corridor.reused_samples);
  appendJsonUint64(stream, "corridor_route_fingerprint",
                   stats.corridor.route_fingerprint);
  appendJsonUint64(stream, "corridor_config_fingerprint",
                   stats.corridor.config_fingerprint);
  appendJsonUint64(stream, "corridor_grid_cells_hash",
                   stats.corridor.prohibited_grid_fingerprint.cells_hash);
  appendJsonUint64(stream, "corridor_grid_inflated_hash",
                   stats.corridor.prohibited_grid_fingerprint.inflated_hash);
  appendJsonNumber(stream, "corridor_sample_build_duration_ms",
                   stats.corridor.sample_build_duration_ms);
  appendJsonNumber(stream, "corridor_raycast_duration_ms",
                   stats.corridor.raycast_duration_ms);
  appendJsonNumber(stream, "corridor_lateral_limit_duration_ms",
                   stats.corridor.lateral_limit_duration_ms);
  appendJsonNumber(stream, "corridor_clearance_field_build_ms",
                   stats.corridor.clearance_field_build_duration_ms);
  appendJsonBool(stream, "clearance_field_reused_by_corridor",
                 stats.corridor.clearance_field_reused);
  appendJsonBool(stream, "corridor_clearance_field_cache_hit",
                 stats.corridor.clearance_field_cache_hit);
  appendJsonSize(stream, "trajectory_optimizer_input_samples",
                 stats.trajectory_optimizer.input_samples);
  appendJsonSize(stream, "trajectory_optimizer_optimizer_samples",
                 stats.trajectory_optimizer.optimizer_samples);
  appendJsonSize(stream, "trajectory_optimizer_output_samples",
                 stats.trajectory_optimizer.output_samples);
  appendJsonSize(stream, "trajectory_optimizer_iterations",
                 stats.trajectory_optimizer.iterations);
  appendJsonSize(stream, "trajectory_optimizer_candidate_evaluations",
                 stats.trajectory_optimizer.candidate_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_collision_rejections",
                 stats.trajectory_optimizer.collision_rejections);
  appendJsonNumber(stream, "trajectory_optimizer_cost_initial",
                   stats.trajectory_optimizer.initial_cost);
  appendJsonNumber(stream, "trajectory_optimizer_cost_final",
                   stats.trajectory_optimizer.final_cost);
  stream << "," << trajectoryOptimizerDiagnosticsJsonFields(stats);
  stream << "," << turnSmoothingDiagnosticsJsonFields(stats);
  stream << "," << speedProfileConstraintDiagnosticsJsonFields(stats);
  stream << "}";
  return stream.str();
}

} // namespace drone_city_nav

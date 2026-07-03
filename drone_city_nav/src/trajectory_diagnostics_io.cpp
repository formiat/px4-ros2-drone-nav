#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace drone_city_nav {
namespace {

constexpr double kTinyCurvature = 1.0e-6;

void writeCsvNumberOrEmpty(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
  }
}

void writeJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

void appendJsonNumber(std::ostream& stream, const std::string_view key,
                      const double value) {
  stream << ",\"" << key << "\":";
  writeJsonNumberOrNull(stream, value);
}

void appendJsonSize(std::ostream& stream, const std::string_view key,
                    const std::size_t value) {
  stream << ",\"" << key << "\":" << value;
}

void appendJsonBool(std::ostream& stream, const std::string_view key,
                    const bool value) {
  stream << ",\"" << key << "\":" << (value ? "true" : "false");
}

void appendJsonString(std::ostream& stream, const std::string_view key,
                      const std::string_view value) {
  stream << ",\"" << key << "\":\"" << value << "\"";
}

void appendJsonUint64(std::ostream& stream, const std::string_view key,
                      const std::uint64_t value) {
  stream << ",\"" << key << "\":" << value;
}

[[nodiscard]] std::optional<std::string_view>
jsonValueForKey(const std::string_view json, const std::string_view key) {
  const std::string pattern = "\"" + std::string{key} + "\":";
  const std::size_t key_position = json.find(pattern);
  if (key_position == std::string_view::npos) {
    return std::nullopt;
  }
  std::size_t value_begin = key_position + pattern.size();
  while (value_begin < json.size() &&
         (json[value_begin] == ' ' || json[value_begin] == '\t' ||
          json[value_begin] == '\n' || json[value_begin] == '\r')) {
    ++value_begin;
  }
  if (value_begin >= json.size()) {
    return std::nullopt;
  }

  if (json[value_begin] == '"') {
    const std::size_t string_end = json.find('"', value_begin + 1U);
    if (string_end == std::string_view::npos) {
      return std::nullopt;
    }
    return json.substr(value_begin + 1U, string_end - value_begin - 1U);
  }

  std::size_t value_end = value_begin;
  while (value_end < json.size() && json[value_end] != ',' && json[value_end] != '}') {
    ++value_end;
  }
  while (value_end > value_begin &&
         (json[value_end - 1U] == ' ' || json[value_end - 1U] == '\t' ||
          json[value_end - 1U] == '\n' || json[value_end - 1U] == '\r')) {
    --value_end;
  }
  return json.substr(value_begin, value_end - value_begin);
}

void parseJsonDouble(const std::string_view json, const std::string_view key,
                     double& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value()) {
    return;
  }
  if (*value == "null") {
    output = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const std::string value_string{*value};
  char* end = nullptr;
  const double parsed = std::strtod(value_string.c_str(), &end);
  if (end != value_string.c_str() && *end == '\0') {
    output = parsed;
  }
}

void parseJsonSize(const std::string_view json, const std::string_view key,
                   std::size_t& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value() || *value == "null") {
    return;
  }

  const std::string value_string{*value};
  char* end = nullptr;
  const double parsed = std::strtod(value_string.c_str(), &end);
  if (end != value_string.c_str() && *end == '\0' && std::isfinite(parsed) &&
      parsed >= 0.0) {
    output = static_cast<std::size_t>(parsed);
  }
}

[[nodiscard]] bool parseJsonUint64(const std::string_view json,
                                   const std::string_view key, std::uint64_t& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value() || *value == "null") {
    return false;
  }
  const std::string value_string{*value};
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value_string.c_str(), &end, 10);
  if (end == value_string.c_str() || *end != '\0') {
    return false;
  }
  output = static_cast<std::uint64_t>(parsed);
  return true;
}

void parseJsonBool(const std::string_view json, const std::string_view key,
                   bool& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value()) {
    return;
  }
  if (*value == "true") {
    output = true;
    return;
  }
  if (*value == "false") {
    output = false;
  }
}

[[nodiscard]] TrajectoryPlannerStatus
parseTrajectoryPlannerStatusName(const std::string_view value) {
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kInvalidRoute)) {
    return TrajectoryPlannerStatus::kInvalidRoute;
  }
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kMissingGrid)) {
    return TrajectoryPlannerStatus::kMissingGrid;
  }
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kCorridorInvalid)) {
    return TrajectoryPlannerStatus::kCorridorInvalid;
  }
  if (value == trajectoryPlannerStatusName(
                   TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid)) {
    return TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid;
  }
  if (value ==
      trajectoryPlannerStatusName(TrajectoryPlannerStatus::kInvalidTrajectory)) {
    return TrajectoryPlannerStatus::kInvalidTrajectory;
  }
  return TrajectoryPlannerStatus::kOk;
}

[[nodiscard]] TrajectoryQuality
parseTrajectoryQualityName(const std::string_view value) {
  if (value == trajectoryQualityName(TrajectoryQuality::kBaseline)) {
    return TrajectoryQuality::kBaseline;
  }
  if (value == trajectoryQualityName(TrajectoryQuality::kRefined)) {
    return TrajectoryQuality::kRefined;
  }
  return TrajectoryQuality::kUnknown;
}

[[nodiscard]] SpeedConstraintType
parseSpeedConstraintTypeName(const std::string_view value) {
  if (value == speedConstraintTypeName(SpeedConstraintType::kArc)) {
    return SpeedConstraintType::kArc;
  }
  if (value == speedConstraintTypeName(SpeedConstraintType::kGoal)) {
    return SpeedConstraintType::kGoal;
  }
  return SpeedConstraintType::kNone;
}

[[nodiscard]] std::string speedProfileTopConstraintPrefix(const std::size_t index) {
  return "speed_profile_top" + std::to_string(index + 1U) + "_";
}

std::string
speedProfileConstraintDiagnosticsJsonFieldsImpl(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"speed_profile_top_constraint_count\":"
         << stats.top_speed_constraints.size();
  for (std::size_t i = 0U; i < stats.top_speed_constraints.size(); ++i) {
    const SpeedProfileConstraintDiagnostic& constraint = stats.top_speed_constraints[i];
    const std::string prefix = speedProfileTopConstraintPrefix(i);
    appendJsonSize(stream, prefix + "sample_index", constraint.sample_index);
    appendJsonNumber(stream, prefix + "s_m", constraint.s_m);
    appendJsonNumber(stream, prefix + "radius_m", constraint.radius_m);
    appendJsonNumber(stream, prefix + "curvature_1pm", constraint.curvature_1pm);
    appendJsonNumber(stream, prefix + "speed_limit_mps", constraint.speed_limit_mps);
    appendJsonNumber(stream, prefix + "profiled_limit_mps",
                     constraint.profiled_limit_mps);
    appendJsonString(stream, prefix + "source",
                     speedConstraintTypeName(constraint.source));
    appendJsonBool(stream, prefix + "isolated_curvature_spike",
                   constraint.isolated_curvature_spike);
  }
  appendJsonSize(stream, "isolated_curvature_spike_candidates",
                 stats.isolated_curvature_spike_candidates);
  appendJsonSize(stream, "isolated_curvature_spikes_smoothed_geometry",
                 stats.isolated_curvature_spikes_smoothed_geometry);
  appendJsonSize(stream, "isolated_curvature_spikes_smoothed_speed_profile",
                 stats.isolated_curvature_spikes_smoothed_speed_profile);
  appendJsonNumber(stream, "isolated_curvature_spike_max_before_1pm",
                   stats.isolated_curvature_spike_max_before_1pm);
  appendJsonNumber(stream, "isolated_curvature_spike_max_after_1pm",
                   stats.isolated_curvature_spike_max_after_1pm);
  return stream.str();
}

} // namespace

std::string
speedProfileConstraintDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  return speedProfileConstraintDiagnosticsJsonFieldsImpl(stats);
}

std::string finalTrajectorySamplesCsvHeader() {
  return "sample_index,s_m,x,y,tangent_x,tangent_y,curvature_1pm,"
         "arc_radius_m,left_bound_m,right_bound_m,lateral_offset_m,"
         "speed_geometric_limit_mps,speed_profiled_limit_mps,speed_reason,"
         "speed_limit_source,constraint_s_m,constraint_limit_mps,"
         "profiled_time_from_start_s,profiled_time_to_finish_s";
}

std::string finalTrajectorySamplesCsvRow(const std::size_t sample_index,
                                         const TrajectoryPointSample& sample,
                                         const TrajectorySpeedSample& speed_sample,
                                         const double time_from_start_s,
                                         const double time_to_finish_s) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << sample_index << ",";
  writeCsvNumberOrEmpty(stream, sample.s_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.point.x);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.point.y);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.tangent.x);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.tangent.y);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.curvature_1pm);
  stream << ",";
  if (std::abs(sample.curvature_1pm) > kTinyCurvature) {
    writeCsvNumberOrEmpty(stream, 1.0 / std::abs(sample.curvature_1pm));
  }
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.left_bound_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.right_bound_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.lateral_offset_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.geometric_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.profiled_limit_mps);
  stream << "," << speedConstraintTypeName(speed_sample.reason) << ","
         << speedConstraintTypeName(speed_sample.reason) << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.constraint_s_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.constraint_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, time_from_start_s);
  stream << ",";
  writeCsvNumberOrEmpty(stream, time_to_finish_s);
  return stream.str();
}

std::string
trajectoryOptimizerDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"trajectory_optimizer_final_estimated_time_s\":";
  writeJsonNumberOrNull(stream, stats.trajectory_optimizer.estimated_time_s);
  appendJsonNumber(stream, "trajectory_optimizer_final_min_speed_limit_mps",
                   stats.trajectory_optimizer.min_speed_limit_mps);
  appendJsonNumber(stream, "trajectory_optimizer_final_max_speed_limit_mps",
                   stats.trajectory_optimizer.max_speed_limit_mps);
  appendJsonSize(stream, "trajectory_optimizer_final_curvature_limited_samples",
                 stats.trajectory_optimizer.curvature_limited_samples);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_length_m",
                   stats.trajectory_optimizer.centerline_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_final_length_m",
                   stats.trajectory_optimizer.final_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_final_length_ratio",
                   stats.trajectory_optimizer.final_length_ratio);
  appendJsonNumber(stream, "trajectory_optimizer_max_abs_offset_m",
                   stats.trajectory_optimizer.max_abs_offset_m);
  appendJsonNumber(stream, "trajectory_optimizer_min_edge_margin_m",
                   stats.trajectory_optimizer.min_edge_margin_m);
  appendJsonNumber(stream, "trajectory_optimizer_mean_edge_margin_m",
                   stats.trajectory_optimizer.mean_edge_margin_m);
  appendJsonNumber(stream, "trajectory_optimizer_cost_curvature",
                   stats.trajectory_optimizer.cost_curvature);
  appendJsonNumber(stream, "trajectory_optimizer_cost_curvature_change",
                   stats.trajectory_optimizer.cost_curvature_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_radius_shortfall",
                   stats.trajectory_optimizer.cost_radius_shortfall);
  appendJsonNumber(stream, "trajectory_optimizer_cost_heading_jump",
                   stats.trajectory_optimizer.cost_heading_jump);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_change",
                   stats.trajectory_optimizer.cost_offset_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_second_change",
                   stats.trajectory_optimizer.cost_offset_second_change);
  appendJsonNumber(stream, "trajectory_optimizer_cost_offset_slope",
                   stats.trajectory_optimizer.cost_offset_slope);
  appendJsonNumber(stream, "trajectory_optimizer_cost_collision",
                   stats.trajectory_optimizer.cost_collision);
  appendJsonNumber(stream, "trajectory_optimizer_cost_outside_grid",
                   stats.trajectory_optimizer.cost_outside_grid);
  appendJsonNumber(stream, "trajectory_optimizer_best_candidate_score",
                   stats.trajectory_optimizer.best_candidate_score);
  appendJsonSize(stream, "trajectory_optimizer_regularization_iterations",
                 stats.trajectory_optimizer.regularization_iterations);
  stream << ",\"trajectory_optimizer_regularization_applied\":"
         << (stats.trajectory_optimizer.regularization_applied ? "true" : "false");
  appendJsonNumber(
      stream, "trajectory_optimizer_pre_regularization_max_curvature_jump_1pm",
      stats.trajectory_optimizer.pre_regularization_max_curvature_jump_1pm);
  appendJsonNumber(
      stream, "trajectory_optimizer_post_regularization_max_curvature_jump_1pm",
      stats.trajectory_optimizer.post_regularization_max_curvature_jump_1pm);
  appendJsonSize(stream, "trajectory_optimizer_skipped_noop_candidates",
                 stats.trajectory_optimizer.skipped_noop_candidates);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_path_evaluation_duration_ms",
                   stats.trajectory_optimizer.candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_score_duration_ms",
                   stats.trajectory_optimizer.candidate_score_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_point_build_duration_ms",
                   stats.trajectory_optimizer.candidate_point_build_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_sample_build_duration_ms",
                   stats.trajectory_optimizer.candidate_sample_build_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_cost_breakdown_duration_ms",
                   stats.trajectory_optimizer.candidate_cost_breakdown_duration_ms);
  appendJsonNumber(stream,
                   "trajectory_optimizer_candidate_shape_diagnostics_duration_ms",
                   stats.trajectory_optimizer.candidate_shape_diagnostics_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_regularization_duration_ms",
                   stats.trajectory_optimizer.regularization_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_scratch_reused_candidates",
                 stats.trajectory_optimizer.scratch_reused_candidates);
  appendJsonBool(stream, "trajectory_optimizer_parallel_candidate_evaluation_used",
                 stats.trajectory_optimizer.parallel_candidate_evaluation_used);
  appendJsonSize(stream, "trajectory_optimizer_parallel_workers_used",
                 stats.trajectory_optimizer.parallel_workers_used);
  appendJsonSize(stream, "trajectory_optimizer_candidate_chunks",
                 stats.trajectory_optimizer.candidate_chunks);
  appendJsonSize(stream, "trajectory_optimizer_candidate_parallel_batches",
                 stats.trajectory_optimizer.candidate_parallel_batches);
  appendJsonSize(stream, "trajectory_optimizer_candidate_threads_launched",
                 stats.trajectory_optimizer.candidate_threads_launched);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_batch_wall_duration_ms",
                   stats.trajectory_optimizer.candidate_batch_wall_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_batch_wait_duration_ms",
                   stats.trajectory_optimizer.candidate_batch_wait_duration_ms);
  appendJsonNumber(
      stream, "trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms",
      stats.trajectory_optimizer.candidate_worker_buffer_prepare_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_candidate_thread_launch_duration_ms",
                   stats.trajectory_optimizer.candidate_thread_launch_duration_ms);
  appendJsonNumber(stream,
                   "trajectory_optimizer_candidate_thread_join_wait_duration_ms",
                   stats.trajectory_optimizer.candidate_thread_join_wait_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_worker_scratch_reuses",
                 stats.trajectory_optimizer.worker_scratch_reuses);
  appendJsonSize(stream, "trajectory_optimizer_candidate_snapshot_allocations_avoided",
                 stats.trajectory_optimizer.candidate_snapshot_allocations_avoided);
  appendJsonSize(stream, "trajectory_optimizer_candidate_offset_changed_samples_total",
                 stats.trajectory_optimizer.candidate_offset_changed_samples_total);
  appendJsonSize(stream, "trajectory_optimizer_candidate_offset_changed_samples_max",
                 stats.trajectory_optimizer.candidate_offset_changed_samples_max);
  appendJsonSize(
      stream, "trajectory_optimizer_candidate_offset_changed_span_samples_total",
      stats.trajectory_optimizer.candidate_offset_changed_span_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_offset_changed_span_samples_max",
                 stats.trajectory_optimizer.candidate_offset_changed_span_samples_max);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_local_speed_window_samples_total",
                 stats.trajectory_optimizer.candidate_local_speed_window_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_candidate_local_speed_window_samples_max",
                 stats.trajectory_optimizer.candidate_local_speed_window_samples_max);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_evaluations",
                 stats.trajectory_optimizer.local_candidate_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_full_score_fallbacks",
                 stats.trajectory_optimizer.local_candidate_full_score_fallbacks);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_full_score_required",
                 stats.trajectory_optimizer.local_candidate_full_score_required);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_invalid_input",
      stats.trajectory_optimizer.local_candidate_full_score_required_invalid_input);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_boundary",
      stats.trajectory_optimizer.local_candidate_full_score_required_boundary);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_unsafe_base",
      stats.trajectory_optimizer.local_candidate_full_score_required_unsafe_base);
  appendJsonSize(
      stream, "trajectory_optimizer_local_candidate_full_score_required_window_invalid",
      stats.trajectory_optimizer.local_candidate_full_score_required_window_invalid);
  appendJsonSize(stream, "trajectory_optimizer_local_candidate_acceptance_full_scores",
                 stats.trajectory_optimizer.local_candidate_acceptance_full_scores);
  appendJsonSize(stream, "trajectory_optimizer_local_score_false_positives",
                 stats.trajectory_optimizer.local_score_false_positives);
  appendJsonNumber(stream,
                   "trajectory_optimizer_local_candidate_point_build_duration_ms",
                   stats.trajectory_optimizer.local_candidate_point_build_duration_ms);
  appendJsonNumber(
      stream, "trajectory_optimizer_local_candidate_path_evaluation_duration_ms",
      stats.trajectory_optimizer.local_candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_local_candidate_score_duration_ms",
                   stats.trajectory_optimizer.local_candidate_score_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_full_candidate_score_duration_ms",
                   stats.trajectory_optimizer.full_candidate_score_duration_ms);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_lower_bound_validation_full_scores",
                 stats.trajectory_optimizer.shadow_lower_bound_validation_full_scores);
  appendJsonNumber(
      stream,
      "trajectory_optimizer_shadow_lower_bound_validation_full_score_duration_ms",
      stats.trajectory_optimizer.shadow_lower_bound_validation_full_score_duration_ms);
  appendJsonSize(stream, "trajectory_optimizer_shadow_lower_bound_evaluations",
                 stats.trajectory_optimizer.shadow_lower_bound_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_shadow_lower_bound_unavailable",
                 stats.trajectory_optimizer.shadow_lower_bound_unavailable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_lower_bound_prunable",
                 stats.trajectory_optimizer.shadow_lower_bound_prunable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_lower_bound_false_prunes",
                 stats.trajectory_optimizer.shadow_lower_bound_false_prunes);
  appendJsonSize(stream, "trajectory_optimizer_shadow_lower_bound_winner_prunes",
                 stats.trajectory_optimizer.shadow_lower_bound_winner_prunes);
  appendJsonNumber(
      stream, "trajectory_optimizer_shadow_lower_bound_prunable_full_score_duration_ms",
      stats.trajectory_optimizer.shadow_lower_bound_prunable_full_score_duration_ms);
  appendJsonNumber(
      stream, "trajectory_optimizer_shadow_lower_bound_max_overestimate_score",
      stats.trajectory_optimizer.shadow_lower_bound_max_overestimate_score);
  appendJsonNumber(
      stream, "trajectory_optimizer_shadow_lower_bound_max_underestimate_score",
      stats.trajectory_optimizer.shadow_lower_bound_max_underestimate_score);
  appendJsonNumber(
      stream,
      "trajectory_optimizer_shadow_lower_bound_max_false_prune_improvement_score",
      stats.trajectory_optimizer.shadow_lower_bound_max_false_prune_improvement_score);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_evaluations",
                 stats.trajectory_optimizer.shadow_segment_score_evaluations);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_unavailable",
                 stats.trajectory_optimizer.shadow_segment_score_unavailable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_prunable",
                 stats.trajectory_optimizer.shadow_segment_score_prunable);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_false_prunes",
                 stats.trajectory_optimizer.shadow_segment_score_false_prunes);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_winner_mismatches",
                 stats.trajectory_optimizer.shadow_segment_score_winner_mismatches);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_segment_score_window_samples_total",
                 stats.trajectory_optimizer.shadow_segment_score_window_samples_total);
  appendJsonSize(stream, "trajectory_optimizer_shadow_segment_score_window_samples_max",
                 stats.trajectory_optimizer.shadow_segment_score_window_samples_max);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_abs_error_sum",
                   stats.trajectory_optimizer.shadow_segment_score_abs_error_sum);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_abs_error_p95",
                   stats.trajectory_optimizer.shadow_segment_score_abs_error_p95);
  appendJsonNumber(stream, "trajectory_optimizer_shadow_segment_score_max_overestimate",
                   stats.trajectory_optimizer.shadow_segment_score_max_overestimate);
  appendJsonNumber(stream,
                   "trajectory_optimizer_shadow_segment_score_max_underestimate",
                   stats.trajectory_optimizer.shadow_segment_score_max_underestimate);
  appendJsonNumber(
      stream,
      "trajectory_optimizer_shadow_segment_score_max_false_prune_improvement_score",
      stats.trajectory_optimizer
          .shadow_segment_score_max_false_prune_improvement_score);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_boundary_clamped_local_candidates",
                 stats.trajectory_optimizer.shadow_boundary_clamped_local_candidates);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_boundary_clamped_window_samples_total",
      stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_total);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_boundary_clamped_window_samples_max",
                 stats.trajectory_optimizer.shadow_boundary_clamped_window_samples_max);
  appendJsonSize(stream, "trajectory_optimizer_window_count",
                 stats.trajectory_optimizer.window_count);
  appendJsonSize(stream, "trajectory_optimizer_active_window_count",
                 stats.trajectory_optimizer.active_window_count);
  appendJsonSize(stream, "trajectory_optimizer_active_window_samples",
                 stats.trajectory_optimizer.active_window_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_centerline_blocked",
                 stats.trajectory_optimizer.active_window_centerline_blocked);
  appendJsonSize(stream, "trajectory_optimizer_active_window_heading_change_samples",
                 stats.trajectory_optimizer.active_window_heading_change_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_heading_span_samples",
                 stats.trajectory_optimizer.active_window_heading_span_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_curvature_samples",
                 stats.trajectory_optimizer.active_window_curvature_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_width_change_samples",
                 stats.trajectory_optimizer.active_window_width_change_samples);
  appendJsonSize(stream, "trajectory_optimizer_active_window_width_asymmetry_samples",
                 stats.trajectory_optimizer.active_window_width_asymmetry_samples);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_asymmetry_count",
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples",
      stats.trajectory_optimizer.shadow_active_window_no_width_asymmetry_samples);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_triggers_count",
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_width_triggers_samples",
      stats.trajectory_optimizer.shadow_active_window_no_width_triggers_samples);
  appendJsonSize(stream,
                 "trajectory_optimizer_shadow_active_window_no_heading_span_count",
                 stats.trajectory_optimizer.shadow_active_window_no_heading_span_count);
  appendJsonSize(
      stream, "trajectory_optimizer_shadow_active_window_no_heading_span_samples",
      stats.trajectory_optimizer.shadow_active_window_no_heading_span_samples);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_windows",
                 stats.trajectory_optimizer.centerline_blocked_windows);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_window_samples",
                 stats.trajectory_optimizer.centerline_blocked_window_samples);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_window_merged_count",
                 stats.trajectory_optimizer.centerline_blocked_window_merged_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_prohibited_cells",
                 stats.trajectory_optimizer.centerline_blocked_prohibited_cells);
  appendJsonSize(stream,
                 "trajectory_optimizer_centerline_blocked_outside_grid_segments",
                 stats.trajectory_optimizer.centerline_blocked_outside_grid_segments);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_segment_count",
                 stats.trajectory_optimizer.centerline_blocked_segment_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_span_count",
                 stats.trajectory_optimizer.centerline_blocked_span_count);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_first_segment_index",
                 stats.trajectory_optimizer.centerline_blocked_first_segment_index);
  appendJsonSize(stream, "trajectory_optimizer_centerline_blocked_last_segment_index",
                 stats.trajectory_optimizer.centerline_blocked_last_segment_index);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_s_m",
                   stats.trajectory_optimizer.centerline_blocked_first_s_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_s_m",
                   stats.trajectory_optimizer.centerline_blocked_last_s_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_span_length_m",
                   stats.trajectory_optimizer.centerline_blocked_span_length_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_x_m",
                   stats.trajectory_optimizer.centerline_blocked_first_x_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_first_y_m",
                   stats.trajectory_optimizer.centerline_blocked_first_y_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_x_m",
                   stats.trajectory_optimizer.centerline_blocked_last_x_m);
  appendJsonNumber(stream, "trajectory_optimizer_centerline_blocked_last_y_m",
                   stats.trajectory_optimizer.centerline_blocked_last_y_m);
  appendJsonBool(stream, "trajectory_optimizer_centerline_blocked_first_outside_grid",
                 stats.trajectory_optimizer.centerline_blocked_first_outside_grid);
  appendJsonBool(stream, "trajectory_optimizer_centerline_blocked_last_outside_grid",
                 stats.trajectory_optimizer.centerline_blocked_last_outside_grid);
  appendJsonSize(stream,
                 "trajectory_optimizer_centerline_blocked_span_diagnostic_count",
                 stats.trajectory_optimizer.centerline_blocked_span_diagnostic_count);
  for (std::size_t i = 0U; i < kMaxCenterlineBlockedSpanDiagnostics; ++i) {
    const std::string prefix =
        "trajectory_optimizer_centerline_blocked_span" + std::to_string(i);
    const TrajectoryOptimizerBlockedSpanDiagnostic& span =
        stats.trajectory_optimizer.centerline_blocked_span_diagnostics.at(i);
    appendJsonSize(stream, prefix + "_begin_segment_index", span.begin_segment_index);
    appendJsonSize(stream, prefix + "_end_segment_index", span.end_segment_index);
    appendJsonNumber(stream, prefix + "_begin_s_m", span.begin_s_m);
    appendJsonNumber(stream, prefix + "_end_s_m", span.end_s_m);
    appendJsonNumber(stream, prefix + "_length_m", span.length_m);
    appendJsonNumber(stream, prefix + "_begin_x_m", span.begin_x_m);
    appendJsonNumber(stream, prefix + "_begin_y_m", span.begin_y_m);
    appendJsonNumber(stream, prefix + "_end_x_m", span.end_x_m);
    appendJsonNumber(stream, prefix + "_end_y_m", span.end_y_m);
    appendJsonSize(stream, prefix + "_prohibited_cells", span.prohibited_cells);
    appendJsonSize(stream, prefix + "_outside_grid_segments",
                   span.outside_grid_segments);
  }
  appendJsonSize(stream, "trajectory_optimizer_dp_states",
                 stats.trajectory_optimizer.dp_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_transitions",
                 stats.trajectory_optimizer.dp_transitions);
  appendJsonSize(stream, "trajectory_optimizer_dp_segment_cache_hits",
                 stats.trajectory_optimizer.dp_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_dp_segment_cache_misses",
                 stats.trajectory_optimizer.dp_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_candidate_segment_cache_hits",
                 stats.trajectory_optimizer.candidate_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_candidate_segment_cache_misses",
                 stats.trajectory_optimizer.candidate_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_full_path_segment_cache_hits",
                 stats.trajectory_optimizer.full_path_segment_cache_hits);
  appendJsonSize(stream, "trajectory_optimizer_full_path_segment_cache_misses",
                 stats.trajectory_optimizer.full_path_segment_cache_misses);
  appendJsonSize(stream, "trajectory_optimizer_dp_coarse_states",
                 stats.trajectory_optimizer.dp_coarse_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_coarse_transitions",
                 stats.trajectory_optimizer.dp_coarse_transitions);
  appendJsonSize(stream, "trajectory_optimizer_dp_fine_states",
                 stats.trajectory_optimizer.dp_fine_states);
  appendJsonSize(stream, "trajectory_optimizer_dp_fine_transitions",
                 stats.trajectory_optimizer.dp_fine_transitions);
  appendJsonBool(stream, "trajectory_optimizer_dp_coarse_to_fine_used",
                 stats.trajectory_optimizer.dp_coarse_to_fine_used);
  appendJsonNumber(stream, "trajectory_optimizer_window_detection_duration_ms",
                   stats.trajectory_optimizer.window_detection_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_window_eval_duration_ms",
                   stats.trajectory_optimizer.window_eval_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_dp_duration_ms",
                   stats.trajectory_optimizer.dp_duration_ms);
  appendJsonNumber(stream, "trajectory_optimizer_full_final_score_duration_ms",
                   stats.trajectory_optimizer.full_final_score_duration_ms);
  appendJsonBool(stream, "trajectory_optimizer_async_refined",
                 stats.trajectory_optimizer.async_refined);
  return stream.str();
}

std::string turnSmoothingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"turn_smoothing_input_samples\":" << stats.turn_smoothing.input_samples;
  appendJsonSize(stream, "turn_smoothing_output_samples",
                 stats.turn_smoothing.output_samples);
  appendJsonSize(stream, "turn_smoothing_detected_corners",
                 stats.turn_smoothing.detected_corners);
  appendJsonSize(stream, "turn_smoothing_attempted_corners",
                 stats.turn_smoothing.attempted_corners);
  appendJsonSize(stream, "turn_smoothing_candidate_attempts",
                 stats.turn_smoothing.candidate_attempts);
  appendJsonSize(stream, "turn_smoothing_relaxed_candidate_attempts",
                 stats.turn_smoothing.relaxed_candidate_attempts);
  appendJsonSize(stream, "turn_smoothing_bezier_cache_hits",
                 stats.turn_smoothing.bezier_cache_hits);
  appendJsonSize(stream, "turn_smoothing_bezier_cache_misses",
                 stats.turn_smoothing.bezier_cache_misses);
  appendJsonSize(stream, "turn_smoothing_before_metrics_cache_hits",
                 stats.turn_smoothing.before_metrics_cache_hits);
  appendJsonSize(stream, "turn_smoothing_before_metrics_cache_misses",
                 stats.turn_smoothing.before_metrics_cache_misses);
  appendJsonSize(stream, "turn_smoothing_traversability_cache_hits",
                 stats.turn_smoothing.traversability_cache_hits);
  appendJsonSize(stream, "turn_smoothing_traversability_cache_misses",
                 stats.turn_smoothing.traversability_cache_misses);
  appendJsonSize(stream, "turn_smoothing_smoothed_corners",
                 stats.turn_smoothing.smoothed_corners);
  appendJsonSize(stream, "turn_smoothing_rejected_prohibited",
                 stats.turn_smoothing.rejected_prohibited);
  appendJsonSize(stream, "turn_smoothing_rejected_corridor",
                 stats.turn_smoothing.rejected_corridor);
  appendJsonSize(stream, "turn_smoothing_rejected_not_improved",
                 stats.turn_smoothing.rejected_not_improved);
  appendJsonSize(stream, "turn_smoothing_rejected_curvature_regression",
                 stats.turn_smoothing.rejected_curvature_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_radius_regression",
                 stats.turn_smoothing.rejected_radius_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_speed_regression",
                 stats.turn_smoothing.rejected_speed_regression);
  appendJsonNumber(stream, "turn_smoothing_heading_delta_before_rad",
                   stats.turn_smoothing.max_heading_delta_before_rad);
  appendJsonNumber(stream, "turn_smoothing_heading_delta_after_rad",
                   stats.turn_smoothing.max_heading_delta_after_rad);
  appendJsonNumber(stream, "turn_smoothing_curvature_jump_before_1pm",
                   stats.turn_smoothing.max_curvature_jump_before_1pm);
  appendJsonNumber(stream, "turn_smoothing_curvature_jump_after_1pm",
                   stats.turn_smoothing.max_curvature_jump_after_1pm);
  appendJsonNumber(stream, "turn_smoothing_min_inner_margin_m",
                   stats.turn_smoothing.min_inner_margin_m);
  appendJsonNumber(stream, "turn_smoothing_max_outer_shift_m",
                   stats.turn_smoothing.max_applied_outer_shift_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_entry_distance_m",
                   stats.turn_smoothing.accepted_entry_distance_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_exit_distance_m",
                   stats.turn_smoothing.accepted_exit_distance_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_shift_scale",
                   stats.turn_smoothing.accepted_shift_scale);
  appendJsonNumber(stream, "turn_smoothing_accepted_relaxed_angle_deg",
                   stats.turn_smoothing.accepted_relaxed_angle_deg);
  appendJsonNumber(stream, "turn_smoothing_accepted_score",
                   stats.turn_smoothing.accepted_score);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_radius_before_m",
                   stats.turn_smoothing.accepted_min_radius_before_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_radius_after_m",
                   stats.turn_smoothing.accepted_min_radius_after_m);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_speed_before_mps",
                   stats.turn_smoothing.accepted_min_speed_before_mps);
  appendJsonNumber(stream, "turn_smoothing_accepted_min_speed_after_mps",
                   stats.turn_smoothing.accepted_min_speed_after_mps);
  appendJsonNumber(stream, "turn_smoothing_accepted_local_time_before_s",
                   stats.turn_smoothing.accepted_local_time_before_s);
  appendJsonNumber(stream, "turn_smoothing_accepted_local_time_after_s",
                   stats.turn_smoothing.accepted_local_time_after_s);
  appendJsonNumber(stream, "turn_smoothing_candidate_build_duration_ms",
                   stats.turn_smoothing.candidate_build_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_candidate_replace_duration_ms",
                   stats.turn_smoothing.candidate_replace_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_collision_check_duration_ms",
                   stats.turn_smoothing.collision_check_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_metrics_duration_ms",
                   stats.turn_smoothing.metrics_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_shape_diagnostics_duration_ms",
                   stats.turn_smoothing.shape_diagnostics_duration_ms);
  appendJsonNumber(stream, "turn_smoothing_speed_profile_duration_ms",
                   stats.turn_smoothing.speed_profile_duration_ms);
  stream << ",\"turn_smoothing_corner_diagnostics\":[";
  for (std::size_t i = 0U; i < stats.turn_smoothing.corner_diagnostics.size(); ++i) {
    const TurnSmoothingCornerDiagnostic& diagnostic =
        stats.turn_smoothing.corner_diagnostics[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{\"accepted\":" << (diagnostic.accepted ? "true" : "false");
    appendJsonString(stream, "reject_reason", diagnostic.reject_reason);
    appendJsonNumber(stream, "corner_s_m", diagnostic.corner_s_m);
    appendJsonNumber(stream, "entry_distance_m", diagnostic.entry_distance_m);
    appendJsonNumber(stream, "exit_distance_m", diagnostic.exit_distance_m);
    appendJsonNumber(stream, "shift_scale", diagnostic.shift_scale);
    appendJsonNumber(stream, "relaxed_angle_deg", diagnostic.relaxed_angle_deg);
    appendJsonNumber(stream, "score", diagnostic.score);
    appendJsonNumber(stream, "min_radius_before_m", diagnostic.min_radius_before_m);
    appendJsonNumber(stream, "min_radius_after_m", diagnostic.min_radius_after_m);
    appendJsonNumber(stream, "min_speed_before_mps", diagnostic.min_speed_before_mps);
    appendJsonNumber(stream, "min_speed_after_mps", diagnostic.min_speed_after_mps);
    appendJsonNumber(stream, "local_time_before_s", diagnostic.local_time_before_s);
    appendJsonNumber(stream, "local_time_after_s", diagnostic.local_time_after_s);
    appendJsonNumber(stream, "curvature_jump_before_1pm",
                     diagnostic.curvature_jump_before_1pm);
    appendJsonNumber(stream, "curvature_jump_after_1pm",
                     diagnostic.curvature_jump_after_1pm);
    appendJsonNumber(stream, "heading_delta_before_rad",
                     diagnostic.heading_delta_before_rad);
    appendJsonNumber(stream, "heading_delta_after_rad",
                     diagnostic.heading_delta_after_rad);
    stream << "}";
  }
  stream << "]";
  return stream.str();
}

std::string trajectoryTimingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"trajectory_total_duration_ms\":";
  writeJsonNumberOrNull(stream, stats.total_duration_ms);
  appendJsonNumber(stream, "trajectory_corridor_duration_ms",
                   stats.corridor_duration_ms);
  appendJsonNumber(stream, "trajectory_trajectory_optimizer_duration_ms",
                   stats.trajectory_optimizer_duration_ms);
  appendJsonNumber(stream, "trajectory_turn_smoothing_duration_ms",
                   stats.turn_smoothing_duration_ms);
  appendJsonNumber(stream, "trajectory_speed_profile_duration_ms",
                   stats.speed_profile_duration_ms);
  return stream.str();
}

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
  appendJsonSize(stream, "speed_profile_curvature_limited_samples",
                 stats.speed_profile_curvature_limited_samples);
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

std::optional<TrajectoryPlannerDiagnosticsEnvelope>
parseTrajectoryPlannerDiagnosticsJson(const std::string& json) {
  TrajectoryPlannerDiagnosticsEnvelope envelope{};
  if (!parseJsonUint64(json, "planner_path_id", envelope.planner_path_id) ||
      !parseJsonUint64(json, "path_stamp_ns", envelope.path_stamp_ns)) {
    return std::nullopt;
  }

  if (const std::optional<std::string_view> status =
          jsonValueForKey(json, "trajectory_status");
      status.has_value()) {
    envelope.stats.status = parseTrajectoryPlannerStatusName(*status);
  }
  if (const std::optional<std::string_view> quality =
          jsonValueForKey(json, "trajectory_quality");
      quality.has_value()) {
    envelope.stats.quality = parseTrajectoryQualityName(*quality);
  }

  parseJsonSize(json, "trajectory_input_points", envelope.stats.input_points);
  parseJsonSize(json, "trajectory_compact_segments", envelope.stats.compact_segments);
  parseJsonSize(json, "trajectory_line_segments", envelope.stats.line_segments);
  parseJsonSize(json, "trajectory_arc_segments", envelope.stats.arc_segments);
  parseJsonSize(json, "trajectory_samples", envelope.stats.samples);
  parseJsonDouble(json, "trajectory_length_m", envelope.stats.length_m);
  parseJsonDouble(json, "curvature_min_1pm", envelope.stats.curvature_min_1pm);
  parseJsonDouble(json, "curvature_max_1pm", envelope.stats.curvature_max_1pm);
  parseJsonDouble(json, "curvature_mean_abs_1pm",
                  envelope.stats.curvature_mean_abs_1pm);
  parseJsonDouble(json, "speed_profile_min_mps", envelope.stats.speed_profile_min_mps);
  parseJsonDouble(json, "speed_profile_max_mps", envelope.stats.speed_profile_max_mps);
  parseJsonDouble(json, "speed_profile_mean_mps",
                  envelope.stats.speed_profile_mean_mps);
  parseJsonSize(json, "speed_profile_curvature_limited_samples",
                envelope.stats.speed_profile_curvature_limited_samples);
  std::size_t top_constraint_count = 0U;
  parseJsonSize(json, "speed_profile_top_constraint_count", top_constraint_count);
  top_constraint_count = std::min<std::size_t>(top_constraint_count, 5U);
  envelope.stats.top_speed_constraints.clear();
  envelope.stats.top_speed_constraints.reserve(top_constraint_count);
  for (std::size_t i = 0U; i < top_constraint_count; ++i) {
    const std::string prefix = speedProfileTopConstraintPrefix(i);
    SpeedProfileConstraintDiagnostic diagnostic{};
    parseJsonSize(json, prefix + "sample_index", diagnostic.sample_index);
    parseJsonDouble(json, prefix + "s_m", diagnostic.s_m);
    parseJsonDouble(json, prefix + "radius_m", diagnostic.radius_m);
    parseJsonDouble(json, prefix + "curvature_1pm", diagnostic.curvature_1pm);
    parseJsonDouble(json, prefix + "speed_limit_mps", diagnostic.speed_limit_mps);
    parseJsonDouble(json, prefix + "profiled_limit_mps", diagnostic.profiled_limit_mps);
    if (const std::optional<std::string_view> source =
            jsonValueForKey(json, prefix + "source");
        source.has_value()) {
      diagnostic.source = parseSpeedConstraintTypeName(*source);
    }
    parseJsonBool(json, prefix + "isolated_curvature_spike",
                  diagnostic.isolated_curvature_spike);
    envelope.stats.top_speed_constraints.push_back(diagnostic);
  }
  parseJsonSize(json, "isolated_curvature_spike_candidates",
                envelope.stats.isolated_curvature_spike_candidates);
  parseJsonSize(json, "isolated_curvature_spikes_smoothed_geometry",
                envelope.stats.isolated_curvature_spikes_smoothed_geometry);
  parseJsonSize(json, "isolated_curvature_spikes_smoothed_speed_profile",
                envelope.stats.isolated_curvature_spikes_smoothed_speed_profile);
  parseJsonDouble(json, "isolated_curvature_spike_max_before_1pm",
                  envelope.stats.isolated_curvature_spike_max_before_1pm);
  parseJsonDouble(json, "isolated_curvature_spike_max_after_1pm",
                  envelope.stats.isolated_curvature_spike_max_after_1pm);
  parseJsonDouble(json, "trajectory_total_duration_ms",
                  envelope.stats.total_duration_ms);
  parseJsonDouble(json, "trajectory_corridor_duration_ms",
                  envelope.stats.corridor_duration_ms);
  parseJsonDouble(json, "trajectory_trajectory_optimizer_duration_ms",
                  envelope.stats.trajectory_optimizer_duration_ms);
  parseJsonDouble(json, "trajectory_turn_smoothing_duration_ms",
                  envelope.stats.turn_smoothing_duration_ms);
  parseJsonDouble(json, "trajectory_speed_profile_duration_ms",
                  envelope.stats.speed_profile_duration_ms);

  CorridorStats& corridor = envelope.stats.corridor;
  parseJsonSize(json, "corridor_input_points", corridor.input_points);
  parseJsonSize(json, "corridor_samples", corridor.samples);
  parseJsonSize(json, "corridor_route_prohibited_samples",
                corridor.route_prohibited_samples);
  parseJsonSize(json, "corridor_center_recovered_samples",
                corridor.center_recovered_samples);
  parseJsonSize(json, "corridor_center_unrecoverable_samples",
                corridor.center_unrecoverable_samples);
  parseJsonSize(json, "corridor_outside_grid_samples", corridor.outside_grid_samples);
  parseJsonSize(json, "corridor_lateral_limited_samples",
                corridor.lateral_limited_samples);
  parseJsonDouble(json, "corridor_width_min_m", corridor.min_width_m);
  parseJsonDouble(json, "corridor_width_mean_m", corridor.mean_width_m);
  parseJsonDouble(json, "corridor_width_max_m", corridor.max_width_m);
  parseJsonDouble(json, "corridor_clearance_min_m", corridor.min_clearance_m);
  parseJsonDouble(json, "corridor_clearance_mean_m", corridor.mean_clearance_m);
  parseJsonDouble(json, "corridor_clearance_max_m", corridor.max_clearance_m);
  parseJsonDouble(json, "corridor_center_recovery_max_m",
                  corridor.max_center_recovery_m);
  parseJsonDouble(json, "corridor_lateral_reduction_max_m",
                  corridor.max_lateral_bound_reduction_m);
  parseJsonSize(json, "corridor_parallel_workers_used", corridor.parallel_workers_used);
  parseJsonBool(json, "corridor_samples_reused", corridor.samples_reused);
  parseJsonSize(json, "corridor_reused_samples", corridor.reused_samples);
  (void)parseJsonUint64(json, "corridor_route_fingerprint", corridor.route_fingerprint);
  (void)parseJsonUint64(json, "corridor_config_fingerprint",
                        corridor.config_fingerprint);
  (void)parseJsonUint64(json, "corridor_grid_cells_hash",
                        corridor.prohibited_grid_fingerprint.cells_hash);
  (void)parseJsonUint64(json, "corridor_grid_inflated_hash",
                        corridor.prohibited_grid_fingerprint.inflated_hash);
  parseJsonDouble(json, "corridor_sample_build_duration_ms",
                  corridor.sample_build_duration_ms);
  parseJsonDouble(json, "corridor_raycast_duration_ms", corridor.raycast_duration_ms);
  parseJsonDouble(json, "corridor_lateral_limit_duration_ms",
                  corridor.lateral_limit_duration_ms);
  parseJsonDouble(json, "corridor_clearance_field_build_ms",
                  corridor.clearance_field_build_duration_ms);
  parseJsonBool(json, "clearance_field_reused_by_corridor",
                corridor.clearance_field_reused);
  parseJsonBool(json, "corridor_clearance_field_cache_hit",
                corridor.clearance_field_cache_hit);

  TrajectoryOptimizerStats& optimizer = envelope.stats.trajectory_optimizer;
  parseJsonSize(json, "trajectory_optimizer_input_samples", optimizer.input_samples);
  parseJsonSize(json, "trajectory_optimizer_optimizer_samples",
                optimizer.optimizer_samples);
  parseJsonSize(json, "trajectory_optimizer_output_samples", optimizer.output_samples);
  parseJsonSize(json, "trajectory_optimizer_iterations", optimizer.iterations);
  parseJsonSize(json, "trajectory_optimizer_candidate_evaluations",
                optimizer.candidate_evaluations);
  parseJsonSize(json, "trajectory_optimizer_collision_rejections",
                optimizer.collision_rejections);
  parseJsonDouble(json, "trajectory_optimizer_cost_initial", optimizer.initial_cost);
  parseJsonDouble(json, "trajectory_optimizer_cost_final", optimizer.final_cost);
  parseJsonDouble(json, "trajectory_optimizer_centerline_length_m",
                  optimizer.centerline_length_m);
  parseJsonDouble(json, "trajectory_optimizer_final_length_m",
                  optimizer.final_length_m);
  parseJsonDouble(json, "trajectory_optimizer_final_length_ratio",
                  optimizer.final_length_ratio);
  parseJsonDouble(json, "trajectory_optimizer_cost_curvature",
                  optimizer.cost_curvature);
  parseJsonDouble(json, "trajectory_optimizer_cost_curvature_change",
                  optimizer.cost_curvature_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_radius_shortfall",
                  optimizer.cost_radius_shortfall);
  parseJsonDouble(json, "trajectory_optimizer_cost_heading_jump",
                  optimizer.cost_heading_jump);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_change",
                  optimizer.cost_offset_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_second_change",
                  optimizer.cost_offset_second_change);
  parseJsonDouble(json, "trajectory_optimizer_cost_offset_slope",
                  optimizer.cost_offset_slope);
  parseJsonDouble(json, "trajectory_optimizer_cost_collision",
                  optimizer.cost_collision);
  parseJsonDouble(json, "trajectory_optimizer_cost_outside_grid",
                  optimizer.cost_outside_grid);
  parseJsonDouble(json, "trajectory_optimizer_final_estimated_time_s",
                  optimizer.estimated_time_s);
  parseJsonDouble(json, "trajectory_optimizer_final_min_speed_limit_mps",
                  optimizer.min_speed_limit_mps);
  parseJsonDouble(json, "trajectory_optimizer_final_max_speed_limit_mps",
                  optimizer.max_speed_limit_mps);
  parseJsonSize(json, "trajectory_optimizer_final_curvature_limited_samples",
                optimizer.curvature_limited_samples);
  parseJsonDouble(json, "trajectory_optimizer_best_candidate_score",
                  optimizer.best_candidate_score);
  parseJsonSize(json, "trajectory_optimizer_regularization_iterations",
                optimizer.regularization_iterations);
  parseJsonBool(json, "trajectory_optimizer_regularization_applied",
                optimizer.regularization_applied);
  parseJsonDouble(json,
                  "trajectory_optimizer_pre_regularization_max_curvature_jump_1pm",
                  optimizer.pre_regularization_max_curvature_jump_1pm);
  parseJsonDouble(json,
                  "trajectory_optimizer_post_regularization_max_curvature_jump_1pm",
                  optimizer.post_regularization_max_curvature_jump_1pm);
  parseJsonSize(json, "trajectory_optimizer_skipped_noop_candidates",
                optimizer.skipped_noop_candidates);
  parseJsonDouble(json, "trajectory_optimizer_candidate_path_evaluation_duration_ms",
                  optimizer.candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_score_duration_ms",
                  optimizer.candidate_score_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_point_build_duration_ms",
                  optimizer.candidate_point_build_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_sample_build_duration_ms",
                  optimizer.candidate_sample_build_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_cost_breakdown_duration_ms",
                  optimizer.candidate_cost_breakdown_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_shape_diagnostics_duration_ms",
                  optimizer.candidate_shape_diagnostics_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_regularization_duration_ms",
                  optimizer.regularization_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_scratch_reused_candidates",
                optimizer.scratch_reused_candidates);
  parseJsonBool(json, "trajectory_optimizer_parallel_candidate_evaluation_used",
                optimizer.parallel_candidate_evaluation_used);
  parseJsonSize(json, "trajectory_optimizer_parallel_workers_used",
                optimizer.parallel_workers_used);
  parseJsonSize(json, "trajectory_optimizer_candidate_chunks",
                optimizer.candidate_chunks);
  parseJsonSize(json, "trajectory_optimizer_candidate_parallel_batches",
                optimizer.candidate_parallel_batches);
  parseJsonSize(json, "trajectory_optimizer_candidate_threads_launched",
                optimizer.candidate_threads_launched);
  parseJsonDouble(json, "trajectory_optimizer_candidate_batch_wall_duration_ms",
                  optimizer.candidate_batch_wall_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_batch_wait_duration_ms",
                  optimizer.candidate_batch_wait_duration_ms);
  parseJsonDouble(json,
                  "trajectory_optimizer_candidate_worker_buffer_prepare_duration_ms",
                  optimizer.candidate_worker_buffer_prepare_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_thread_launch_duration_ms",
                  optimizer.candidate_thread_launch_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_candidate_thread_join_wait_duration_ms",
                  optimizer.candidate_thread_join_wait_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_worker_scratch_reuses",
                optimizer.worker_scratch_reuses);
  parseJsonSize(json, "trajectory_optimizer_candidate_snapshot_allocations_avoided",
                optimizer.candidate_snapshot_allocations_avoided);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_samples_total",
                optimizer.candidate_offset_changed_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_samples_max",
                optimizer.candidate_offset_changed_samples_max);
  parseJsonSize(json,
                "trajectory_optimizer_candidate_offset_changed_span_samples_total",
                optimizer.candidate_offset_changed_span_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_offset_changed_span_samples_max",
                optimizer.candidate_offset_changed_span_samples_max);
  parseJsonSize(json, "trajectory_optimizer_candidate_local_speed_window_samples_total",
                optimizer.candidate_local_speed_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_candidate_local_speed_window_samples_max",
                optimizer.candidate_local_speed_window_samples_max);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_evaluations",
                optimizer.local_candidate_evaluations);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_full_score_fallbacks",
                optimizer.local_candidate_full_score_fallbacks);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_full_score_required",
                optimizer.local_candidate_full_score_required);
  parseJsonSize(
      json, "trajectory_optimizer_local_candidate_full_score_required_invalid_input",
      optimizer.local_candidate_full_score_required_invalid_input);
  parseJsonSize(json,
                "trajectory_optimizer_local_candidate_full_score_required_boundary",
                optimizer.local_candidate_full_score_required_boundary);
  parseJsonSize(json,
                "trajectory_optimizer_local_candidate_full_score_required_unsafe_base",
                optimizer.local_candidate_full_score_required_unsafe_base);
  parseJsonSize(
      json, "trajectory_optimizer_local_candidate_full_score_required_window_invalid",
      optimizer.local_candidate_full_score_required_window_invalid);
  parseJsonSize(json, "trajectory_optimizer_local_candidate_acceptance_full_scores",
                optimizer.local_candidate_acceptance_full_scores);
  parseJsonSize(json, "trajectory_optimizer_local_score_false_positives",
                optimizer.local_score_false_positives);
  parseJsonDouble(json, "trajectory_optimizer_local_candidate_point_build_duration_ms",
                  optimizer.local_candidate_point_build_duration_ms);
  parseJsonDouble(json,
                  "trajectory_optimizer_local_candidate_path_evaluation_duration_ms",
                  optimizer.local_candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_local_candidate_score_duration_ms",
                  optimizer.local_candidate_score_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_full_candidate_score_duration_ms",
                  optimizer.full_candidate_score_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_validation_full_scores",
                optimizer.shadow_lower_bound_validation_full_scores);
  parseJsonDouble(
      json, "trajectory_optimizer_shadow_lower_bound_validation_full_score_duration_ms",
      optimizer.shadow_lower_bound_validation_full_score_duration_ms);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_evaluations",
                optimizer.shadow_lower_bound_evaluations);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_unavailable",
                optimizer.shadow_lower_bound_unavailable);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_prunable",
                optimizer.shadow_lower_bound_prunable);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_false_prunes",
                optimizer.shadow_lower_bound_false_prunes);
  parseJsonSize(json, "trajectory_optimizer_shadow_lower_bound_winner_prunes",
                optimizer.shadow_lower_bound_winner_prunes);
  parseJsonDouble(
      json, "trajectory_optimizer_shadow_lower_bound_prunable_full_score_duration_ms",
      optimizer.shadow_lower_bound_prunable_full_score_duration_ms);
  parseJsonDouble(json,
                  "trajectory_optimizer_shadow_lower_bound_max_overestimate_score",
                  optimizer.shadow_lower_bound_max_overestimate_score);
  parseJsonDouble(json,
                  "trajectory_optimizer_shadow_lower_bound_max_underestimate_score",
                  optimizer.shadow_lower_bound_max_underestimate_score);
  parseJsonDouble(
      json, "trajectory_optimizer_shadow_lower_bound_max_false_prune_improvement_score",
      optimizer.shadow_lower_bound_max_false_prune_improvement_score);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_evaluations",
                optimizer.shadow_segment_score_evaluations);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_unavailable",
                optimizer.shadow_segment_score_unavailable);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_prunable",
                optimizer.shadow_segment_score_prunable);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_false_prunes",
                optimizer.shadow_segment_score_false_prunes);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_winner_mismatches",
                optimizer.shadow_segment_score_winner_mismatches);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_window_samples_total",
                optimizer.shadow_segment_score_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_shadow_segment_score_window_samples_max",
                optimizer.shadow_segment_score_window_samples_max);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_abs_error_sum",
                  optimizer.shadow_segment_score_abs_error_sum);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_abs_error_p95",
                  optimizer.shadow_segment_score_abs_error_p95);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_max_overestimate",
                  optimizer.shadow_segment_score_max_overestimate);
  parseJsonDouble(json, "trajectory_optimizer_shadow_segment_score_max_underestimate",
                  optimizer.shadow_segment_score_max_underestimate);
  parseJsonDouble(
      json,
      "trajectory_optimizer_shadow_segment_score_max_false_prune_improvement_score",
      optimizer.shadow_segment_score_max_false_prune_improvement_score);
  parseJsonSize(json, "trajectory_optimizer_shadow_boundary_clamped_local_candidates",
                optimizer.shadow_boundary_clamped_local_candidates);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_boundary_clamped_window_samples_total",
                optimizer.shadow_boundary_clamped_window_samples_total);
  parseJsonSize(json, "trajectory_optimizer_shadow_boundary_clamped_window_samples_max",
                optimizer.shadow_boundary_clamped_window_samples_max);
  parseJsonSize(json, "trajectory_optimizer_window_count", optimizer.window_count);
  parseJsonSize(json, "trajectory_optimizer_active_window_count",
                optimizer.active_window_count);
  parseJsonSize(json, "trajectory_optimizer_active_window_samples",
                optimizer.active_window_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_centerline_blocked",
                optimizer.active_window_centerline_blocked);
  parseJsonSize(json, "trajectory_optimizer_active_window_heading_change_samples",
                optimizer.active_window_heading_change_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_heading_span_samples",
                optimizer.active_window_heading_span_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_curvature_samples",
                optimizer.active_window_curvature_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_width_change_samples",
                optimizer.active_window_width_change_samples);
  parseJsonSize(json, "trajectory_optimizer_active_window_width_asymmetry_samples",
                optimizer.active_window_width_asymmetry_samples);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_asymmetry_count",
                optimizer.shadow_active_window_no_width_asymmetry_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_asymmetry_samples",
                optimizer.shadow_active_window_no_width_asymmetry_samples);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_triggers_count",
                optimizer.shadow_active_window_no_width_triggers_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_width_triggers_samples",
                optimizer.shadow_active_window_no_width_triggers_samples);
  parseJsonSize(json, "trajectory_optimizer_shadow_active_window_no_heading_span_count",
                optimizer.shadow_active_window_no_heading_span_count);
  parseJsonSize(json,
                "trajectory_optimizer_shadow_active_window_no_heading_span_samples",
                optimizer.shadow_active_window_no_heading_span_samples);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_windows",
                optimizer.centerline_blocked_windows);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_window_samples",
                optimizer.centerline_blocked_window_samples);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_window_merged_count",
                optimizer.centerline_blocked_window_merged_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_prohibited_cells",
                optimizer.centerline_blocked_prohibited_cells);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_outside_grid_segments",
                optimizer.centerline_blocked_outside_grid_segments);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_segment_count",
                optimizer.centerline_blocked_segment_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_span_count",
                optimizer.centerline_blocked_span_count);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_first_segment_index",
                optimizer.centerline_blocked_first_segment_index);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_last_segment_index",
                optimizer.centerline_blocked_last_segment_index);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_s_m",
                  optimizer.centerline_blocked_first_s_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_s_m",
                  optimizer.centerline_blocked_last_s_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_span_length_m",
                  optimizer.centerline_blocked_span_length_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_x_m",
                  optimizer.centerline_blocked_first_x_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_first_y_m",
                  optimizer.centerline_blocked_first_y_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_x_m",
                  optimizer.centerline_blocked_last_x_m);
  parseJsonDouble(json, "trajectory_optimizer_centerline_blocked_last_y_m",
                  optimizer.centerline_blocked_last_y_m);
  parseJsonBool(json, "trajectory_optimizer_centerline_blocked_first_outside_grid",
                optimizer.centerline_blocked_first_outside_grid);
  parseJsonBool(json, "trajectory_optimizer_centerline_blocked_last_outside_grid",
                optimizer.centerline_blocked_last_outside_grid);
  parseJsonSize(json, "trajectory_optimizer_centerline_blocked_span_diagnostic_count",
                optimizer.centerline_blocked_span_diagnostic_count);
  for (std::size_t i = 0U; i < kMaxCenterlineBlockedSpanDiagnostics; ++i) {
    const std::string prefix =
        "trajectory_optimizer_centerline_blocked_span" + std::to_string(i);
    TrajectoryOptimizerBlockedSpanDiagnostic& span =
        optimizer.centerline_blocked_span_diagnostics.at(i);
    parseJsonSize(json, prefix + "_begin_segment_index", span.begin_segment_index);
    parseJsonSize(json, prefix + "_end_segment_index", span.end_segment_index);
    parseJsonDouble(json, prefix + "_begin_s_m", span.begin_s_m);
    parseJsonDouble(json, prefix + "_end_s_m", span.end_s_m);
    parseJsonDouble(json, prefix + "_length_m", span.length_m);
    parseJsonDouble(json, prefix + "_begin_x_m", span.begin_x_m);
    parseJsonDouble(json, prefix + "_begin_y_m", span.begin_y_m);
    parseJsonDouble(json, prefix + "_end_x_m", span.end_x_m);
    parseJsonDouble(json, prefix + "_end_y_m", span.end_y_m);
    parseJsonSize(json, prefix + "_prohibited_cells", span.prohibited_cells);
    parseJsonSize(json, prefix + "_outside_grid_segments", span.outside_grid_segments);
  }
  parseJsonSize(json, "trajectory_optimizer_dp_states", optimizer.dp_states);
  parseJsonSize(json, "trajectory_optimizer_dp_transitions", optimizer.dp_transitions);
  parseJsonSize(json, "trajectory_optimizer_dp_segment_cache_hits",
                optimizer.dp_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_dp_segment_cache_misses",
                optimizer.dp_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_candidate_segment_cache_hits",
                optimizer.candidate_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_candidate_segment_cache_misses",
                optimizer.candidate_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_full_path_segment_cache_hits",
                optimizer.full_path_segment_cache_hits);
  parseJsonSize(json, "trajectory_optimizer_full_path_segment_cache_misses",
                optimizer.full_path_segment_cache_misses);
  parseJsonSize(json, "trajectory_optimizer_dp_coarse_states",
                optimizer.dp_coarse_states);
  parseJsonSize(json, "trajectory_optimizer_dp_coarse_transitions",
                optimizer.dp_coarse_transitions);
  parseJsonSize(json, "trajectory_optimizer_dp_fine_states", optimizer.dp_fine_states);
  parseJsonSize(json, "trajectory_optimizer_dp_fine_transitions",
                optimizer.dp_fine_transitions);
  parseJsonBool(json, "trajectory_optimizer_dp_coarse_to_fine_used",
                optimizer.dp_coarse_to_fine_used);
  parseJsonDouble(json, "trajectory_optimizer_window_detection_duration_ms",
                  optimizer.window_detection_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_window_eval_duration_ms",
                  optimizer.window_eval_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_dp_duration_ms",
                  optimizer.dp_duration_ms);
  parseJsonDouble(json, "trajectory_optimizer_full_final_score_duration_ms",
                  optimizer.full_final_score_duration_ms);
  parseJsonBool(json, "trajectory_optimizer_async_refined", optimizer.async_refined);
  parseJsonDouble(json, "trajectory_optimizer_max_abs_offset_m",
                  optimizer.max_abs_offset_m);
  parseJsonDouble(json, "trajectory_optimizer_min_edge_margin_m",
                  optimizer.min_edge_margin_m);
  parseJsonDouble(json, "trajectory_optimizer_mean_edge_margin_m",
                  optimizer.mean_edge_margin_m);

  TurnSmoothingStats& turn_smoothing = envelope.stats.turn_smoothing;
  parseJsonSize(json, "turn_smoothing_input_samples", turn_smoothing.input_samples);
  parseJsonSize(json, "turn_smoothing_output_samples", turn_smoothing.output_samples);
  parseJsonSize(json, "turn_smoothing_detected_corners",
                turn_smoothing.detected_corners);
  parseJsonSize(json, "turn_smoothing_attempted_corners",
                turn_smoothing.attempted_corners);
  parseJsonSize(json, "turn_smoothing_candidate_attempts",
                turn_smoothing.candidate_attempts);
  parseJsonSize(json, "turn_smoothing_relaxed_candidate_attempts",
                turn_smoothing.relaxed_candidate_attempts);
  parseJsonSize(json, "turn_smoothing_bezier_cache_hits",
                turn_smoothing.bezier_cache_hits);
  parseJsonSize(json, "turn_smoothing_bezier_cache_misses",
                turn_smoothing.bezier_cache_misses);
  parseJsonSize(json, "turn_smoothing_before_metrics_cache_hits",
                turn_smoothing.before_metrics_cache_hits);
  parseJsonSize(json, "turn_smoothing_before_metrics_cache_misses",
                turn_smoothing.before_metrics_cache_misses);
  parseJsonSize(json, "turn_smoothing_traversability_cache_hits",
                turn_smoothing.traversability_cache_hits);
  parseJsonSize(json, "turn_smoothing_traversability_cache_misses",
                turn_smoothing.traversability_cache_misses);
  parseJsonSize(json, "turn_smoothing_smoothed_corners",
                turn_smoothing.smoothed_corners);
  parseJsonSize(json, "turn_smoothing_rejected_prohibited",
                turn_smoothing.rejected_prohibited);
  parseJsonSize(json, "turn_smoothing_rejected_corridor",
                turn_smoothing.rejected_corridor);
  parseJsonSize(json, "turn_smoothing_rejected_not_improved",
                turn_smoothing.rejected_not_improved);
  parseJsonSize(json, "turn_smoothing_rejected_curvature_regression",
                turn_smoothing.rejected_curvature_regression);
  parseJsonSize(json, "turn_smoothing_rejected_radius_regression",
                turn_smoothing.rejected_radius_regression);
  parseJsonSize(json, "turn_smoothing_rejected_speed_regression",
                turn_smoothing.rejected_speed_regression);
  parseJsonDouble(json, "turn_smoothing_heading_delta_before_rad",
                  turn_smoothing.max_heading_delta_before_rad);
  parseJsonDouble(json, "turn_smoothing_heading_delta_after_rad",
                  turn_smoothing.max_heading_delta_after_rad);
  parseJsonDouble(json, "turn_smoothing_curvature_jump_before_1pm",
                  turn_smoothing.max_curvature_jump_before_1pm);
  parseJsonDouble(json, "turn_smoothing_curvature_jump_after_1pm",
                  turn_smoothing.max_curvature_jump_after_1pm);
  parseJsonDouble(json, "turn_smoothing_min_inner_margin_m",
                  turn_smoothing.min_inner_margin_m);
  parseJsonDouble(json, "turn_smoothing_max_outer_shift_m",
                  turn_smoothing.max_applied_outer_shift_m);
  parseJsonDouble(json, "turn_smoothing_accepted_entry_distance_m",
                  turn_smoothing.accepted_entry_distance_m);
  parseJsonDouble(json, "turn_smoothing_accepted_exit_distance_m",
                  turn_smoothing.accepted_exit_distance_m);
  parseJsonDouble(json, "turn_smoothing_accepted_shift_scale",
                  turn_smoothing.accepted_shift_scale);
  parseJsonDouble(json, "turn_smoothing_accepted_relaxed_angle_deg",
                  turn_smoothing.accepted_relaxed_angle_deg);
  parseJsonDouble(json, "turn_smoothing_accepted_score", turn_smoothing.accepted_score);
  parseJsonDouble(json, "turn_smoothing_accepted_min_radius_before_m",
                  turn_smoothing.accepted_min_radius_before_m);
  parseJsonDouble(json, "turn_smoothing_accepted_min_radius_after_m",
                  turn_smoothing.accepted_min_radius_after_m);
  parseJsonDouble(json, "turn_smoothing_accepted_min_speed_before_mps",
                  turn_smoothing.accepted_min_speed_before_mps);
  parseJsonDouble(json, "turn_smoothing_accepted_min_speed_after_mps",
                  turn_smoothing.accepted_min_speed_after_mps);
  parseJsonDouble(json, "turn_smoothing_accepted_local_time_before_s",
                  turn_smoothing.accepted_local_time_before_s);
  parseJsonDouble(json, "turn_smoothing_accepted_local_time_after_s",
                  turn_smoothing.accepted_local_time_after_s);
  parseJsonDouble(json, "turn_smoothing_candidate_build_duration_ms",
                  turn_smoothing.candidate_build_duration_ms);
  parseJsonDouble(json, "turn_smoothing_candidate_replace_duration_ms",
                  turn_smoothing.candidate_replace_duration_ms);
  parseJsonDouble(json, "turn_smoothing_collision_check_duration_ms",
                  turn_smoothing.collision_check_duration_ms);
  parseJsonDouble(json, "turn_smoothing_metrics_duration_ms",
                  turn_smoothing.metrics_duration_ms);
  parseJsonDouble(json, "turn_smoothing_shape_diagnostics_duration_ms",
                  turn_smoothing.shape_diagnostics_duration_ms);
  parseJsonDouble(json, "turn_smoothing_speed_profile_duration_ms",
                  turn_smoothing.speed_profile_duration_ms);

  return envelope;
}

} // namespace drone_city_nav

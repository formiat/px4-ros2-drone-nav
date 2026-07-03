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
  if (value ==
      trajectoryPlannerStatusName(TrajectoryPlannerStatus::kRacingLineInvalid)) {
    return TrajectoryPlannerStatus::kRacingLineInvalid;
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
         "arc_radius_m,left_bound_m,right_bound_m,racing_offset_m,"
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
  writeCsvNumberOrEmpty(stream, sample.racing_offset_m);
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

std::string racingLineDiagnosticsJsonFields(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"racing_final_estimated_time_s\":";
  writeJsonNumberOrNull(stream, stats.racing_line.estimated_time_s);
  appendJsonNumber(stream, "racing_final_min_speed_limit_mps",
                   stats.racing_line.min_speed_limit_mps);
  appendJsonNumber(stream, "racing_final_max_speed_limit_mps",
                   stats.racing_line.max_speed_limit_mps);
  appendJsonSize(stream, "racing_final_curvature_limited_samples",
                 stats.racing_line.curvature_limited_samples);
  appendJsonNumber(stream, "racing_centerline_length_m",
                   stats.racing_line.centerline_length_m);
  appendJsonNumber(stream, "racing_final_length_m", stats.racing_line.final_length_m);
  appendJsonNumber(stream, "racing_final_length_ratio",
                   stats.racing_line.final_length_ratio);
  appendJsonNumber(stream, "racing_max_abs_offset_m",
                   stats.racing_line.max_abs_offset_m);
  appendJsonNumber(stream, "racing_min_edge_margin_m",
                   stats.racing_line.min_edge_margin_m);
  appendJsonNumber(stream, "racing_mean_edge_margin_m",
                   stats.racing_line.mean_edge_margin_m);
  appendJsonNumber(stream, "racing_cost_length", stats.racing_line.cost_length);
  appendJsonNumber(stream, "racing_cost_time", stats.racing_line.cost_time);
  appendJsonNumber(stream, "racing_cost_curvature", stats.racing_line.cost_curvature);
  appendJsonNumber(stream, "racing_cost_curvature_change",
                   stats.racing_line.cost_curvature_change);
  appendJsonNumber(stream, "racing_cost_heading_jump",
                   stats.racing_line.cost_heading_jump);
  appendJsonNumber(stream, "racing_cost_offset_change",
                   stats.racing_line.cost_offset_change);
  appendJsonNumber(stream, "racing_cost_offset_second_change",
                   stats.racing_line.cost_offset_second_change);
  appendJsonNumber(stream, "racing_cost_offset_slope",
                   stats.racing_line.cost_offset_slope);
  appendJsonNumber(stream, "racing_cost_collision", stats.racing_line.cost_collision);
  appendJsonNumber(stream, "racing_cost_outside_grid",
                   stats.racing_line.cost_outside_grid);
  appendJsonNumber(stream, "racing_cost_length_overrun",
                   stats.racing_line.cost_length_overrun);
  appendJsonNumber(stream, "racing_centerline_estimated_time_s",
                   stats.racing_line.centerline_estimated_time_s);
  appendJsonNumber(stream, "racing_centerline_min_speed_limit_mps",
                   stats.racing_line.centerline_min_speed_limit_mps);
  appendJsonNumber(stream, "racing_centerline_max_speed_limit_mps",
                   stats.racing_line.centerline_max_speed_limit_mps);
  appendJsonSize(stream, "racing_centerline_curvature_limited_samples",
                 stats.racing_line.centerline_curvature_limited_samples);
  appendJsonNumber(stream, "racing_best_candidate_estimated_time_s",
                   stats.racing_line.best_candidate_estimated_time_s);
  appendJsonNumber(stream, "racing_best_candidate_score",
                   stats.racing_line.best_candidate_score);
  appendJsonNumber(stream, "racing_best_candidate_min_speed_limit_mps",
                   stats.racing_line.best_candidate_min_speed_limit_mps);
  appendJsonNumber(stream, "racing_best_candidate_max_speed_limit_mps",
                   stats.racing_line.best_candidate_max_speed_limit_mps);
  appendJsonSize(stream, "racing_best_candidate_curvature_limited_samples",
                 stats.racing_line.best_candidate_curvature_limited_samples);
  appendJsonNumber(stream, "racing_time_gain_s", stats.racing_line.time_gain_s);
  appendJsonNumber(stream, "racing_regularization_time_delta_s",
                   stats.racing_line.regularization_time_delta_s);
  appendJsonSize(stream, "racing_regularization_iterations",
                 stats.racing_line.regularization_iterations);
  stream << ",\"racing_regularization_applied\":"
         << (stats.racing_line.regularization_applied ? "true" : "false");
  appendJsonNumber(stream, "racing_pre_regularization_max_curvature_jump_1pm",
                   stats.racing_line.pre_regularization_max_curvature_jump_1pm);
  appendJsonNumber(stream, "racing_post_regularization_max_curvature_jump_1pm",
                   stats.racing_line.post_regularization_max_curvature_jump_1pm);
  appendJsonSize(stream, "racing_skipped_noop_candidates",
                 stats.racing_line.skipped_noop_candidates);
  appendJsonNumber(stream, "racing_candidate_path_evaluation_duration_ms",
                   stats.racing_line.candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "racing_candidate_score_duration_ms",
                   stats.racing_line.candidate_score_duration_ms);
  appendJsonNumber(stream, "racing_candidate_point_build_duration_ms",
                   stats.racing_line.candidate_point_build_duration_ms);
  appendJsonNumber(stream, "racing_candidate_sample_build_duration_ms",
                   stats.racing_line.candidate_sample_build_duration_ms);
  appendJsonNumber(stream, "racing_candidate_cost_breakdown_duration_ms",
                   stats.racing_line.candidate_cost_breakdown_duration_ms);
  appendJsonNumber(stream, "racing_candidate_shape_diagnostics_duration_ms",
                   stats.racing_line.candidate_shape_diagnostics_duration_ms);
  appendJsonNumber(stream, "racing_candidate_speed_profile_duration_ms",
                   stats.racing_line.candidate_speed_profile_duration_ms);
  appendJsonSize(stream, "racing_candidate_speed_profile_calls",
                 stats.racing_line.candidate_speed_profile_calls);
  appendJsonSize(stream, "racing_candidate_speed_profile_samples_total",
                 stats.racing_line.candidate_speed_profile_samples_total);
  appendJsonSize(stream, "racing_candidate_speed_profile_samples_max",
                 stats.racing_line.candidate_speed_profile_samples_max);
  appendJsonNumber(stream, "racing_regularization_duration_ms",
                   stats.racing_line.regularization_duration_ms);
  appendJsonSize(stream, "racing_scratch_reused_candidates",
                 stats.racing_line.scratch_reused_candidates);
  appendJsonBool(stream, "racing_parallel_candidate_evaluation_used",
                 stats.racing_line.parallel_candidate_evaluation_used);
  appendJsonSize(stream, "racing_parallel_workers_used",
                 stats.racing_line.parallel_workers_used);
  appendJsonSize(stream, "racing_candidate_chunks", stats.racing_line.candidate_chunks);
  appendJsonSize(stream, "racing_candidate_parallel_batches",
                 stats.racing_line.candidate_parallel_batches);
  appendJsonSize(stream, "racing_candidate_threads_launched",
                 stats.racing_line.candidate_threads_launched);
  appendJsonNumber(stream, "racing_candidate_batch_wall_duration_ms",
                   stats.racing_line.candidate_batch_wall_duration_ms);
  appendJsonNumber(stream, "racing_candidate_batch_wait_duration_ms",
                   stats.racing_line.candidate_batch_wait_duration_ms);
  appendJsonNumber(stream, "racing_candidate_worker_buffer_prepare_duration_ms",
                   stats.racing_line.candidate_worker_buffer_prepare_duration_ms);
  appendJsonNumber(stream, "racing_candidate_thread_launch_duration_ms",
                   stats.racing_line.candidate_thread_launch_duration_ms);
  appendJsonNumber(stream, "racing_candidate_thread_join_wait_duration_ms",
                   stats.racing_line.candidate_thread_join_wait_duration_ms);
  appendJsonSize(stream, "racing_worker_scratch_reuses",
                 stats.racing_line.worker_scratch_reuses);
  appendJsonSize(stream, "racing_candidate_snapshot_allocations_avoided",
                 stats.racing_line.candidate_snapshot_allocations_avoided);
  appendJsonSize(stream, "racing_candidate_offset_changed_samples_total",
                 stats.racing_line.candidate_offset_changed_samples_total);
  appendJsonSize(stream, "racing_candidate_offset_changed_samples_max",
                 stats.racing_line.candidate_offset_changed_samples_max);
  appendJsonSize(stream, "racing_candidate_offset_changed_span_samples_total",
                 stats.racing_line.candidate_offset_changed_span_samples_total);
  appendJsonSize(stream, "racing_candidate_offset_changed_span_samples_max",
                 stats.racing_line.candidate_offset_changed_span_samples_max);
  appendJsonSize(stream, "racing_candidate_local_speed_window_samples_total",
                 stats.racing_line.candidate_local_speed_window_samples_total);
  appendJsonSize(stream, "racing_candidate_local_speed_window_samples_max",
                 stats.racing_line.candidate_local_speed_window_samples_max);
  appendJsonSize(stream, "racing_local_candidate_evaluations",
                 stats.racing_line.local_candidate_evaluations);
  appendJsonSize(stream, "racing_local_candidate_full_score_fallbacks",
                 stats.racing_line.local_candidate_full_score_fallbacks);
  appendJsonSize(stream, "racing_local_candidate_full_score_required",
                 stats.racing_line.local_candidate_full_score_required);
  appendJsonSize(stream, "racing_local_candidate_full_score_required_invalid_input",
                 stats.racing_line.local_candidate_full_score_required_invalid_input);
  appendJsonSize(stream, "racing_local_candidate_full_score_required_boundary",
                 stats.racing_line.local_candidate_full_score_required_boundary);
  appendJsonSize(stream, "racing_local_candidate_full_score_required_unsafe_base",
                 stats.racing_line.local_candidate_full_score_required_unsafe_base);
  appendJsonSize(stream, "racing_local_candidate_full_score_required_window_invalid",
                 stats.racing_line.local_candidate_full_score_required_window_invalid);
  appendJsonSize(stream, "racing_local_candidate_acceptance_full_scores",
                 stats.racing_line.local_candidate_acceptance_full_scores);
  appendJsonSize(stream, "racing_local_score_false_positives",
                 stats.racing_line.local_score_false_positives);
  appendJsonNumber(stream, "racing_local_candidate_point_build_duration_ms",
                   stats.racing_line.local_candidate_point_build_duration_ms);
  appendJsonNumber(stream, "racing_local_candidate_path_evaluation_duration_ms",
                   stats.racing_line.local_candidate_path_evaluation_duration_ms);
  appendJsonNumber(stream, "racing_local_candidate_score_duration_ms",
                   stats.racing_line.local_candidate_score_duration_ms);
  appendJsonNumber(stream, "racing_local_candidate_traversal_estimate_duration_ms",
                   stats.racing_line.local_candidate_traversal_estimate_duration_ms);
  appendJsonNumber(stream, "racing_full_candidate_score_duration_ms",
                   stats.racing_line.full_candidate_score_duration_ms);
  appendJsonSize(stream, "racing_shadow_lower_bound_validation_full_scores",
                 stats.racing_line.shadow_lower_bound_validation_full_scores);
  appendJsonNumber(
      stream, "racing_shadow_lower_bound_validation_full_score_duration_ms",
      stats.racing_line.shadow_lower_bound_validation_full_score_duration_ms);
  appendJsonSize(stream, "racing_shadow_lower_bound_evaluations",
                 stats.racing_line.shadow_lower_bound_evaluations);
  appendJsonSize(stream, "racing_shadow_lower_bound_unavailable",
                 stats.racing_line.shadow_lower_bound_unavailable);
  appendJsonSize(stream, "racing_shadow_lower_bound_prunable",
                 stats.racing_line.shadow_lower_bound_prunable);
  appendJsonSize(stream, "racing_shadow_lower_bound_false_prunes",
                 stats.racing_line.shadow_lower_bound_false_prunes);
  appendJsonSize(stream, "racing_shadow_lower_bound_winner_prunes",
                 stats.racing_line.shadow_lower_bound_winner_prunes);
  appendJsonNumber(
      stream, "racing_shadow_lower_bound_prunable_full_score_duration_ms",
      stats.racing_line.shadow_lower_bound_prunable_full_score_duration_ms);
  appendJsonNumber(stream, "racing_shadow_lower_bound_max_overestimate_score",
                   stats.racing_line.shadow_lower_bound_max_overestimate_score);
  appendJsonNumber(stream, "racing_shadow_lower_bound_max_underestimate_score",
                   stats.racing_line.shadow_lower_bound_max_underestimate_score);
  appendJsonNumber(
      stream, "racing_shadow_lower_bound_max_false_prune_improvement_score",
      stats.racing_line.shadow_lower_bound_max_false_prune_improvement_score);
  appendJsonSize(stream, "racing_shadow_local_speed_evaluations",
                 stats.racing_line.shadow_local_speed_evaluations);
  appendJsonSize(stream, "racing_shadow_local_speed_unavailable",
                 stats.racing_line.shadow_local_speed_unavailable);
  appendJsonSize(stream, "racing_shadow_local_speed_prunable",
                 stats.racing_line.shadow_local_speed_prunable);
  appendJsonSize(stream, "racing_shadow_local_speed_false_prunes",
                 stats.racing_line.shadow_local_speed_false_prunes);
  appendJsonSize(stream, "racing_shadow_local_speed_winner_mismatches",
                 stats.racing_line.shadow_local_speed_winner_mismatches);
  appendJsonNumber(stream, "racing_shadow_local_speed_abs_time_error_sum_s",
                   stats.racing_line.shadow_local_speed_abs_time_error_sum_s);
  appendJsonNumber(stream, "racing_shadow_local_speed_abs_time_error_p95_s",
                   stats.racing_line.shadow_local_speed_abs_time_error_p95_s);
  appendJsonNumber(stream, "racing_shadow_local_speed_max_time_overestimate_s",
                   stats.racing_line.shadow_local_speed_max_time_overestimate_s);
  appendJsonNumber(stream, "racing_shadow_local_speed_max_time_underestimate_s",
                   stats.racing_line.shadow_local_speed_max_time_underestimate_s);
  appendJsonNumber(stream, "racing_shadow_local_speed_abs_score_error_sum",
                   stats.racing_line.shadow_local_speed_abs_score_error_sum);
  appendJsonNumber(stream, "racing_shadow_local_speed_abs_score_error_p95",
                   stats.racing_line.shadow_local_speed_abs_score_error_p95);
  appendJsonNumber(stream, "racing_shadow_local_speed_max_score_overestimate",
                   stats.racing_line.shadow_local_speed_max_score_overestimate);
  appendJsonNumber(stream, "racing_shadow_local_speed_max_score_underestimate",
                   stats.racing_line.shadow_local_speed_max_score_underestimate);
  appendJsonNumber(
      stream, "racing_shadow_local_speed_max_false_prune_improvement_score",
      stats.racing_line.shadow_local_speed_max_false_prune_improvement_score);
  appendJsonSize(stream, "racing_shadow_segment_score_evaluations",
                 stats.racing_line.shadow_segment_score_evaluations);
  appendJsonSize(stream, "racing_shadow_segment_score_unavailable",
                 stats.racing_line.shadow_segment_score_unavailable);
  appendJsonSize(stream, "racing_shadow_segment_score_prunable",
                 stats.racing_line.shadow_segment_score_prunable);
  appendJsonSize(stream, "racing_shadow_segment_score_false_prunes",
                 stats.racing_line.shadow_segment_score_false_prunes);
  appendJsonSize(stream, "racing_shadow_segment_score_winner_mismatches",
                 stats.racing_line.shadow_segment_score_winner_mismatches);
  appendJsonSize(stream, "racing_shadow_segment_score_window_samples_total",
                 stats.racing_line.shadow_segment_score_window_samples_total);
  appendJsonSize(stream, "racing_shadow_segment_score_window_samples_max",
                 stats.racing_line.shadow_segment_score_window_samples_max);
  appendJsonNumber(stream, "racing_shadow_segment_score_abs_error_sum",
                   stats.racing_line.shadow_segment_score_abs_error_sum);
  appendJsonNumber(stream, "racing_shadow_segment_score_abs_error_p95",
                   stats.racing_line.shadow_segment_score_abs_error_p95);
  appendJsonNumber(stream, "racing_shadow_segment_score_max_overestimate",
                   stats.racing_line.shadow_segment_score_max_overestimate);
  appendJsonNumber(stream, "racing_shadow_segment_score_max_underestimate",
                   stats.racing_line.shadow_segment_score_max_underestimate);
  appendJsonNumber(
      stream, "racing_shadow_segment_score_max_false_prune_improvement_score",
      stats.racing_line.shadow_segment_score_max_false_prune_improvement_score);
  appendJsonSize(stream, "racing_line_window_count", stats.racing_line.window_count);
  appendJsonSize(stream, "racing_line_active_window_count",
                 stats.racing_line.active_window_count);
  appendJsonSize(stream, "racing_line_active_window_samples",
                 stats.racing_line.active_window_samples);
  appendJsonSize(stream, "racing_line_active_window_centerline_blocked",
                 stats.racing_line.active_window_centerline_blocked);
  appendJsonSize(stream, "racing_line_active_window_heading_change_samples",
                 stats.racing_line.active_window_heading_change_samples);
  appendJsonSize(stream, "racing_line_active_window_heading_span_samples",
                 stats.racing_line.active_window_heading_span_samples);
  appendJsonSize(stream, "racing_line_active_window_curvature_samples",
                 stats.racing_line.active_window_curvature_samples);
  appendJsonSize(stream, "racing_line_active_window_width_change_samples",
                 stats.racing_line.active_window_width_change_samples);
  appendJsonSize(stream, "racing_line_active_window_width_asymmetry_samples",
                 stats.racing_line.active_window_width_asymmetry_samples);
  appendJsonSize(stream, "racing_line_dp_states", stats.racing_line.dp_states);
  appendJsonSize(stream, "racing_line_dp_transitions",
                 stats.racing_line.dp_transitions);
  appendJsonSize(stream, "racing_line_dp_segment_cache_hits",
                 stats.racing_line.dp_segment_cache_hits);
  appendJsonSize(stream, "racing_line_dp_segment_cache_misses",
                 stats.racing_line.dp_segment_cache_misses);
  appendJsonSize(stream, "racing_line_candidate_segment_cache_hits",
                 stats.racing_line.candidate_segment_cache_hits);
  appendJsonSize(stream, "racing_line_candidate_segment_cache_misses",
                 stats.racing_line.candidate_segment_cache_misses);
  appendJsonSize(stream, "racing_line_full_path_segment_cache_hits",
                 stats.racing_line.full_path_segment_cache_hits);
  appendJsonSize(stream, "racing_line_full_path_segment_cache_misses",
                 stats.racing_line.full_path_segment_cache_misses);
  appendJsonSize(stream, "racing_line_dp_coarse_states",
                 stats.racing_line.dp_coarse_states);
  appendJsonSize(stream, "racing_line_dp_coarse_transitions",
                 stats.racing_line.dp_coarse_transitions);
  appendJsonSize(stream, "racing_line_dp_fine_states",
                 stats.racing_line.dp_fine_states);
  appendJsonSize(stream, "racing_line_dp_fine_transitions",
                 stats.racing_line.dp_fine_transitions);
  appendJsonBool(stream, "racing_line_dp_coarse_to_fine_used",
                 stats.racing_line.dp_coarse_to_fine_used);
  appendJsonNumber(stream, "racing_line_window_detection_duration_ms",
                   stats.racing_line.window_detection_duration_ms);
  appendJsonNumber(stream, "racing_line_window_eval_duration_ms",
                   stats.racing_line.window_eval_duration_ms);
  appendJsonNumber(stream, "racing_line_dp_duration_ms",
                   stats.racing_line.dp_duration_ms);
  appendJsonNumber(stream, "racing_line_full_final_score_duration_ms",
                   stats.racing_line.full_final_score_duration_ms);
  appendJsonBool(stream, "racing_line_async_refined", stats.racing_line.async_refined);
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
  appendJsonSize(stream, "turn_smoothing_rejected_length",
                 stats.turn_smoothing.rejected_length);
  appendJsonSize(stream, "turn_smoothing_rejected_not_improved",
                 stats.turn_smoothing.rejected_not_improved);
  appendJsonSize(stream, "turn_smoothing_rejected_curvature_regression",
                 stats.turn_smoothing.rejected_curvature_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_radius_regression",
                 stats.turn_smoothing.rejected_radius_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_speed_regression",
                 stats.turn_smoothing.rejected_speed_regression);
  appendJsonSize(stream, "turn_smoothing_rejected_time_regression",
                 stats.turn_smoothing.rejected_time_regression);
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
  appendJsonNumber(stream, "trajectory_racing_line_duration_ms",
                   stats.racing_line_duration_ms);
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
  stream << "," << racingLineDiagnosticsJsonFields(stats);
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
  appendJsonSize(stream, "racing_line_input_samples", stats.racing_line.input_samples);
  appendJsonSize(stream, "racing_line_optimizer_samples",
                 stats.racing_line.optimizer_samples);
  appendJsonSize(stream, "racing_line_output_samples",
                 stats.racing_line.output_samples);
  appendJsonSize(stream, "racing_line_iterations", stats.racing_line.iterations);
  appendJsonSize(stream, "racing_line_candidate_evaluations",
                 stats.racing_line.candidate_evaluations);
  appendJsonSize(stream, "racing_line_collision_rejections",
                 stats.racing_line.collision_rejections);
  appendJsonNumber(stream, "racing_line_cost_initial", stats.racing_line.initial_cost);
  appendJsonNumber(stream, "racing_line_cost_final", stats.racing_line.final_cost);
  stream << "," << racingLineDiagnosticsJsonFields(stats);
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
  parseJsonDouble(json, "trajectory_racing_line_duration_ms",
                  envelope.stats.racing_line_duration_ms);
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

  RacingLineStats& racing = envelope.stats.racing_line;
  parseJsonSize(json, "racing_line_input_samples", racing.input_samples);
  parseJsonSize(json, "racing_line_optimizer_samples", racing.optimizer_samples);
  parseJsonSize(json, "racing_line_output_samples", racing.output_samples);
  parseJsonSize(json, "racing_line_iterations", racing.iterations);
  parseJsonSize(json, "racing_line_candidate_evaluations",
                racing.candidate_evaluations);
  parseJsonSize(json, "racing_line_collision_rejections", racing.collision_rejections);
  parseJsonDouble(json, "racing_line_cost_initial", racing.initial_cost);
  parseJsonDouble(json, "racing_line_cost_final", racing.final_cost);
  parseJsonDouble(json, "racing_centerline_length_m", racing.centerline_length_m);
  parseJsonDouble(json, "racing_final_length_m", racing.final_length_m);
  parseJsonDouble(json, "racing_final_length_ratio", racing.final_length_ratio);
  parseJsonDouble(json, "racing_cost_length", racing.cost_length);
  parseJsonDouble(json, "racing_cost_time", racing.cost_time);
  parseJsonDouble(json, "racing_cost_curvature", racing.cost_curvature);
  parseJsonDouble(json, "racing_cost_curvature_change", racing.cost_curvature_change);
  parseJsonDouble(json, "racing_cost_heading_jump", racing.cost_heading_jump);
  parseJsonDouble(json, "racing_cost_offset_change", racing.cost_offset_change);
  parseJsonDouble(json, "racing_cost_offset_second_change",
                  racing.cost_offset_second_change);
  parseJsonDouble(json, "racing_cost_offset_slope", racing.cost_offset_slope);
  parseJsonDouble(json, "racing_cost_collision", racing.cost_collision);
  parseJsonDouble(json, "racing_cost_outside_grid", racing.cost_outside_grid);
  parseJsonDouble(json, "racing_cost_length_overrun", racing.cost_length_overrun);
  parseJsonDouble(json, "racing_final_estimated_time_s", racing.estimated_time_s);
  parseJsonDouble(json, "racing_final_min_speed_limit_mps", racing.min_speed_limit_mps);
  parseJsonDouble(json, "racing_final_max_speed_limit_mps", racing.max_speed_limit_mps);
  parseJsonSize(json, "racing_final_curvature_limited_samples",
                racing.curvature_limited_samples);
  parseJsonDouble(json, "racing_centerline_estimated_time_s",
                  racing.centerline_estimated_time_s);
  parseJsonDouble(json, "racing_centerline_min_speed_limit_mps",
                  racing.centerline_min_speed_limit_mps);
  parseJsonDouble(json, "racing_centerline_max_speed_limit_mps",
                  racing.centerline_max_speed_limit_mps);
  parseJsonSize(json, "racing_centerline_curvature_limited_samples",
                racing.centerline_curvature_limited_samples);
  parseJsonDouble(json, "racing_best_candidate_estimated_time_s",
                  racing.best_candidate_estimated_time_s);
  parseJsonDouble(json, "racing_best_candidate_score", racing.best_candidate_score);
  parseJsonDouble(json, "racing_best_candidate_min_speed_limit_mps",
                  racing.best_candidate_min_speed_limit_mps);
  parseJsonDouble(json, "racing_best_candidate_max_speed_limit_mps",
                  racing.best_candidate_max_speed_limit_mps);
  parseJsonSize(json, "racing_best_candidate_curvature_limited_samples",
                racing.best_candidate_curvature_limited_samples);
  parseJsonDouble(json, "racing_time_gain_s", racing.time_gain_s);
  parseJsonDouble(json, "racing_regularization_time_delta_s",
                  racing.regularization_time_delta_s);
  parseJsonSize(json, "racing_regularization_iterations",
                racing.regularization_iterations);
  parseJsonBool(json, "racing_regularization_applied", racing.regularization_applied);
  parseJsonDouble(json, "racing_pre_regularization_max_curvature_jump_1pm",
                  racing.pre_regularization_max_curvature_jump_1pm);
  parseJsonDouble(json, "racing_post_regularization_max_curvature_jump_1pm",
                  racing.post_regularization_max_curvature_jump_1pm);
  parseJsonSize(json, "racing_skipped_noop_candidates", racing.skipped_noop_candidates);
  parseJsonDouble(json, "racing_candidate_path_evaluation_duration_ms",
                  racing.candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "racing_candidate_score_duration_ms",
                  racing.candidate_score_duration_ms);
  parseJsonDouble(json, "racing_candidate_point_build_duration_ms",
                  racing.candidate_point_build_duration_ms);
  parseJsonDouble(json, "racing_candidate_sample_build_duration_ms",
                  racing.candidate_sample_build_duration_ms);
  parseJsonDouble(json, "racing_candidate_cost_breakdown_duration_ms",
                  racing.candidate_cost_breakdown_duration_ms);
  parseJsonDouble(json, "racing_candidate_shape_diagnostics_duration_ms",
                  racing.candidate_shape_diagnostics_duration_ms);
  parseJsonDouble(json, "racing_candidate_speed_profile_duration_ms",
                  racing.candidate_speed_profile_duration_ms);
  parseJsonSize(json, "racing_candidate_speed_profile_calls",
                racing.candidate_speed_profile_calls);
  parseJsonSize(json, "racing_candidate_speed_profile_samples_total",
                racing.candidate_speed_profile_samples_total);
  parseJsonSize(json, "racing_candidate_speed_profile_samples_max",
                racing.candidate_speed_profile_samples_max);
  parseJsonDouble(json, "racing_regularization_duration_ms",
                  racing.regularization_duration_ms);
  parseJsonSize(json, "racing_scratch_reused_candidates",
                racing.scratch_reused_candidates);
  parseJsonBool(json, "racing_parallel_candidate_evaluation_used",
                racing.parallel_candidate_evaluation_used);
  parseJsonSize(json, "racing_parallel_workers_used", racing.parallel_workers_used);
  parseJsonSize(json, "racing_candidate_chunks", racing.candidate_chunks);
  parseJsonSize(json, "racing_candidate_parallel_batches",
                racing.candidate_parallel_batches);
  parseJsonSize(json, "racing_candidate_threads_launched",
                racing.candidate_threads_launched);
  parseJsonDouble(json, "racing_candidate_batch_wall_duration_ms",
                  racing.candidate_batch_wall_duration_ms);
  parseJsonDouble(json, "racing_candidate_batch_wait_duration_ms",
                  racing.candidate_batch_wait_duration_ms);
  parseJsonDouble(json, "racing_candidate_worker_buffer_prepare_duration_ms",
                  racing.candidate_worker_buffer_prepare_duration_ms);
  parseJsonDouble(json, "racing_candidate_thread_launch_duration_ms",
                  racing.candidate_thread_launch_duration_ms);
  parseJsonDouble(json, "racing_candidate_thread_join_wait_duration_ms",
                  racing.candidate_thread_join_wait_duration_ms);
  parseJsonSize(json, "racing_worker_scratch_reuses", racing.worker_scratch_reuses);
  parseJsonSize(json, "racing_candidate_snapshot_allocations_avoided",
                racing.candidate_snapshot_allocations_avoided);
  parseJsonSize(json, "racing_candidate_offset_changed_samples_total",
                racing.candidate_offset_changed_samples_total);
  parseJsonSize(json, "racing_candidate_offset_changed_samples_max",
                racing.candidate_offset_changed_samples_max);
  parseJsonSize(json, "racing_candidate_offset_changed_span_samples_total",
                racing.candidate_offset_changed_span_samples_total);
  parseJsonSize(json, "racing_candidate_offset_changed_span_samples_max",
                racing.candidate_offset_changed_span_samples_max);
  parseJsonSize(json, "racing_candidate_local_speed_window_samples_total",
                racing.candidate_local_speed_window_samples_total);
  parseJsonSize(json, "racing_candidate_local_speed_window_samples_max",
                racing.candidate_local_speed_window_samples_max);
  parseJsonSize(json, "racing_local_candidate_evaluations",
                racing.local_candidate_evaluations);
  parseJsonSize(json, "racing_local_candidate_full_score_fallbacks",
                racing.local_candidate_full_score_fallbacks);
  parseJsonSize(json, "racing_local_candidate_full_score_required",
                racing.local_candidate_full_score_required);
  parseJsonSize(json, "racing_local_candidate_full_score_required_invalid_input",
                racing.local_candidate_full_score_required_invalid_input);
  parseJsonSize(json, "racing_local_candidate_full_score_required_boundary",
                racing.local_candidate_full_score_required_boundary);
  parseJsonSize(json, "racing_local_candidate_full_score_required_unsafe_base",
                racing.local_candidate_full_score_required_unsafe_base);
  parseJsonSize(json, "racing_local_candidate_full_score_required_window_invalid",
                racing.local_candidate_full_score_required_window_invalid);
  parseJsonSize(json, "racing_local_candidate_acceptance_full_scores",
                racing.local_candidate_acceptance_full_scores);
  parseJsonSize(json, "racing_local_score_false_positives",
                racing.local_score_false_positives);
  parseJsonDouble(json, "racing_local_candidate_point_build_duration_ms",
                  racing.local_candidate_point_build_duration_ms);
  parseJsonDouble(json, "racing_local_candidate_path_evaluation_duration_ms",
                  racing.local_candidate_path_evaluation_duration_ms);
  parseJsonDouble(json, "racing_local_candidate_score_duration_ms",
                  racing.local_candidate_score_duration_ms);
  parseJsonDouble(json, "racing_local_candidate_traversal_estimate_duration_ms",
                  racing.local_candidate_traversal_estimate_duration_ms);
  parseJsonDouble(json, "racing_full_candidate_score_duration_ms",
                  racing.full_candidate_score_duration_ms);
  parseJsonSize(json, "racing_shadow_lower_bound_validation_full_scores",
                racing.shadow_lower_bound_validation_full_scores);
  parseJsonDouble(json, "racing_shadow_lower_bound_validation_full_score_duration_ms",
                  racing.shadow_lower_bound_validation_full_score_duration_ms);
  parseJsonSize(json, "racing_shadow_lower_bound_evaluations",
                racing.shadow_lower_bound_evaluations);
  parseJsonSize(json, "racing_shadow_lower_bound_unavailable",
                racing.shadow_lower_bound_unavailable);
  parseJsonSize(json, "racing_shadow_lower_bound_prunable",
                racing.shadow_lower_bound_prunable);
  parseJsonSize(json, "racing_shadow_lower_bound_false_prunes",
                racing.shadow_lower_bound_false_prunes);
  parseJsonSize(json, "racing_shadow_lower_bound_winner_prunes",
                racing.shadow_lower_bound_winner_prunes);
  parseJsonDouble(json, "racing_shadow_lower_bound_prunable_full_score_duration_ms",
                  racing.shadow_lower_bound_prunable_full_score_duration_ms);
  parseJsonDouble(json, "racing_shadow_lower_bound_max_overestimate_score",
                  racing.shadow_lower_bound_max_overestimate_score);
  parseJsonDouble(json, "racing_shadow_lower_bound_max_underestimate_score",
                  racing.shadow_lower_bound_max_underestimate_score);
  parseJsonDouble(json, "racing_shadow_lower_bound_max_false_prune_improvement_score",
                  racing.shadow_lower_bound_max_false_prune_improvement_score);
  parseJsonSize(json, "racing_shadow_local_speed_evaluations",
                racing.shadow_local_speed_evaluations);
  parseJsonSize(json, "racing_shadow_local_speed_unavailable",
                racing.shadow_local_speed_unavailable);
  parseJsonSize(json, "racing_shadow_local_speed_prunable",
                racing.shadow_local_speed_prunable);
  parseJsonSize(json, "racing_shadow_local_speed_false_prunes",
                racing.shadow_local_speed_false_prunes);
  parseJsonSize(json, "racing_shadow_local_speed_winner_mismatches",
                racing.shadow_local_speed_winner_mismatches);
  parseJsonDouble(json, "racing_shadow_local_speed_abs_time_error_sum_s",
                  racing.shadow_local_speed_abs_time_error_sum_s);
  parseJsonDouble(json, "racing_shadow_local_speed_abs_time_error_p95_s",
                  racing.shadow_local_speed_abs_time_error_p95_s);
  parseJsonDouble(json, "racing_shadow_local_speed_max_time_overestimate_s",
                  racing.shadow_local_speed_max_time_overestimate_s);
  parseJsonDouble(json, "racing_shadow_local_speed_max_time_underestimate_s",
                  racing.shadow_local_speed_max_time_underestimate_s);
  parseJsonDouble(json, "racing_shadow_local_speed_abs_score_error_sum",
                  racing.shadow_local_speed_abs_score_error_sum);
  parseJsonDouble(json, "racing_shadow_local_speed_abs_score_error_p95",
                  racing.shadow_local_speed_abs_score_error_p95);
  parseJsonDouble(json, "racing_shadow_local_speed_max_score_overestimate",
                  racing.shadow_local_speed_max_score_overestimate);
  parseJsonDouble(json, "racing_shadow_local_speed_max_score_underestimate",
                  racing.shadow_local_speed_max_score_underestimate);
  parseJsonDouble(json, "racing_shadow_local_speed_max_false_prune_improvement_score",
                  racing.shadow_local_speed_max_false_prune_improvement_score);
  parseJsonSize(json, "racing_shadow_segment_score_evaluations",
                racing.shadow_segment_score_evaluations);
  parseJsonSize(json, "racing_shadow_segment_score_unavailable",
                racing.shadow_segment_score_unavailable);
  parseJsonSize(json, "racing_shadow_segment_score_prunable",
                racing.shadow_segment_score_prunable);
  parseJsonSize(json, "racing_shadow_segment_score_false_prunes",
                racing.shadow_segment_score_false_prunes);
  parseJsonSize(json, "racing_shadow_segment_score_winner_mismatches",
                racing.shadow_segment_score_winner_mismatches);
  parseJsonSize(json, "racing_shadow_segment_score_window_samples_total",
                racing.shadow_segment_score_window_samples_total);
  parseJsonSize(json, "racing_shadow_segment_score_window_samples_max",
                racing.shadow_segment_score_window_samples_max);
  parseJsonDouble(json, "racing_shadow_segment_score_abs_error_sum",
                  racing.shadow_segment_score_abs_error_sum);
  parseJsonDouble(json, "racing_shadow_segment_score_abs_error_p95",
                  racing.shadow_segment_score_abs_error_p95);
  parseJsonDouble(json, "racing_shadow_segment_score_max_overestimate",
                  racing.shadow_segment_score_max_overestimate);
  parseJsonDouble(json, "racing_shadow_segment_score_max_underestimate",
                  racing.shadow_segment_score_max_underestimate);
  parseJsonDouble(json, "racing_shadow_segment_score_max_false_prune_improvement_score",
                  racing.shadow_segment_score_max_false_prune_improvement_score);
  parseJsonSize(json, "racing_line_window_count", racing.window_count);
  parseJsonSize(json, "racing_line_active_window_count", racing.active_window_count);
  parseJsonSize(json, "racing_line_active_window_samples",
                racing.active_window_samples);
  parseJsonSize(json, "racing_line_active_window_centerline_blocked",
                racing.active_window_centerline_blocked);
  parseJsonSize(json, "racing_line_active_window_heading_change_samples",
                racing.active_window_heading_change_samples);
  parseJsonSize(json, "racing_line_active_window_heading_span_samples",
                racing.active_window_heading_span_samples);
  parseJsonSize(json, "racing_line_active_window_curvature_samples",
                racing.active_window_curvature_samples);
  parseJsonSize(json, "racing_line_active_window_width_change_samples",
                racing.active_window_width_change_samples);
  parseJsonSize(json, "racing_line_active_window_width_asymmetry_samples",
                racing.active_window_width_asymmetry_samples);
  parseJsonSize(json, "racing_line_dp_states", racing.dp_states);
  parseJsonSize(json, "racing_line_dp_transitions", racing.dp_transitions);
  parseJsonSize(json, "racing_line_dp_segment_cache_hits",
                racing.dp_segment_cache_hits);
  parseJsonSize(json, "racing_line_dp_segment_cache_misses",
                racing.dp_segment_cache_misses);
  parseJsonSize(json, "racing_line_candidate_segment_cache_hits",
                racing.candidate_segment_cache_hits);
  parseJsonSize(json, "racing_line_candidate_segment_cache_misses",
                racing.candidate_segment_cache_misses);
  parseJsonSize(json, "racing_line_full_path_segment_cache_hits",
                racing.full_path_segment_cache_hits);
  parseJsonSize(json, "racing_line_full_path_segment_cache_misses",
                racing.full_path_segment_cache_misses);
  parseJsonSize(json, "racing_line_dp_coarse_states", racing.dp_coarse_states);
  parseJsonSize(json, "racing_line_dp_coarse_transitions",
                racing.dp_coarse_transitions);
  parseJsonSize(json, "racing_line_dp_fine_states", racing.dp_fine_states);
  parseJsonSize(json, "racing_line_dp_fine_transitions", racing.dp_fine_transitions);
  parseJsonBool(json, "racing_line_dp_coarse_to_fine_used",
                racing.dp_coarse_to_fine_used);
  parseJsonDouble(json, "racing_line_window_detection_duration_ms",
                  racing.window_detection_duration_ms);
  parseJsonDouble(json, "racing_line_window_eval_duration_ms",
                  racing.window_eval_duration_ms);
  parseJsonDouble(json, "racing_line_dp_duration_ms", racing.dp_duration_ms);
  parseJsonDouble(json, "racing_line_full_final_score_duration_ms",
                  racing.full_final_score_duration_ms);
  parseJsonBool(json, "racing_line_async_refined", racing.async_refined);
  parseJsonDouble(json, "racing_max_abs_offset_m", racing.max_abs_offset_m);
  parseJsonDouble(json, "racing_min_edge_margin_m", racing.min_edge_margin_m);
  parseJsonDouble(json, "racing_mean_edge_margin_m", racing.mean_edge_margin_m);

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
  parseJsonSize(json, "turn_smoothing_rejected_length", turn_smoothing.rejected_length);
  parseJsonSize(json, "turn_smoothing_rejected_not_improved",
                turn_smoothing.rejected_not_improved);
  parseJsonSize(json, "turn_smoothing_rejected_curvature_regression",
                turn_smoothing.rejected_curvature_regression);
  parseJsonSize(json, "turn_smoothing_rejected_radius_regression",
                turn_smoothing.rejected_radius_regression);
  parseJsonSize(json, "turn_smoothing_rejected_speed_regression",
                turn_smoothing.rejected_speed_regression);
  parseJsonSize(json, "turn_smoothing_rejected_time_regression",
                turn_smoothing.rejected_time_regression);
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

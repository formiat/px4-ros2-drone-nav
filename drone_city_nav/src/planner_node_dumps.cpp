#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
[[nodiscard]] std::filesystem::path PlannerNode::corridorSamplesDirectory() {
  return std::filesystem::path{"log"} / "corridor_samples";
}

[[nodiscard]] std::filesystem::path PlannerNode::trajectoryCandidatesDirectory() {
  return std::filesystem::path{"log"} / "trajectory_candidates";
}

bool PlannerNode::writeCorridorSamplesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  return drone_city_nav::writeCorridorSamplesCsvFile(path, result, source_label,
                                                     candidate_path_id);
}

void PlannerNode::writeCorridorSamplesDump(
    const TrajectoryPlannerResult& result, const char* source_label,
    const std::uint64_t candidate_path_id) const {
  if (result.corridor_samples.empty()) {
    return;
  }

  const std::filesystem::path directory = corridorSamplesDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    RCLCPP_WARN(get_logger(), "Failed to create corridor samples directory '%s': %s",
                directory.string().c_str(), error.message().c_str());
    return;
  }

  const std::int64_t stamp_ns = get_clock()->now().nanoseconds();
  const std::filesystem::path latest_path = directory / "latest.csv";
  const std::filesystem::path history_path =
      directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                   std::to_string(stamp_ns) + ".csv");
  const bool wrote_latest =
      writeCorridorSamplesCsvFile(latest_path, result, source_label, candidate_path_id);
  const bool wrote_history = writeCorridorSamplesCsvFile(
      history_path, result, source_label, candidate_path_id);
  if (!wrote_latest || !wrote_history) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to write corridor samples dump: latest='%s' history='%s'",
        latest_path.string().c_str(), history_path.string().c_str());
  }
}

bool PlannerNode::writeTrajectoryOptimizerCandidatesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "phase,decision,local_full_score_reason,iteration,order,center_index,"
            "center_s_m,step_m,delta_m,score,incumbent_score,length_m,noop,"
            "traversable,local_evaluated,requires_full_score,full_score_used,"
            "prohibited_cells,outside_grid_segments,changed_samples,"
            "changed_span_samples,cost_curvature,cost_curvature_change,"
            "cost_radius_shortfall,cost_heading_jump,"
            "cost_offset_change,cost_offset_second_change,cost_offset_slope,"
            "cost_collision,cost_outside_grid,point_build_ms,path_eval_ms,score_ms,"
            "full_score_ms\n";
  for (const TrajectoryOptimizerCandidateDiagnostic& diagnostic :
       result.stats.trajectory_optimizer.candidate_diagnostics) {
    stream << diagnostic.phase << "," << diagnostic.decision << ","
           << diagnostic.local_full_score_reason << "," << diagnostic.iteration << ","
           << diagnostic.order << "," << diagnostic.center_index << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.center_s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.step_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.delta_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.incumbent_score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.length_m);
    stream << "," << (diagnostic.noop ? "true" : "false") << ","
           << (diagnostic.traversable ? "true" : "false") << ","
           << (diagnostic.local_evaluated ? "true" : "false") << ","
           << (diagnostic.requires_full_score ? "true" : "false") << ","
           << (diagnostic.full_score_used ? "true" : "false") << ","
           << diagnostic.prohibited_cells << "," << diagnostic.outside_grid_segments
           << "," << diagnostic.changed_samples << ","
           << diagnostic.changed_span_samples << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_curvature);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_curvature_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_radius_shortfall);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_heading_jump);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_second_change);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_offset_slope);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_collision);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.cost_outside_grid);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.point_build_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.path_evaluation_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score_duration_ms);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.full_score_duration_ms);
    stream << "\n";
  }
  return stream.good();
}

bool PlannerNode::writeTurnSmoothingCandidatesCsvFile(
    const std::filesystem::path& path, const TrajectoryPlannerResult& result,
    const char* source_label, const std::uint64_t candidate_path_id) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "decision,reject_reason,reject_detail,pass,attempt_index,corner_index,"
            "corner_s_m,"
            "entry_distance_m,exit_distance_m,shift_scale,applied_shift_m,"
            "relaxed_angle_deg,score,min_radius_before_m,min_radius_after_m,"
            "min_speed_before_mps,min_speed_after_mps,local_time_before_s,"
            "local_time_after_s,curvature_jump_before_1pm,"
            "curvature_jump_after_1pm,heading_delta_before_rad,"
            "heading_delta_after_rad\n";
  for (const TurnSmoothingCandidateDiagnostic& diagnostic :
       result.stats.turn_smoothing.candidate_diagnostics) {
    stream << diagnostic.decision << "," << diagnostic.reject_reason << ","
           << diagnostic.reject_detail << "," << diagnostic.pass << ","
           << diagnostic.attempt_index << "," << diagnostic.corner_index << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.corner_s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.entry_distance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.exit_distance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.shift_scale);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.applied_shift_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.relaxed_angle_deg);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.score);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_radius_before_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_radius_after_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_speed_before_mps);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.min_speed_after_mps);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.local_time_before_s);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.local_time_after_s);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.curvature_jump_before_1pm);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.curvature_jump_after_1pm);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.heading_delta_before_rad);
    stream << ",";
    writeCsvNumberOrEmpty(stream, diagnostic.heading_delta_after_rad);
    stream << "\n";
  }
  return stream.good();
}

void PlannerNode::writeTrajectoryCandidateDumps(
    const TrajectoryPlannerResult& result, const char* source_label,
    const std::uint64_t candidate_path_id) const {
  const bool has_optimizer_candidates =
      !result.stats.trajectory_optimizer.candidate_diagnostics.empty();
  const bool has_turn_smoothing_candidates =
      !result.stats.turn_smoothing.candidate_diagnostics.empty();
  if (!has_optimizer_candidates && !has_turn_smoothing_candidates) {
    return;
  }

  const std::filesystem::path directory = trajectoryCandidatesDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    RCLCPP_WARN(get_logger(),
                "Failed to create trajectory candidates directory '%s': %s",
                directory.string().c_str(), error.message().c_str());
    return;
  }

  const std::int64_t stamp_ns = get_clock()->now().nanoseconds();
  bool wrote_all = true;
  if (has_optimizer_candidates) {
    const std::filesystem::path latest_path =
        directory / "latest_optimizer_candidates.csv";
    const std::filesystem::path history_path =
        directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                     std::to_string(stamp_ns) + "_optimizer_candidates.csv");
    wrote_all = writeTrajectoryOptimizerCandidatesCsvFile(
                    latest_path, result, source_label, candidate_path_id) &&
                wrote_all;
    wrote_all = writeTrajectoryOptimizerCandidatesCsvFile(
                    history_path, result, source_label, candidate_path_id) &&
                wrote_all;
  }
  if (has_turn_smoothing_candidates) {
    const std::filesystem::path latest_path =
        directory / "latest_turn_smoothing_candidates.csv";
    const std::filesystem::path history_path =
        directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                     std::to_string(stamp_ns) + "_turn_smoothing_candidates.csv");
    wrote_all = writeTurnSmoothingCandidatesCsvFile(latest_path, result, source_label,
                                                    candidate_path_id) &&
                wrote_all;
    wrote_all = writeTurnSmoothingCandidatesCsvFile(history_path, result, source_label,
                                                    candidate_path_id) &&
                wrote_all;
  }
  if (!wrote_all) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to write one or more trajectory candidate dump files in '%s'",
        directory.string().c_str());
  }
}

} // namespace drone_city_nav

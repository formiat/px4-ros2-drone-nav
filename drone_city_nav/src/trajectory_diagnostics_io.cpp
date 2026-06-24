#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <cmath>
#include <iomanip>
#include <ostream>
#include <sstream>
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

} // namespace

std::string finalTrajectorySamplesCsvHeader() {
  return "sample_index,s_m,x,y,tangent_x,tangent_y,curvature_1pm,"
         "arc_radius_m,left_bound_m,right_bound_m,racing_offset_m,"
         "speed_geometric_limit_mps,speed_profiled_limit_mps,speed_reason,"
         "constraint_s_m,constraint_limit_mps,profiled_time_from_start_s,"
         "profiled_time_to_finish_s";
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
  stream << "," << speedConstraintTypeName(speed_sample.reason) << ",";
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
finalTrajectoryDiagnosticsSummaryJson(const TrajectoryPlannerStats& stats,
                                      const TrajectoryShapeDiagnostics& shape) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "{\"racing_final_estimated_time_s\":";
  writeJsonNumberOrNull(stream, stats.racing_line.estimated_time_s);
  appendJsonNumber(stream, "racing_final_min_speed_limit_mps",
                   stats.racing_line.min_speed_limit_mps);
  appendJsonNumber(stream, "racing_final_max_speed_limit_mps",
                   stats.racing_line.max_speed_limit_mps);
  appendJsonSize(stream, "racing_final_curvature_limited_samples",
                 stats.racing_line.curvature_limited_samples);
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

} // namespace drone_city_nav

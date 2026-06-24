#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>

namespace drone_city_nav {
namespace {

template<std::size_t Size>
void expectContainsAll(const std::string& text,
                       const std::array<const char*, Size>& expected_tokens) {
  for (const char* token : expected_tokens) {
    EXPECT_NE(text.find(token), std::string::npos) << token;
  }
}

[[nodiscard]] TrajectoryPlannerStats populatedStats() {
  TrajectoryPlannerStats stats{};
  stats.racing_line.estimated_time_s = 12.5;
  stats.racing_line.min_speed_limit_mps = 1.0;
  stats.racing_line.max_speed_limit_mps = 10.0;
  stats.racing_line.curvature_limited_samples = 3U;
  stats.racing_line.centerline_estimated_time_s = 14.0;
  stats.racing_line.centerline_min_speed_limit_mps = 2.0;
  stats.racing_line.centerline_max_speed_limit_mps = 11.0;
  stats.racing_line.centerline_curvature_limited_samples = 4U;
  stats.racing_line.best_candidate_estimated_time_s = 12.25;
  stats.racing_line.best_candidate_score = 42.0;
  stats.racing_line.best_candidate_min_speed_limit_mps = 1.5;
  stats.racing_line.best_candidate_max_speed_limit_mps = 10.5;
  stats.racing_line.best_candidate_curvature_limited_samples = 5U;
  stats.racing_line.time_gain_s = 1.5;
  stats.racing_line.regularization_applied = true;
  stats.racing_line.regularization_iterations = 2U;
  stats.racing_line.regularization_time_delta_s = 0.1;
  stats.racing_line.pre_regularization_max_curvature_jump_1pm = 0.4;
  stats.racing_line.post_regularization_max_curvature_jump_1pm = 0.2;
  stats.racing_line.centerline_length_m = 100.0;
  stats.racing_line.final_length_m = 108.0;
  stats.racing_line.final_length_ratio = 1.08;
  stats.racing_line.max_abs_offset_m = 3.0;
  stats.racing_line.min_edge_margin_m = 2.5;
  stats.racing_line.mean_edge_margin_m = 4.5;
  stats.racing_line.edge_margin_limited_samples = 6U;
  stats.racing_line.cost_length = 2.0;
  stats.racing_line.cost_time = 625.0;
  stats.racing_line.cost_curvature = 12.0;
  stats.racing_line.cost_curvature_change = 3.0;
  stats.racing_line.cost_offset_change = 1.0;
  stats.racing_line.cost_offset_second_change = 4.0;
  stats.racing_line.cost_center_bias = 0.0;
  stats.racing_line.cost_edge_margin = 7.0;
  stats.racing_line.cost_collision = 0.0;
  stats.racing_line.cost_outside_grid = 0.0;
  stats.racing_line.cost_length_overrun = 0.0;
  return stats;
}

} // namespace

TEST(TrajectoryDiagnosticsIo, CsvHeaderAndRowContainProfiledTiming) {
  TrajectoryPointSample sample{};
  sample.s_m = 4.0;
  sample.point = Point2{1.0, 2.0};
  sample.tangent = Point2{1.0, 0.0};
  sample.curvature_1pm = 0.25;
  sample.left_bound_m = 3.0;
  sample.right_bound_m = 4.0;
  sample.racing_offset_m = -0.5;

  TrajectorySpeedSample speed_sample{};
  speed_sample.geometric_limit_mps = 8.0;
  speed_sample.profiled_limit_mps = 6.0;
  speed_sample.reason = SpeedConstraintType::kArc;
  speed_sample.constraint_s_m = 5.0;
  speed_sample.constraint_limit_mps = 4.0;

  const std::string header = finalTrajectorySamplesCsvHeader();
  const std::string row =
      finalTrajectorySamplesCsvRow(7U, sample, speed_sample, 1.25, 3.5);

  expectContainsAll(header, std::array{
                                "sample_index",
                                "s_m",
                                "x",
                                "y",
                                "curvature_1pm",
                                "speed_geometric_limit_mps",
                                "speed_profiled_limit_mps",
                                "speed_reason",
                                "speed_limit_source",
                                "constraint_s_m",
                                "constraint_limit_mps",
                                "profiled_time_from_start_s",
                                "profiled_time_to_finish_s",
                            });
  EXPECT_NE(row.find("arc"), std::string::npos);
  EXPECT_NE(row.find("1.25"), std::string::npos);
  EXPECT_NE(row.find("3.5"), std::string::npos);
  EXPECT_EQ(row.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, SummaryJsonContainsTraversalAndShapeMetrics) {
  TrajectoryShapeDiagnostics shape{};
  shape.segment_count = 9U;
  shape.max_curvature_jump_1pm = 0.2;
  shape.max_heading_delta_rad = 0.3;
  shape.max_offset_delta_m = 0.4;

  const std::string json =
      finalTrajectoryDiagnosticsSummaryJson(populatedStats(), shape);

  EXPECT_NE(json.find("\"racing_final_estimated_time_s\":12.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_min_speed_limit_mps\":1"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_max_speed_limit_mps\":10"), std::string::npos);
  EXPECT_NE(json.find("\"racing_centerline_estimated_time_s\":14"), std::string::npos);
  EXPECT_NE(json.find("\"racing_centerline_length_m\":100"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_length_ratio\":1.08"), std::string::npos);
  EXPECT_NE(json.find("\"racing_cost_time\":625"), std::string::npos);
  EXPECT_NE(json.find("\"racing_cost_edge_margin\":7"), std::string::npos);
  EXPECT_NE(json.find("\"racing_best_candidate_estimated_time_s\":12.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_applied\":true"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_shape_segment_count\":9"), std::string::npos);
  EXPECT_EQ(json.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, RacingLineJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = racingLineDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment,
                    std::array{
                        "\"racing_final_estimated_time_s\"",
                        "\"racing_final_min_speed_limit_mps\"",
                        "\"racing_final_max_speed_limit_mps\"",
                        "\"racing_final_curvature_limited_samples\"",
                        "\"racing_centerline_length_m\"",
                        "\"racing_final_length_m\"",
                        "\"racing_final_length_ratio\"",
                        "\"racing_max_abs_offset_m\"",
                        "\"racing_min_edge_margin_m\"",
                        "\"racing_mean_edge_margin_m\"",
                        "\"racing_edge_margin_limited_samples\"",
                        "\"racing_cost_length\"",
                        "\"racing_cost_time\"",
                        "\"racing_cost_curvature\"",
                        "\"racing_cost_curvature_change\"",
                        "\"racing_cost_offset_change\"",
                        "\"racing_cost_offset_second_change\"",
                        "\"racing_cost_center_bias\"",
                        "\"racing_cost_edge_margin\"",
                        "\"racing_cost_collision\"",
                        "\"racing_cost_outside_grid\"",
                        "\"racing_cost_length_overrun\"",
                        "\"racing_centerline_estimated_time_s\"",
                        "\"racing_centerline_min_speed_limit_mps\"",
                        "\"racing_centerline_max_speed_limit_mps\"",
                        "\"racing_centerline_curvature_limited_samples\"",
                        "\"racing_best_candidate_estimated_time_s\"",
                        "\"racing_best_candidate_score\"",
                        "\"racing_best_candidate_min_speed_limit_mps\"",
                        "\"racing_best_candidate_max_speed_limit_mps\"",
                        "\"racing_best_candidate_curvature_limited_samples\"",
                        "\"racing_time_gain_s\"",
                        "\"racing_regularization_time_delta_s\"",
                        "\"racing_regularization_iterations\"",
                        "\"racing_regularization_applied\"",
                        "\"racing_pre_regularization_max_curvature_jump_1pm\"",
                        "\"racing_post_regularization_max_curvature_jump_1pm\"",
                    });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, RacingLineJsonFragmentWritesNullForNonFiniteMetrics) {
  const std::string fragment =
      racingLineDiagnosticsJsonFields(TrajectoryPlannerStats{});

  EXPECT_NE(fragment.find("\"racing_final_estimated_time_s\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_centerline_estimated_time_s\":null"),
            std::string::npos);
  EXPECT_NE(fragment.find("\"racing_best_candidate_score\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_final_length_ratio\":null"), std::string::npos);
  EXPECT_NE(fragment.find("\"racing_cost_time\":null"), std::string::npos);
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

} // namespace drone_city_nav

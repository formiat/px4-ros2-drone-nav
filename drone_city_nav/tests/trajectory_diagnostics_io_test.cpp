#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <optional>
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
  stats.racing_line.candidate_path_evaluation_duration_ms = 7.25;
  stats.racing_line.candidate_score_duration_ms = 8.5;
  stats.racing_line.candidate_point_build_duration_ms = 1.25;
  stats.racing_line.candidate_sample_build_duration_ms = 2.5;
  stats.racing_line.regularization_duration_ms = 3.75;
  stats.racing_line.scratch_reused_candidates = 13U;
  stats.racing_line.parallel_candidate_evaluation_used = true;
  stats.racing_line.cost_length = 2.0;
  stats.racing_line.cost_time = 625.0;
  stats.racing_line.cost_curvature = 12.0;
  stats.racing_line.cost_curvature_change = 3.0;
  stats.racing_line.cost_heading_jump = 5.5;
  stats.racing_line.cost_offset_change = 1.0;
  stats.racing_line.cost_offset_second_change = 4.0;
  stats.racing_line.cost_edge_margin = 7.0;
  stats.racing_line.cost_collision = 0.0;
  stats.racing_line.cost_outside_grid = 0.0;
  stats.racing_line.cost_length_overrun = 0.0;
  stats.turn_smoothing.input_samples = 48U;
  stats.turn_smoothing.output_samples = 72U;
  stats.turn_smoothing.detected_corners = 3U;
  stats.turn_smoothing.attempted_corners = 2U;
  stats.turn_smoothing.candidate_attempts = 11U;
  stats.turn_smoothing.relaxed_candidate_attempts = 6U;
  stats.turn_smoothing.smoothed_corners = 1U;
  stats.turn_smoothing.rejected_prohibited = 0U;
  stats.turn_smoothing.rejected_corridor = 1U;
  stats.turn_smoothing.rejected_length = 0U;
  stats.turn_smoothing.rejected_not_improved = 0U;
  stats.turn_smoothing.max_heading_delta_before_rad = 1.2;
  stats.turn_smoothing.max_heading_delta_after_rad = 0.4;
  stats.turn_smoothing.max_curvature_jump_before_1pm = 0.5;
  stats.turn_smoothing.max_curvature_jump_after_1pm = 0.2;
  stats.turn_smoothing.min_inner_margin_m = 2.25;
  stats.turn_smoothing.max_applied_outer_shift_m = 6.5;
  stats.turn_smoothing.accepted_entry_distance_m = 30.0;
  stats.turn_smoothing.accepted_exit_distance_m = 30.0;
  stats.turn_smoothing.accepted_shift_scale = 0.5;
  stats.turn_smoothing.accepted_relaxed_angle_deg = 15.0;
  stats.corridor.samples = 42U;
  stats.corridor.min_width_m = 17.5;
  stats.corridor.mean_width_m = 24.25;
  stats.corridor.max_width_m = 58.75;
  stats.corridor.lateral_limited_samples = 9U;
  stats.corridor.max_center_recovery_m = 1.25;
  stats.corridor.max_lateral_bound_reduction_m = 2.5;
  stats.input_points = 8U;
  stats.samples = 78U;
  stats.length_m = 412.25;
  stats.curvature_min_1pm = -0.05;
  stats.curvature_max_1pm = 0.06;
  stats.curvature_mean_abs_1pm = 0.02;
  stats.speed_profile_min_mps = 0.0;
  stats.speed_profile_mean_mps = 13.4;
  stats.speed_profile_max_mps = 19.1;
  stats.speed_profile_curvature_limited_samples = 69U;
  stats.total_duration_ms = 123.4;
  stats.corridor_duration_ms = 5.5;
  stats.racing_line_duration_ms = 99.9;
  stats.turn_smoothing_duration_ms = 8.75;
  stats.speed_profile_duration_ms = 1.5;
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
  EXPECT_NE(json.find("\"racing_cost_heading_jump\":5.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_point_build_duration_ms\":1.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_candidate_sample_build_duration_ms\":2.5"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_duration_ms\":3.75"), std::string::npos);
  EXPECT_NE(json.find("\"racing_scratch_reused_candidates\":13"), std::string::npos);
  EXPECT_NE(json.find("\"racing_parallel_candidate_evaluation_used\":true"),
            std::string::npos);
  EXPECT_NE(json.find("\"trajectory_total_duration_ms\":123.4"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_racing_line_duration_ms\":99.9"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_best_candidate_estimated_time_s\":12.25"),
            std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_applied\":true"), std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_smoothed_corners\":1"), std::string::npos);
  EXPECT_NE(json.find("\"turn_smoothing_heading_delta_after_rad\":0.4"),
            std::string::npos);
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
                        "\"racing_cost_heading_jump\"",
                        "\"racing_cost_offset_change\"",
                        "\"racing_cost_offset_second_change\"",
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
                        "\"racing_candidate_path_evaluation_duration_ms\"",
                        "\"racing_candidate_score_duration_ms\"",
                        "\"racing_candidate_point_build_duration_ms\"",
                        "\"racing_candidate_sample_build_duration_ms\"",
                        "\"racing_regularization_duration_ms\"",
                        "\"racing_scratch_reused_candidates\"",
                        "\"racing_parallel_candidate_evaluation_used\"",
                    });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, TurnSmoothingJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = turnSmoothingDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment, std::array{
                                  "\"turn_smoothing_input_samples\"",
                                  "\"turn_smoothing_output_samples\"",
                                  "\"turn_smoothing_detected_corners\"",
                                  "\"turn_smoothing_attempted_corners\"",
                                  "\"turn_smoothing_candidate_attempts\"",
                                  "\"turn_smoothing_relaxed_candidate_attempts\"",
                                  "\"turn_smoothing_smoothed_corners\"",
                                  "\"turn_smoothing_rejected_prohibited\"",
                                  "\"turn_smoothing_rejected_corridor\"",
                                  "\"turn_smoothing_rejected_length\"",
                                  "\"turn_smoothing_rejected_not_improved\"",
                                  "\"turn_smoothing_heading_delta_before_rad\"",
                                  "\"turn_smoothing_heading_delta_after_rad\"",
                                  "\"turn_smoothing_curvature_jump_before_1pm\"",
                                  "\"turn_smoothing_curvature_jump_after_1pm\"",
                                  "\"turn_smoothing_min_inner_margin_m\"",
                                  "\"turn_smoothing_max_outer_shift_m\"",
                                  "\"turn_smoothing_accepted_entry_distance_m\"",
                                  "\"turn_smoothing_accepted_exit_distance_m\"",
                                  "\"turn_smoothing_accepted_shift_scale\"",
                                  "\"turn_smoothing_accepted_relaxed_angle_deg\"",
                              });
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, TimingJsonFragmentContainsBlackboxRequiredKeys) {
  const std::string fragment = trajectoryTimingDiagnosticsJsonFields(populatedStats());

  expectContainsAll(fragment, std::array{
                                  "\"trajectory_total_duration_ms\"",
                                  "\"trajectory_corridor_duration_ms\"",
                                  "\"trajectory_racing_line_duration_ms\"",
                                  "\"trajectory_turn_smoothing_duration_ms\"",
                                  "\"trajectory_speed_profile_duration_ms\"",
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
  EXPECT_NE(fragment.find("\"racing_cost_heading_jump\":null"), std::string::npos);
  EXPECT_EQ(fragment.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, PlannerDiagnosticsJsonRoundTripsRuntimeStats) {
  const std::uint64_t planner_path_id = 42U;
  const std::uint64_t path_stamp_ns = 1'782'477'871'305'471'587ULL;
  const std::string json = trajectoryPlannerDiagnosticsJson(
      planner_path_id, path_stamp_ns, populatedStats());

  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> parsed =
      parseTrajectoryPlannerDiagnosticsJson(json);

  ASSERT_TRUE(parsed.has_value());
  const TrajectoryPlannerDiagnosticsEnvelope parsed_value =
      parsed.value_or(TrajectoryPlannerDiagnosticsEnvelope{});
  EXPECT_EQ(parsed_value.planner_path_id, planner_path_id);
  EXPECT_EQ(parsed_value.path_stamp_ns, path_stamp_ns);
  EXPECT_EQ(parsed_value.stats.samples, 78U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.length_m, 412.25);
  EXPECT_EQ(parsed_value.stats.corridor.samples, 42U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.min_width_m, 17.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.mean_width_m, 24.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor.max_width_m, 58.75);
  EXPECT_EQ(parsed_value.stats.corridor.lateral_limited_samples, 9U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.final_length_m, 108.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.final_length_ratio, 1.08);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.time_gain_s, 1.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.min_edge_margin_m, 2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_path_evaluation_duration_ms,
                   7.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_score_duration_ms, 8.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_point_build_duration_ms,
                   1.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.candidate_sample_build_duration_ms,
                   2.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line.regularization_duration_ms, 3.75);
  EXPECT_EQ(parsed_value.stats.racing_line.scratch_reused_candidates, 13U);
  EXPECT_TRUE(parsed_value.stats.racing_line.parallel_candidate_evaluation_used);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.input_samples, 48U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.output_samples, 72U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.candidate_attempts, 11U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.relaxed_candidate_attempts, 6U);
  EXPECT_EQ(parsed_value.stats.turn_smoothing.smoothed_corners, 1U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_heading_delta_before_rad, 1.2);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_heading_delta_after_rad, 0.4);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.min_inner_margin_m, 2.25);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.max_applied_outer_shift_m, 6.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_entry_distance_m, 30.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_exit_distance_m, 30.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_shift_scale, 0.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing.accepted_relaxed_angle_deg, 15.0);
  EXPECT_DOUBLE_EQ(parsed_value.stats.speed_profile_mean_mps, 13.4);
  EXPECT_EQ(parsed_value.stats.speed_profile_curvature_limited_samples, 69U);
  EXPECT_DOUBLE_EQ(parsed_value.stats.total_duration_ms, 123.4);
  EXPECT_DOUBLE_EQ(parsed_value.stats.corridor_duration_ms, 5.5);
  EXPECT_DOUBLE_EQ(parsed_value.stats.racing_line_duration_ms, 99.9);
  EXPECT_DOUBLE_EQ(parsed_value.stats.turn_smoothing_duration_ms, 8.75);
  EXPECT_DOUBLE_EQ(parsed_value.stats.speed_profile_duration_ms, 1.5);
}

} // namespace drone_city_nav

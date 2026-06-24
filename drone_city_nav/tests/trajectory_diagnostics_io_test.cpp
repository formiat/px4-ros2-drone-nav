#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <gtest/gtest.h>

#include <string>

namespace drone_city_nav {

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

  EXPECT_NE(header.find("profiled_time_from_start_s"), std::string::npos);
  EXPECT_NE(header.find("profiled_time_to_finish_s"), std::string::npos);
  EXPECT_NE(row.find("arc"), std::string::npos);
  EXPECT_NE(row.find("1.25"), std::string::npos);
  EXPECT_NE(row.find("3.5"), std::string::npos);
  EXPECT_EQ(row.find("nan"), std::string::npos);
}

TEST(TrajectoryDiagnosticsIo, SummaryJsonContainsTraversalAndShapeMetrics) {
  TrajectoryPlannerStats stats{};
  stats.racing_line.estimated_time_s = 12.5;
  stats.racing_line.centerline_estimated_time_s = 14.0;
  stats.racing_line.best_candidate_estimated_time_s = 12.25;
  stats.racing_line.best_candidate_score = 42.0;
  stats.racing_line.time_gain_s = 1.5;
  stats.racing_line.regularization_applied = true;
  stats.racing_line.regularization_iterations = 2U;
  stats.racing_line.regularization_time_delta_s = 0.1;
  stats.racing_line.pre_regularization_max_curvature_jump_1pm = 0.4;
  stats.racing_line.post_regularization_max_curvature_jump_1pm = 0.2;

  TrajectoryShapeDiagnostics shape{};
  shape.segment_count = 9U;
  shape.max_curvature_jump_1pm = 0.2;
  shape.max_heading_delta_rad = 0.3;
  shape.max_offset_delta_m = 0.4;

  const std::string json = finalTrajectoryDiagnosticsSummaryJson(stats, shape);

  EXPECT_NE(json.find("\"racing_final_estimated_time_s\":12.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_centerline_estimated_time_s\":14"), std::string::npos);
  EXPECT_NE(json.find("\"racing_regularization_applied\":true"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_shape_segment_count\":9"), std::string::npos);
  EXPECT_EQ(json.find("nan"), std::string::npos);
}

} // namespace drone_city_nav

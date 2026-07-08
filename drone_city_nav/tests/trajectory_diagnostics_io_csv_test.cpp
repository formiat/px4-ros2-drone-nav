#include "trajectory_diagnostics_io_test_helpers.hpp"

namespace drone_city_nav {

using trajectory_diagnostics_io_test_helpers::expectContainsAll;
using trajectory_diagnostics_io_test_helpers::populatedStats;

TEST(TrajectoryDiagnosticsIo, CsvHeaderAndRowContainProfiledTiming) {
  TrajectoryPointSample sample{};
  sample.s_m = 4.0;
  sample.point = Point2{1.0, 2.0};
  sample.z_m = 18.0;
  sample.tangent = Point2{1.0, 0.0};
  sample.curvature_1pm = 0.25;
  sample.left_bound_m = 3.0;
  sample.right_bound_m = 4.0;
  sample.lateral_offset_m = -0.5;
  sample.vertical_slope_dz_ds = 0.2;
  sample.vertical_constraint_active = true;
  sample.vertical_hard_window_active = true;
  sample.vertical_safe_min_z_m = 8.0;
  sample.vertical_safe_max_z_m = 10.0;
  sample.vertical_gate_z_m = 9.0;
  sample.vertical_profile_passage_id = "window_01";

  TrajectorySpeedSample speed_sample{};
  speed_sample.geometric_limit_mps = 8.0;
  speed_sample.profiled_limit_mps = 6.0;
  speed_sample.reason = SpeedConstraintType::kArc;
  speed_sample.constraint_s_m = 5.0;
  speed_sample.constraint_limit_mps = 4.0;
  speed_sample.vertical_speed_limit_mps = 10.0;
  speed_sample.vertical_accel_limit_mps = 9.0;
  speed_sample.vertical_jerk_limit_mps = 8.0;

  const std::string header = finalTrajectorySamplesCsvHeader();
  const std::string row =
      finalTrajectorySamplesCsvRow(7U, sample, speed_sample, 1.25, 3.5);

  expectContainsAll(header, std::array{
                                "sample_index",
                                "s_m",
                                "x",
                                "y",
                                "z_m",
                                "curvature_1pm",
                                "vertical_slope_dz_ds",
                                "vertical_speed_limit_mps",
                                "vertical_accel_limit_mps",
                                "vertical_jerk_limit_mps",
                                "vertical_constraint_active",
                                "vertical_profile_passage_id",
                                "vertical_hard_window_active",
                                "vertical_safe_min_z_m",
                                "vertical_safe_max_z_m",
                                "vertical_gate_z_m",
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
  EXPECT_NE(row.find("18"), std::string::npos);
  EXPECT_NE(row.find("window_01"), std::string::npos);
  EXPECT_NE(row.find("true"), std::string::npos);
  EXPECT_NE(row.find("1.25"), std::string::npos);
  EXPECT_NE(row.find("3.5"), std::string::npos);
  EXPECT_EQ(row.find("nan"), std::string::npos);
}

} // namespace drone_city_nav

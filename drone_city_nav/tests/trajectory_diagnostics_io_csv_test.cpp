#include "trajectory_diagnostics_io_test_helpers.hpp"

namespace drone_city_nav {

using trajectory_diagnostics_io_test_helpers::expectContainsAll;
using trajectory_diagnostics_io_test_helpers::populatedStats;

TEST(TrajectoryDiagnosticsIo, CsvHeaderAndRowContainProfiledTiming) {
  TrajectoryPointSample sample{};
  sample.s_m = 4.0;
  sample.point = Point2{1.0, 2.0};
  sample.tangent = Point2{1.0, 0.0};
  sample.curvature_1pm = 0.25;
  sample.left_bound_m = 3.0;
  sample.right_bound_m = 4.0;
  sample.lateral_offset_m = -0.5;

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

} // namespace drone_city_nav

#include "drone_city_nav/final_trajectory_debug_io.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(FinalTrajectoryDebugIoTest, WritesSamplesCsvWithProfiledTimes) {
  std::vector<TrajectoryPointSample> samples{
      TrajectoryPointSample{.s_m = 0.0, .point = Point2{1.0, 2.0}},
      TrajectoryPointSample{.s_m = 2.0, .point = Point2{3.0, 2.0}},
  };
  TrajectorySpeedProfile speed_profile;
  speed_profile.valid = true;
  speed_profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0, .profiled_limit_mps = 2.0},
      TrajectorySpeedSample{.s_m = 2.0, .profiled_limit_mps = 2.0},
  };

  std::ostringstream stream;
  const FinalTrajectorySamplesCsvInput input{
      .source_label = "unit",
      .local_path_update_id = 7U,
      .planner_path_id = 11U,
      .trajectory_valid = true,
      .trajectory_status = TrajectoryPlannerStatus::kOk,
      .samples = samples,
      .speed_profile = &speed_profile,
  };

  ASSERT_TRUE(writeFinalTrajectorySamplesCsv(stream, input));
  const std::string csv = stream.str();
  EXPECT_NE(csv.find("# source=unit local_path_update_id=7 planner_path_id=11"),
            std::string::npos);
  EXPECT_NE(csv.find("sample_index,s_m,x,y"), std::string::npos);
  EXPECT_NE(csv.find("0,0,1,2"), std::string::npos);
  EXPECT_NE(csv.find("1,2,3,2"), std::string::npos);

  const std::vector<double> times =
      finalTrajectoryProfiledTimesFromStart(samples, speed_profile);
  ASSERT_EQ(times.size(), 2U);
  EXPECT_DOUBLE_EQ(times[0], 0.0);
  EXPECT_DOUBLE_EQ(times[1], 1.0);
}

TEST(FinalTrajectoryDebugIoTest, WritesSummaryJsonWithShapeFields) {
  TrajectoryPlannerStats stats;
  stats.total_duration_ms = 4.5;
  stats.racing_line.final_length_m = 42.0;
  stats.racing_line.window_count = 2U;
  stats.racing_line.dp_states = 10U;
  TrajectoryShapeDiagnostics shape;
  shape.max_heading_delta_rad = 0.25;

  std::ostringstream stream;
  ASSERT_TRUE(writeFinalTrajectorySummaryJson(stream, stats, shape));
  const std::string json = stream.str();
  EXPECT_NE(json.find("\"trajectory_total_duration_ms\":4.5"), std::string::npos);
  EXPECT_NE(json.find("\"racing_final_length_m\":42"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_window_count\":2"), std::string::npos);
  EXPECT_NE(json.find("\"racing_line_dp_states\":10"), std::string::npos);
  EXPECT_NE(json.find("\"trajectory_shape_max_heading_delta_rad\":0.25"),
            std::string::npos);
}

TEST(FinalTrajectoryDebugIoTest, HandlesInvalidStationingWithoutThrowing) {
  std::vector<TrajectoryPointSample> samples{
      TrajectoryPointSample{.s_m = 0.0, .point = Point2{0.0, 0.0}},
      TrajectoryPointSample{.s_m = std::numeric_limits<double>::quiet_NaN(),
                            .point = Point2{0.0, 0.0}},
  };
  TrajectorySpeedProfile speed_profile;
  speed_profile.valid = true;
  speed_profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0, .profiled_limit_mps = 0.0},
  };

  std::ostringstream stream;
  const FinalTrajectorySamplesCsvInput input{
      .source_label = "edge",
      .samples = samples,
      .speed_profile = &speed_profile,
  };
  EXPECT_TRUE(writeFinalTrajectorySamplesCsv(stream, input));
  EXPECT_NE(stream.str().find("trajectory_status=invalid_trajectory"),
            std::string::npos);
}

} // namespace
} // namespace drone_city_nav

#include "drone_city_nav/corridor_samples_io.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>

namespace drone_city_nav {
namespace {

TEST(CorridorSamplesIoTest, WritesExpectedCsvColumnsAndValues) {
  TrajectoryPlannerResult result;
  result.valid = true;
  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.corridor_samples.push_back(CorridorSample{
      .s_m = 3.0,
      .route_center = Point2{1.0, 2.0},
      .center = Point2{1.5, 2.5},
      .tangent = Point2{1.0, 0.0},
      .normal = Point2{0.0, 1.0},
      .left_bound_m = 4.0,
      .right_bound_m = 5.0,
      .clearance_m = 2.0,
      .center_recovery_m = 0.25,
  });
  TrajectoryPointSample sample{};
  sample.curvature_1pm = 0.1;
  sample.racing_offset_m = 0.5;
  result.samples.push_back(sample);
  result.racing_windows.push_back(RacingLineWindowMetadata{
      .id = 1U,
      .begin_s_m = 2.0,
      .end_s_m = 4.0,
  });

  std::ostringstream stream;
  ASSERT_TRUE(writeCorridorSamplesCsv(stream, result, "unit", 9U));
  const std::string csv = stream.str();
  EXPECT_NE(csv.find("# source=unit candidate_path_id=9 status=none valid=true"),
            std::string::npos);
  EXPECT_NE(csv.find("sample_index,s_m,route_x,route_y,center_x,center_y"),
            std::string::npos);
  EXPECT_NE(csv.find("window_id,active_window,selected_offset_m,"
                     "distance_to_prohibited_m"),
            std::string::npos);
  EXPECT_NE(csv.find("0,3,1,2,1.5,2.5,1,0,0,1,4,5,9,2,0.25,1,true,0.5,2"),
            std::string::npos);
}

TEST(CorridorSamplesIoTest, WritesDistinctWindowIdsFromMetadata) {
  TrajectoryPlannerResult result;
  result.valid = true;
  result.stats.status = TrajectoryPlannerStatus::kOk;
  result.racing_windows.push_back(RacingLineWindowMetadata{
      .id = 7U,
      .begin_s_m = 0.0,
      .end_s_m = 2.0,
  });
  result.racing_windows.push_back(RacingLineWindowMetadata{
      .id = 9U,
      .begin_s_m = 8.0,
      .end_s_m = 12.0,
  });
  for (const double s_m : {1.0, 5.0, 10.0}) {
    result.corridor_samples.push_back(CorridorSample{
        .s_m = s_m,
        .route_center = Point2{s_m, 0.0},
        .center = Point2{s_m, 1.0},
        .tangent = Point2{1.0, 0.0},
        .normal = Point2{0.0, 1.0},
        .left_bound_m = 3.0,
        .right_bound_m = 4.0,
        .clearance_m = 5.0,
        .center_recovery_m = 0.0,
    });
    TrajectoryPointSample sample{};
    sample.s_m = s_m;
    sample.racing_offset_m = s_m * 0.1;
    result.samples.push_back(sample);
  }

  std::ostringstream stream;
  ASSERT_TRUE(writeCorridorSamplesCsv(stream, result, "unit", 10U));
  const std::string csv = stream.str();

  EXPECT_NE(csv.find("0,1,1,0,1,1,1,0,0,1,3,4,7,5,0,7,true,0.1,5"), std::string::npos);
  EXPECT_NE(csv.find("1,5,5,0,5,1,1,0,0,1,3,4,7,5,0,,false,0.5,5"), std::string::npos);
  EXPECT_NE(csv.find("2,10,10,0,10,1,1,0,0,1,3,4,7,5,0,9,true,1,5"), std::string::npos);
}

TEST(CorridorSamplesIoTest, WritesEmptyCellForNonFiniteNumber) {
  std::ostringstream stream;
  writeCsvNumberOrEmpty(stream, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(stream.str().empty());
}

} // namespace
} // namespace drone_city_nav

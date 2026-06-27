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

  std::ostringstream stream;
  ASSERT_TRUE(writeCorridorSamplesCsv(stream, result, "unit", 9U));
  const std::string csv = stream.str();
  EXPECT_NE(csv.find("# source=unit candidate_path_id=9 status=none valid=true"),
            std::string::npos);
  EXPECT_NE(csv.find("sample_index,s_m,route_x,route_y,center_x,center_y"),
            std::string::npos);
  EXPECT_NE(csv.find("0,3,1,2,1.5,2.5,1,0,0,1,4,5,9,2,0.25"), std::string::npos);
}

TEST(CorridorSamplesIoTest, WritesEmptyCellForNonFiniteNumber) {
  std::ostringstream stream;
  writeCsvNumberOrEmpty(stream, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(stream.str().empty());
}

} // namespace
} // namespace drone_city_nav

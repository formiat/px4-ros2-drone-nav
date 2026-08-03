#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(TrackedAgentLidarFilterTest, InvalidatesOnlyAgentHits) {
  const std::vector<float> ranges{10.0F, 10.0F, 10.0F};
  const TrackedAgentLidarFilterResult result =
      filterTrackedAgentLidarHits(ranges, TrackedAgentLidarFilterInput{
                                              .scan_pose = Pose2{Point2{0.0, 0.0}, 0.0},
                                              .scan_altitude_m = 10.0,
                                              .angle_min_rad = -0.1,
                                              .angle_increment_rad = 0.1,
                                              .agent_position = Point3{10.0, 0.0, 10.0},
                                              .agent_radius_m = 0.5,
                                              .vertical_tolerance_m = 1.0,
                                          });
  ASSERT_EQ(result.filtered_beams, 1U);
  EXPECT_TRUE(std::isfinite(result.ranges[0]));
  EXPECT_TRUE(std::isnan(result.ranges[1]));
  EXPECT_TRUE(std::isfinite(result.ranges[2]));
}

TEST(TrackedAgentLidarFilterTest, LeavesScanUntouchedAtDifferentAltitude) {
  const std::vector<float> ranges{10.0F};
  const TrackedAgentLidarFilterResult result =
      filterTrackedAgentLidarHits(ranges, TrackedAgentLidarFilterInput{
                                              .scan_pose = Pose2{Point2{0.0, 0.0}, 0.0},
                                              .scan_altitude_m = 10.0,
                                              .agent_position = Point3{10.0, 0.0, 15.0},
                                              .agent_radius_m = 1.0,
                                              .vertical_tolerance_m = 1.0,
                                          });
  EXPECT_EQ(result.filtered_beams, 0U);
  EXPECT_FLOAT_EQ(result.ranges.front(), 10.0F);
}

} // namespace
} // namespace drone_city_nav

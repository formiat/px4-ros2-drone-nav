#include "drone_city_nav/safe_trajectory_truncation.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample> lineSamples() {
  std::vector<Point2> points;
  for (int x = 0; x <= 100; x += 10) {
    points.push_back(Point2{static_cast<double>(x), 0.0});
  }
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(points);
  assignTrajectorySampleAltitude(samples, 18.0);
  return samples;
}

} // namespace

TEST(SafeTrajectoryTruncation, RetainsPrefixToFixedMarginBeforeBlocker) {
  const std::vector<TrajectoryPointSample> samples = lineSamples();

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples, SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                               .blocker_path_distance_m = 50.0,
                                               .truncation_margin_m = 10.0});

  ASSERT_TRUE(result.applied) << result.reason;
  EXPECT_FALSE(result.immediate_hold);
  EXPECT_STREQ(result.reason, "safe_prefix");
  EXPECT_NEAR(result.current_s_m, 20.0, 1.0e-6);
  EXPECT_NEAR(result.blocker_s_m, 70.0, 1.0e-6);
  EXPECT_NEAR(result.stop_s_m, 60.0, 1.0e-6);
  ASSERT_TRUE(trajectorySamplesAreUsable(result.samples));
  EXPECT_NEAR(result.samples.front().point.x, 20.0, 1.0e-6);
  EXPECT_NEAR(result.samples.back().point.x, 60.0, 1.0e-6);
  EXPECT_NEAR(result.samples.back().s_m, 40.0, 1.0e-6);
}

TEST(SafeTrajectoryTruncation, HoldsImmediatelyWhenMarginIsAlreadyPassed) {
  const std::vector<TrajectoryPointSample> samples = lineSamples();

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples, SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                               .blocker_path_distance_m = 8.0,
                                               .truncation_margin_m = 10.0});

  EXPECT_TRUE(result.applied);
  EXPECT_TRUE(result.immediate_hold);
  EXPECT_STREQ(result.reason, "stop_station_not_ahead");
  EXPECT_TRUE(result.samples.empty());
}

} // namespace drone_city_nav

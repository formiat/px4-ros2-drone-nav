#include "drone_city_nav/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {

TEST(Trajectory, ProjectsPointOnLineSegment) {
  const std::vector<TrajectorySegment> trajectory =
      lineTrajectoryFromPoints(std::vector<Point2>{{0.0, 0.0}, {10.0, 0.0}});

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectory(trajectory, Point2{4.0, 3.0});

  ASSERT_TRUE(projection.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  const TrajectoryProjection projection_value = projection.value();
  EXPECT_NEAR(projection_value.point.x, 4.0, 1.0e-9);
  EXPECT_NEAR(projection_value.point.y, 0.0, 1.0e-9);
  EXPECT_NEAR(projection_value.s_m, 4.0, 1.0e-9);
  EXPECT_NEAR(projection_value.tangent.x, 1.0, 1.0e-9);
  EXPECT_NEAR(projection_value.tangent.y, 0.0, 1.0e-9);
  EXPECT_NEAR(projection_value.curvature_1pm, 0.0, 1.0e-9);
}

TEST(Trajectory, ProjectsPointOnCounterClockwiseArc) {
  std::vector<TrajectorySegment> trajectory{makeArcSegment(
      Point2{1.0, 0.0}, Point2{0.0, 1.0}, Point2{0.0, 0.0}, std::numbers::pi / 2.0)};
  assignTrajectoryStationing(trajectory);

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectory(trajectory, Point2{0.7, 0.7});

  ASSERT_TRUE(projection.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  const TrajectoryProjection projection_value = projection.value();
  EXPECT_NEAR(projection_value.s_m, std::numbers::pi / 4.0, 1.0e-6);
  EXPECT_NEAR(projection_value.point.x, std::sqrt(0.5), 1.0e-6);
  EXPECT_NEAR(projection_value.point.y, std::sqrt(0.5), 1.0e-6);
  EXPECT_NEAR(projection_value.curvature_1pm, 1.0, 1.0e-9);
  EXPECT_NEAR(projection_value.tangent.x, -std::sqrt(0.5), 1.0e-6);
  EXPECT_NEAR(projection_value.tangent.y, std::sqrt(0.5), 1.0e-6);
}

TEST(Trajectory, ClockwiseArcHasNegativeCurvatureAndTangent) {
  std::vector<TrajectorySegment> trajectory{makeArcSegment(
      Point2{1.0, 0.0}, Point2{0.0, -1.0}, Point2{0.0, 0.0}, -std::numbers::pi / 2.0)};
  assignTrajectoryStationing(trajectory);

  EXPECT_NEAR(trajectoryCurvatureAtS(trajectory, 0.1), -1.0, 1.0e-9);
  const Point2 tangent = trajectoryTangentAtS(trajectory, 0.0);
  EXPECT_NEAR(tangent.x, 0.0, 1.0e-9);
  EXPECT_NEAR(tangent.y, -1.0, 1.0e-9);
}

TEST(Trajectory, MetricsAndSamplingCoverEndPoint) {
  std::vector<TrajectorySegment> trajectory{
      makeLineSegment(Point2{0.0, 0.0}, Point2{2.0, 0.0}),
      makeArcSegment(Point2{2.0, 0.0}, Point2{3.0, 1.0}, Point2{2.0, 1.0},
                     -std::numbers::pi / 2.0)};
  assignTrajectoryStationing(trajectory);

  const TrajectoryMetrics metrics = trajectoryMetrics(trajectory);
  const std::vector<Point2> samples = sampleTrajectory(trajectory, 0.5);

  EXPECT_EQ(metrics.line_segments, 1U);
  EXPECT_EQ(metrics.arc_segments, 1U);
  EXPECT_NEAR(metrics.length_m, 2.0 + std::numbers::pi / 2.0, 1.0e-9);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.back().x, 3.0, 1.0e-9);
  EXPECT_NEAR(samples.back().y, 1.0, 1.0e-9);
}

TEST(Trajectory, InvalidInputsReturnInvalidProjection) {
  const std::vector<TrajectorySegment> trajectory =
      lineTrajectoryFromPoints(std::vector<Point2>{{0.0, 0.0}, {10.0, 0.0}});

  EXPECT_FALSE(projectOnTrajectory(
                   trajectory, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0})
                   .has_value());
  EXPECT_FALSE(
      projectOnTrajectory(std::span<const TrajectorySegment>{}, Point2{0.0, 0.0})
          .has_value());
}

} // namespace drone_city_nav

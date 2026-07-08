#include "drone_city_nav/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
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

TEST(Trajectory, DetailedSamplesAreMonotonicAndPreserveCurvature) {
  std::vector<TrajectorySegment> trajectory{
      makeLineSegment(Point2{0.0, 0.0}, Point2{2.0, 0.0}),
      makeArcSegment(Point2{2.0, 0.0}, Point2{3.0, 1.0}, Point2{2.0, 1.0},
                     -std::numbers::pi / 2.0)};
  assignTrajectoryStationing(trajectory);

  const std::vector<TrajectoryPointSample> samples =
      sampleTrajectoryDetailed(trajectory, 0.25);

  ASSERT_TRUE(trajectorySamplesAreUsable(samples));
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    EXPECT_GE(samples[i].s_m, samples[i - 1U].s_m);
  }
  const auto arc_sample = std::ranges::find_if(
      samples, [](const auto& sample) { return std::abs(sample.curvature_1pm) > 0.5; });
  ASSERT_NE(arc_sample, samples.end());
  EXPECT_NEAR(arc_sample->curvature_1pm, -1.0, 1.0e-9);
}

TEST(Trajectory, ProjectsOnSamplesWithInterpolatedTangentAndCurvature) {
  std::vector<TrajectoryPointSample> samples(2U);
  samples[0].s_m = 0.0;
  samples[0].point = Point2{0.0, 0.0};
  samples[0].tangent = Point2{1.0, 0.0};
  samples[0].curvature_1pm = 0.0;
  samples[0].z_m = 10.0;
  samples[1].s_m = 10.0;
  samples[1].point = Point2{10.0, 0.0};
  samples[1].tangent = Point2{0.0, 1.0};
  samples[1].curvature_1pm = 0.2;
  samples[1].z_m = 20.0;

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(samples, Point2{5.0, 2.0});

  ASSERT_TRUE(projection.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  const TrajectoryProjection projection_value = projection.value();
  EXPECT_NEAR(projection_value.point.x, 5.0, 1.0e-9);
  EXPECT_NEAR(projection_value.point.y, 0.0, 1.0e-9);
  EXPECT_NEAR(projection_value.s_m, 5.0, 1.0e-9);
  EXPECT_NEAR(projection_value.tangent.x, std::sqrt(0.5), 1.0e-9);
  EXPECT_NEAR(projection_value.tangent.y, std::sqrt(0.5), 1.0e-9);
  EXPECT_NEAR(projection_value.curvature_1pm, 0.1, 1.0e-9);
  EXPECT_NEAR(trajectorySampleAltitudeAtS(samples, projection_value.s_m), 15.0, 1.0e-9);
}

TEST(Trajectory, LineTrajectoryFromSamplesRejectsNonFiniteSample) {
  std::vector<TrajectoryPointSample> samples(2U);
  samples[0].point = Point2{0.0, 0.0};
  samples[0].tangent = Point2{1.0, 0.0};
  samples[1].s_m = 1.0;
  samples[1].point = Point2{std::numeric_limits<double>::quiet_NaN(), 0.0};
  samples[1].tangent = Point2{1.0, 0.0};

  EXPECT_TRUE(lineTrajectoryFromSamples(samples).empty());
  EXPECT_FALSE(trajectorySamplesAreUsable(samples));
}

TEST(Trajectory, SamplesRejectNonFiniteAltitude) {
  std::vector<TrajectoryPointSample> samples(2U);
  samples[0].s_m = 0.0;
  samples[0].point = Point2{0.0, 0.0};
  samples[0].tangent = Point2{1.0, 0.0};
  samples[0].z_m = 12.0;
  samples[1].s_m = 1.0;
  samples[1].point = Point2{1.0, 0.0};
  samples[1].tangent = Point2{1.0, 0.0};
  samples[1].z_m = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(trajectorySamplesAreUsable(samples));
  EXPECT_TRUE(lineTrajectoryFromSamples(samples).empty());
}

TEST(Trajectory, AssignsAndSummarizesSampleAltitude) {
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{Point2{0.0, 0.0}, Point2{4.0, 0.0}, Point2{8.0, 0.0}});
  ASSERT_TRUE(trajectorySamplesAreUsable(samples));

  assignTrajectorySampleAltitude(samples, 18.0);

  EXPECT_DOUBLE_EQ(samples.front().z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 18.0);
  const TrajectoryAltitudeStats stats = trajectoryAltitudeStats(samples);
  ASSERT_TRUE(stats.valid);
  EXPECT_DOUBLE_EQ(stats.min_z_m, 18.0);
  EXPECT_DOUBLE_EQ(stats.mean_z_m, 18.0);
  EXPECT_DOUBLE_EQ(stats.max_z_m, 18.0);
}

TEST(Trajectory, AltitudeInterpolationIsByStationing) {
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{Point2{0.0, 0.0}, Point2{4.0, 0.0}, Point2{8.0, 0.0}});
  ASSERT_EQ(samples.size(), 3U);
  samples[0].z_m = 10.0;
  samples[1].z_m = 20.0;
  samples[2].z_m = 30.0;

  EXPECT_NEAR(trajectorySampleAltitudeAtS(samples, 2.0), 15.0, 1.0e-9);
  EXPECT_NEAR(trajectorySampleAltitudeAtS(samples, 6.0), 25.0, 1.0e-9);
  EXPECT_NEAR(trajectorySampleAltitudeAtS(samples, -5.0), 10.0, 1.0e-9);
  EXPECT_NEAR(trajectorySampleAltitudeAtS(samples, 99.0), 30.0, 1.0e-9);
}

TEST(Trajectory, VerticalTargetInterpolatesAltitudeSlopeAndPassageMetadata) {
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{20.0, 0.0}});
  ASSERT_EQ(samples.size(), 3U);
  samples[0].z_m = 10.0;
  samples[0].vertical_slope_dz_ds = 0.1;
  samples[1].z_m = 12.0;
  samples[1].vertical_slope_dz_ds = 0.3;
  samples[1].vertical_constraint_active = true;
  samples[1].vertical_hard_window_active = true;
  samples[1].vertical_safe_min_z_m = 9.0;
  samples[1].vertical_safe_max_z_m = 13.0;
  samples[1].vertical_gate_z_m = 11.0;
  samples[1].vertical_profile_passage_id = "arch_main";
  samples[2].z_m = 20.0;
  samples[2].vertical_slope_dz_ds = 0.5;

  const TrajectoryVerticalTarget target = trajectoryVerticalTargetAtS(samples, 5.0);

  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.s_m, 5.0, 1.0e-9);
  EXPECT_NEAR(target.z_m, 11.0, 1.0e-9);
  EXPECT_NEAR(target.vertical_slope_dz_ds, 0.2, 1.0e-9);
  EXPECT_TRUE(target.vertical_constraint_active);
  EXPECT_TRUE(target.vertical_hard_window_active);
  EXPECT_DOUBLE_EQ(target.vertical_safe_min_z_m, 9.0);
  EXPECT_DOUBLE_EQ(target.vertical_safe_max_z_m, 13.0);
  EXPECT_DOUBLE_EQ(target.vertical_gate_z_m, 11.0);
  EXPECT_EQ(target.vertical_profile_passage_id, "arch_main");
}

TEST(Trajectory, VerticalTargetRejectsInvalidSamples) {
  std::vector<TrajectoryPointSample> samples = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  ASSERT_EQ(samples.size(), 2U);
  samples[1].z_m = std::numeric_limits<double>::quiet_NaN();

  const TrajectoryVerticalTarget target = trajectoryVerticalTargetAtS(samples, 5.0);

  EXPECT_FALSE(target.valid);
}

TEST(Trajectory, ProjectionAndLengthStayTwoDimensionalWithAltitude) {
  std::vector<TrajectoryPointSample> flat_samples = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  std::vector<TrajectoryPointSample> elevated_samples = flat_samples;
  elevated_samples[0].z_m = 0.0;
  elevated_samples[1].z_m = 100.0;

  const std::vector<TrajectorySegment> flat = lineTrajectoryFromSamples(flat_samples);
  const std::vector<TrajectorySegment> elevated =
      lineTrajectoryFromSamples(elevated_samples);
  const std::optional<TrajectoryProjection> flat_projection =
      projectOnTrajectorySamples(flat_samples, Point2{5.0, 3.0});
  const std::optional<TrajectoryProjection> elevated_projection =
      projectOnTrajectorySamples(elevated_samples, Point2{5.0, 3.0});

  ASSERT_TRUE(flat_projection.has_value());
  ASSERT_TRUE(elevated_projection.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  const TrajectoryProjection flat_projection_value = flat_projection.value();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access): asserted above.
  const TrajectoryProjection elevated_projection_value = elevated_projection.value();
  EXPECT_DOUBLE_EQ(trajectoryLengthM(flat), trajectoryLengthM(elevated));
  EXPECT_DOUBLE_EQ(flat_projection_value.point.x, elevated_projection_value.point.x);
  EXPECT_DOUBLE_EQ(flat_projection_value.point.y, elevated_projection_value.point.y);
  EXPECT_DOUBLE_EQ(flat_projection_value.s_m, elevated_projection_value.s_m);
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

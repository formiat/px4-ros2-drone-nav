#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace drone_city_nav {

TEST(TrajectoryDiagnostics, ReportsSegmentAndShapeOutliers) {
  std::vector<TrajectoryPointSample> samples;
  samples.push_back(TrajectoryPointSample{
      .s_m = 0.0,
      .point = Point2{0.0, 0.0},
      .lateral_offset_m = 0.0,
  });
  samples.push_back(TrajectoryPointSample{
      .s_m = 0.25,
      .point = Point2{0.25, 0.0},
      .curvature_1pm = 0.0,
      .lateral_offset_m = 0.2,
  });
  samples.push_back(TrajectoryPointSample{
      .s_m = 1.25,
      .point = Point2{1.25, 0.0},
      .curvature_1pm = 0.3,
      .lateral_offset_m = 1.0,
  });
  samples.push_back(TrajectoryPointSample{
      .s_m = 2.25,
      .point = Point2{1.25, 1.0},
      .curvature_1pm = -0.1,
      .lateral_offset_m = 0.1,
  });

  const TrajectoryShapeDiagnostics diagnostics =
      computeTrajectoryShapeDiagnostics(samples);

  EXPECT_EQ(diagnostics.segment_count, 3U);
  EXPECT_EQ(diagnostics.segments_shorter_than_0_5m, 1U);
  EXPECT_EQ(diagnostics.segments_shorter_than_1m, 1U);
  EXPECT_EQ(diagnostics.segments_shorter_than_2m, 3U);
  EXPECT_NEAR(diagnostics.min_segment_length_m, 0.25, 1.0e-9);
  EXPECT_NEAR(diagnostics.max_segment_length_m, 1.0, 1.0e-9);
  EXPECT_NEAR(diagnostics.mean_segment_length_m, 0.75, 1.0e-9);
  EXPECT_NEAR(diagnostics.max_heading_delta_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_EQ(diagnostics.max_heading_delta_index, 3U);
  EXPECT_NEAR(diagnostics.max_curvature_jump_1pm, 0.4, 1.0e-9);
  EXPECT_EQ(diagnostics.max_curvature_jump_index, 3U);
  EXPECT_NEAR(diagnostics.max_offset_delta_m, 0.9, 1.0e-9);
  EXPECT_EQ(diagnostics.max_offset_second_delta_index, 2U);
}

TEST(TrajectoryDiagnostics, EmptyTrajectoryReturnsZeroCounts) {
  const TrajectoryShapeDiagnostics diagnostics =
      computeTrajectoryShapeDiagnostics(std::span<const TrajectoryPointSample>{});

  EXPECT_EQ(diagnostics.segment_count, 0U);
  EXPECT_TRUE(std::isnan(diagnostics.min_segment_length_m));
  EXPECT_TRUE(std::isnan(diagnostics.mean_segment_length_m));
}

} // namespace drone_city_nav

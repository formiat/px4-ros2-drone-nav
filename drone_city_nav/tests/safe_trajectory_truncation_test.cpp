#include "drone_city_nav/safe_trajectory_truncation.hpp"

#include <gtest/gtest.h>

#include <cmath>
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

[[nodiscard]] OccupancyGrid2D prohibitedGrid() {
  OccupancyGrid2D grid{GridBounds{-0.5, -10.5, 1.0, 102, 22}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
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

TEST(SafeTrajectoryTruncation, PreservesPassageMetadataWhenRebasingPrefix) {
  std::vector<TrajectoryPointSample> samples = lineSamples();
  for (TrajectoryPointSample& sample : samples) {
    if (sample.s_m <= 30.0) {
      sample.vertical_profile_passage_id = "connector_04_12_opening";
      sample.vertical_hard_window_active = true;
      sample.vertical_safe_min_z_m = 12.5;
      sample.vertical_safe_max_z_m = 17.5;
      sample.vertical_gate_z_m = 16.5;
      continue;
    }
    sample.z_m = 23.5;
    sample.vertical_profile_passage_id = "connector_06_14_opening";
    sample.vertical_hard_window_active = true;
    sample.vertical_safe_min_z_m = 22.5;
    sample.vertical_safe_max_z_m = 27.5;
    sample.vertical_gate_z_m = 23.5;
  }

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples, SafeTrajectoryTruncationRequest{.current_position = Point2{45.0, 0.0},
                                               .blocker_path_distance_m = 45.0,
                                               .truncation_margin_m = 15.0});

  ASSERT_TRUE(result.applied) << result.reason;
  ASSERT_FALSE(result.immediate_hold);
  ASSERT_TRUE(trajectorySamplesAreUsable(result.samples));
  ASSERT_FALSE(result.samples.empty());
  EXPECT_NEAR(result.samples.front().s_m, 0.0, 1.0e-6);
  EXPECT_NEAR(result.samples.back().s_m, 30.0, 1.0e-6);
  for (const TrajectoryPointSample& sample : result.samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 23.5);
    EXPECT_EQ(sample.vertical_profile_passage_id, "connector_06_14_opening");
    EXPECT_TRUE(sample.vertical_hard_window_active);
    EXPECT_DOUBLE_EQ(sample.vertical_safe_min_z_m, 22.5);
    EXPECT_DOUBLE_EQ(sample.vertical_safe_max_z_m, 27.5);
    EXPECT_DOUBLE_EQ(sample.vertical_gate_z_m, 23.5);
  }
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

TEST(SafeTrajectoryTruncation, MovesTerminalBackwardToProhibitedClearance) {
  const std::vector<TrajectoryPointSample> samples = lineSamples();
  OccupancyGrid2D prohibited_grid = prohibitedGrid();
  prohibited_grid.setOccupied(GridIndex{60, 13});

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples,
      SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                      .blocker_path_distance_m = 55.0,
                                      .truncation_margin_m = 15.0,
                                      .raw_occupancy = &prohibited_grid,
                                      .terminal_clearance_beyond_critical_m = 5.0});

  ASSERT_TRUE(result.applied) << result.reason;
  EXPECT_FALSE(result.immediate_hold);
  EXPECT_TRUE(result.clearance_adjusted);
  EXPECT_NEAR(result.nominal_stop_s_m, 60.0, 1.0e-6);
  EXPECT_NEAR(result.stop_s_m, 54.0, 1.0e-6);
  EXPECT_GE(result.terminal_raw_clearance_m, 6.0);
  ASSERT_FALSE(result.samples.empty());
  EXPECT_NEAR(result.samples.back().point.x, 54.0, 1.0e-6);
}

TEST(SafeTrajectoryTruncation, IncludesFormerCriticalBandInRawTerminalClearance) {
  const std::vector<TrajectoryPointSample> samples = lineSamples();
  OccupancyGrid2D prohibited_grid = prohibitedGrid();
  prohibited_grid.setOccupied(GridIndex{60, 10});

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples,
      SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                      .blocker_path_distance_m = 55.0,
                                      .truncation_margin_m = 15.0,
                                      .raw_occupancy = &prohibited_grid,
                                      .terminal_clearance_beyond_critical_m = 5.0});

  ASSERT_TRUE(result.applied) << result.reason;
  EXPECT_FALSE(result.immediate_hold);
  EXPECT_TRUE(result.clearance_adjusted);
  EXPECT_LE(result.stop_s_m, 54.0);
  EXPECT_GE(result.terminal_raw_clearance_m, 6.0);
}

TEST(SafeTrajectoryTruncation, HoldsWhenNoClearTerminalExistsAhead) {
  const std::vector<TrajectoryPointSample> samples = lineSamples();
  OccupancyGrid2D prohibited_grid = prohibitedGrid();
  for (int x = 20; x <= 60; ++x) {
    prohibited_grid.setOccupied(GridIndex{x, 12});
  }

  const SafeTrajectoryTruncationResult result = truncateTrajectoryBeforeBlocker(
      samples,
      SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                      .blocker_path_distance_m = 55.0,
                                      .truncation_margin_m = 15.0,
                                      .raw_occupancy = &prohibited_grid,
                                      .terminal_clearance_beyond_critical_m = 5.0});

  EXPECT_TRUE(result.applied);
  EXPECT_TRUE(result.immediate_hold);
  EXPECT_STREQ(result.reason, "no_clear_stop_station_ahead");
  EXPECT_TRUE(result.samples.empty());
}

TEST(SafeTrajectoryTruncation, StitchesRemainingPrefixToSuffixAtConfirmedPoint) {
  const std::vector<TrajectoryPointSample> full_samples = lineSamples();
  const SafeTrajectoryTruncationResult truncation = truncateTrajectoryBeforeBlocker(
      full_samples,
      SafeTrajectoryTruncationRequest{.current_position = Point2{20.0, 0.0},
                                      .blocker_path_distance_m = 50.0,
                                      .truncation_margin_m = 10.0});
  ASSERT_TRUE(truncation.applied);
  std::vector<TrajectoryPointSample> suffix = trajectoryPointSamplesFromPoints(
      std::vector<Point2>{{60.0, 0.0}, {60.0, 20.0}, {80.0, 20.0}});
  assignTrajectorySampleAltitude(suffix, 18.0);

  const TruncatedPrefixStitchResult stitched = stitchTruncatedPrefixWithSuffix(
      truncation.samples, suffix,
      TruncatedPrefixStitchRequest{.current_position = Point2{30.0, 0.0},
                                   .truncation_point = Point2{60.0, 0.0},
                                   .max_join_distance_m = 0.5});

  ASSERT_TRUE(stitched.applied) << stitched.reason;
  EXPECT_STREQ(stitched.reason, "prefix_suffix_stitched");
  EXPECT_NEAR(stitched.samples.front().point.x, 30.0, 1.0e-6);
  EXPECT_NEAR(stitched.samples.back().point.x, 80.0, 1.0e-6);
  EXPECT_NEAR(stitched.samples.back().point.y, 20.0, 1.0e-6);
  EXPECT_NEAR(stitched.suffix_station_offset_m, 30.0, 1.0e-6);
  EXPECT_NEAR(stitched.samples.back().s_m, 70.0, 1.0e-6);
}

TEST(SafeTrajectoryTruncation, RejectsSuffixThatDoesNotStartAtTruncationPoint) {
  const std::vector<TrajectoryPointSample> prefix = lineSamples();
  std::vector<TrajectoryPointSample> suffix =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{80.0, 5.0}, {90.0, 5.0}});
  assignTrajectorySampleAltitude(suffix, 18.0);

  const TruncatedPrefixStitchResult stitched = stitchTruncatedPrefixWithSuffix(
      prefix, suffix,
      TruncatedPrefixStitchRequest{.current_position = Point2{20.0, 0.0},
                                   .truncation_point = Point2{60.0, 0.0},
                                   .max_join_distance_m = 0.5});

  EXPECT_FALSE(stitched.applied);
  EXPECT_STREQ(stitched.reason, "join_point_mismatch");
}

TEST(SafeTrajectoryTruncation, FingerprintChangesWithPrefixGeometry) {
  std::vector<TrajectoryPointSample> first = lineSamples();
  std::vector<TrajectoryPointSample> second = first;
  const std::uint64_t first_fingerprint = trajectoryPrefixFingerprint(first);
  EXPECT_EQ(first_fingerprint, trajectoryPrefixFingerprint(first));
  second.back().point.y = 1.0;
  EXPECT_NE(first_fingerprint, trajectoryPrefixFingerprint(second));
}

TEST(SafeTrajectoryTruncation, AcceptsCompatibleSuffixJoin) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 18.0;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.point.x += 0.2;
  suffix_initial.tangent = Point2{0.98, 0.2};
  suffix_initial.z_m += 0.1;

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial,
      TruncationSuffixJoinRequest{.max_position_jump_m = 0.5,
                                  .max_tangent_jump_rad = 0.3,
                                  .max_altitude_jump_m = 0.2});

  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_STREQ(validation.reason, "join_valid");
  EXPECT_NEAR(validation.position_jump_m, 0.2, 1.0e-9);
  EXPECT_LT(validation.tangent_jump_rad, 0.3);
  EXPECT_NEAR(validation.altitude_jump_m, 0.1, 1.0e-9);
}

TEST(SafeTrajectoryTruncation, RejectsSuffixJoinPositionMismatch) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 18.0;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.point.x += 1.1;

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial, TruncationSuffixJoinRequest{});

  EXPECT_FALSE(validation.valid);
  EXPECT_STREQ(validation.reason, "position_mismatch");
}

TEST(SafeTrajectoryTruncation, RejectsSuffixJoinTangentMismatch) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 18.0;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.tangent = Point2{-1.0, 0.0};

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial,
      TruncationSuffixJoinRequest{.max_position_jump_m = 1.0,
                                  .max_tangent_jump_rad = 1.57,
                                  .max_altitude_jump_m = 0.4});

  EXPECT_FALSE(validation.valid);
  EXPECT_STREQ(validation.reason, "tangent_mismatch");
}

TEST(SafeTrajectoryTruncation, AcceptsAfterHoldSuffixWithoutTangentMatch) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 18.0;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.tangent = Point2{-1.0, 0.0};

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial,
      TruncationSuffixJoinRequest{.max_position_jump_m = 1.0,
                                  .max_tangent_jump_rad = 0.01,
                                  .max_altitude_jump_m = 0.4,
                                  .require_tangent_match = false});

  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_STREQ(validation.reason, "join_valid");
  EXPECT_TRUE(std::isnan(validation.tangent_jump_rad));
}

TEST(SafeTrajectoryTruncation, RejectsSuffixJoinAltitudeMismatch) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 18.0;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.z_m = 18.5;

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial, TruncationSuffixJoinRequest{});

  EXPECT_FALSE(validation.valid);
  EXPECT_STREQ(validation.reason, "altitude_mismatch");
}

TEST(SafeTrajectoryTruncation, AcceptsPreAlignedSuffixWithoutAltitudeMatch) {
  TrajectoryPointSample prefix_terminal;
  prefix_terminal.point = Point2{10.0, 20.0};
  prefix_terminal.tangent = Point2{1.0, 0.0};
  prefix_terminal.z_m = 23.5;
  TrajectoryPointSample suffix_initial = prefix_terminal;
  suffix_initial.z_m = 6.5;

  const TruncationSuffixJoinValidation validation = validateTruncationSuffixJoin(
      prefix_terminal, suffix_initial,
      TruncationSuffixJoinRequest{.max_position_jump_m = 1.0,
                                  .max_tangent_jump_rad = 0.01,
                                  .max_altitude_jump_m = 0.3,
                                  .require_tangent_match = false,
                                  .require_altitude_match = false});

  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_NEAR(validation.altitude_jump_m, 17.0, 1.0e-9);
}

} // namespace drone_city_nav

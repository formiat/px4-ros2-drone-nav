#include "drone_city_nav/trajectory_update_continuity.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromPoints(const std::vector<Point2>& points) {
  return trajectoryPointSamplesFromPoints(points);
}

[[nodiscard]] TrajectorySpeedProfile
profileForSamples(const std::vector<TrajectoryPointSample>& samples) {
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 12.0;
  config.speed_profile_sample_step_m = 1.0;
  return buildTrajectorySpeedProfile(samples, config);
}

} // namespace

TEST(TrajectoryUpdateContinuity, RejectsInvalidNewTrajectory) {
  const std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  const TrajectorySpeedProfile old_profile = profileForSamples(old_samples);
  const std::vector<TrajectoryPointSample> new_samples;
  const TrajectorySpeedProfile new_profile;

  const TrajectoryContinuityResult result =
      evaluateTrajectoryContinuity(old_samples, old_profile, new_samples, new_profile,
                                   Point2{2.0, 0.0}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(result.reason, "new_trajectory_invalid");
}

TEST(TrajectoryUpdateContinuity, PreservesSmootherForCompatibleTrajectory) {
  const std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 0.2}, Point2{40.0, 0.2}});

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.1}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kPreserveSmoother);
  EXPECT_STREQ(result.reason, "compatible");
  EXPECT_LT(result.projection_jump_m, 3.0);
  EXPECT_LT(result.tangent_speed_command_jump_mps, 8.0);
}

TEST(TrajectoryUpdateContinuity, ResetsSmootherForModerateProjectionJump) {
  const std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 4.0}, Point2{40.0, 4.0}});

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.0}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kResetSmoother);
  EXPECT_STREQ(result.reason, "moderate_discontinuity");
  EXPECT_GT(result.projection_jump_m, 3.0);
  EXPECT_LT(result.projection_jump_m, 8.0);
  EXPECT_TRUE(result.preserve_vertical_smoother_state);
}

TEST(TrajectoryUpdateContinuity, RejectsLargeProjectionJump) {
  const std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 12.0}, Point2{40.0, 12.0}});

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.0}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(result.reason, "control_discontinuity");
  EXPECT_GT(result.projection_jump_m, 8.0);
}

TEST(TrajectoryUpdateContinuity, RejectsLargeTangentSpeedCommandJump) {
  const std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  TrajectoryContinuityThresholds thresholds{};
  thresholds.reject_tangent_speed_command_jump_mps = 3.0;

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.0}, Point2{0.0, 0.0}, true,
      thresholds);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(result.reason, "control_discontinuity");
  EXPECT_GT(result.tangent_speed_command_jump_mps, 3.0);
}

TEST(TrajectoryUpdateContinuity, ResetsWhenNoPreviousTrajectoryExists) {
  const std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const std::vector<TrajectoryPointSample> old_samples;
  const TrajectorySpeedProfile old_profile;

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, old_profile, new_samples, profileForSamples(new_samples),
      Point2{8.0, 0.0}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kResetSmoother);
  EXPECT_STREQ(result.reason, "no_previous_trajectory");
}

TEST(TrajectoryUpdateContinuity, ResetsVerticalSmootherForTargetAltitudeJump) {
  std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  std::vector<TrajectoryPointSample> new_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  assignTrajectorySampleAltitude(old_samples, 10.0);
  assignTrajectorySampleAltitude(new_samples, 15.0);

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.0}, Point2{12.0, 0.0}, true);

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kResetSmoother);
  EXPECT_STREQ(result.reason, "vertical_discontinuity");
  EXPECT_DOUBLE_EQ(result.vertical_target_z_jump_m, 5.0);
  EXPECT_FALSE(result.preserve_vertical_smoother_state);
}

TEST(TrajectoryUpdateContinuity, RejectsUnsafeImmediateVerticalHardWindow) {
  std::vector<TrajectoryPointSample> old_samples =
      samplesFromPoints({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  std::vector<TrajectoryPointSample> new_samples = old_samples;
  assignTrajectorySampleAltitude(old_samples, 12.0);
  assignTrajectorySampleAltitude(new_samples, 8.0);
  for (TrajectoryPointSample& sample : new_samples) {
    sample.vertical_hard_window_active = true;
    sample.vertical_safe_min_z_m = 6.0;
    sample.vertical_safe_max_z_m = 8.0;
    sample.vertical_gate_z_m = 7.0;
    sample.vertical_profile_passage_id = "low_opening";
  }

  const TrajectoryContinuityResult result = evaluateTrajectoryContinuity(
      old_samples, profileForSamples(old_samples), new_samples,
      profileForSamples(new_samples), Point2{8.0, 0.0}, Point2{12.0, 0.0}, true,
      TrajectoryContinuityThresholds{},
      TrajectoryVerticalContinuityState{.current_altitude_m = 12.0,
                                        .altitude_valid = true});

  EXPECT_EQ(result.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(result.reason, "vertical_hard_window_unsafe");
  EXPECT_TRUE(result.vertical_hard_window_unsafe);
  EXPECT_FALSE(result.preserve_vertical_smoother_state);
}

} // namespace drone_city_nav

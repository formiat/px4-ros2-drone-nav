#include "drone_city_nav/offboard_vertical_follower.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample> rampSamples() {
  std::vector<TrajectoryPointSample> samples(3U);
  samples[0].s_m = 0.0;
  samples[0].point = Point2{0.0, 0.0};
  samples[0].tangent = Point2{1.0, 0.0};
  samples[0].z_m = 10.0;
  samples[0].vertical_slope_dz_ds = 0.1;
  samples[1].s_m = 10.0;
  samples[1].point = Point2{10.0, 0.0};
  samples[1].tangent = Point2{1.0, 0.0};
  samples[1].z_m = 11.0;
  samples[1].vertical_slope_dz_ds = 0.1;
  samples[2].s_m = 20.0;
  samples[2].point = Point2{20.0, 0.0};
  samples[2].tangent = Point2{1.0, 0.0};
  samples[2].z_m = 12.0;
  samples[2].vertical_slope_dz_ds = 0.1;
  return samples;
}

} // namespace

TEST(OffboardVerticalFollower, ConstantAltitudeCommandsZeroWhenOnTarget) {
  std::vector<TrajectoryPointSample> samples = rampSamples();
  for (TrajectoryPointSample& sample : samples) {
    sample.z_m = 18.0;
    sample.vertical_slope_dz_ds = 0.0;
  }

  const VerticalSetpointPlan plan =
      planVerticalSetpoint(samples, 5.0, 10.0, 18.0, true, 0.1, VerticalFollowerState{},
                           VerticalFollowerConfig{});

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.trajectory_target_valid);
  EXPECT_FALSE(plan.passage_mode);
  EXPECT_NEAR(plan.target_z_m, 18.0, 1.0e-9);
  EXPECT_NEAR(plan.z_error_m, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.target_vz_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_ned_mps, 0.0, 1.0e-9);
}

TEST(OffboardVerticalFollower, RampProfileAddsTargetVerticalVelocity) {
  const std::vector<TrajectoryPointSample> samples = rampSamples();

  const VerticalSetpointPlan plan =
      planVerticalSetpoint(samples, 5.0, 10.0, 10.5, true, 0.1, VerticalFollowerState{},
                           VerticalFollowerConfig{});

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.target_z_m, 10.5, 1.0e-9);
  EXPECT_NEAR(plan.vertical_slope_dz_ds, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.target_vz_mps, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.feedback_vz_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_mps, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_ned_mps, -1.0, 1.0e-9);
}

TEST(OffboardVerticalFollower, AltitudeFeedbackAddsToFeedforwardAndClamps) {
  VerticalFollowerConfig config;
  config.altitude_feedback_kp_1ps = 1.0;
  config.max_vertical_speed_mps = 1.25;
  const std::vector<TrajectoryPointSample> samples = rampSamples();

  const VerticalSetpointPlan plan = planVerticalSetpoint(
      samples, 5.0, 10.0, 8.5, true, 0.1, VerticalFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.target_z_m, 10.5, 1.0e-9);
  EXPECT_NEAR(plan.z_error_m, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.target_vz_mps, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.feedback_vz_mps, 1.25, 1.0e-9);
  EXPECT_NEAR(plan.desired_vz_mps, 1.25, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_mps, 1.25, 1.0e-9);
}

TEST(OffboardVerticalFollower, LimitsVerticalAccelAndJerkAfterFirstCommand) {
  VerticalFollowerConfig config;
  config.max_vertical_speed_mps = 10.0;
  config.max_vertical_accel_mps2 = 2.0;
  config.max_vertical_jerk_mps3 = 5.0;
  config.altitude_feedback_kp_1ps = 2.0;
  VerticalFollowerState state;
  state.previous_command_valid = true;
  state.previous_commanded_vz_mps = 0.0;
  state.previous_vertical_accel_mps2 = 0.0;
  const std::vector<TrajectoryPointSample> samples = rampSamples();

  const VerticalSetpointPlan plan =
      planVerticalSetpoint(samples, 5.0, 0.0, 0.0, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.desired_vz_mps, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_mps, 0.05, 1.0e-9);
  EXPECT_NEAR(plan.vertical_accel_mps2, 0.5, 1.0e-9);
  EXPECT_NEAR(plan.vertical_jerk_mps3, 5.0, 1.0e-9);
}

TEST(OffboardVerticalFollower, InvalidAltitudeFailsSafeWithZeroCommand) {
  const std::vector<TrajectoryPointSample> samples = rampSamples();

  const VerticalSetpointPlan plan = planVerticalSetpoint(
      samples, 5.0, 10.0, std::numeric_limits<double>::quiet_NaN(), false, 0.1,
      VerticalFollowerState{}, VerticalFollowerConfig{});

  EXPECT_FALSE(plan.valid);
  EXPECT_FALSE(plan.trajectory_target_valid);
  EXPECT_EQ(plan.reason, "invalid_altitude");
  EXPECT_NEAR(plan.commanded_vz_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.commanded_vz_ned_mps, 0.0, 1.0e-9);
}

TEST(OffboardVerticalFollower, PassageModeUsesVerticalConstraintMetadata) {
  std::vector<TrajectoryPointSample> samples = rampSamples();
  samples[1].vertical_constraint_active = true;
  samples[1].vertical_profile_passage_id = "arch_main";

  const VerticalSetpointPlan plan =
      planVerticalSetpoint(samples, 9.0, 5.0, 10.9, true, 0.1, VerticalFollowerState{},
                           VerticalFollowerConfig{});

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.passage_mode);
  EXPECT_TRUE(plan.vertical_constraint_active);
  EXPECT_EQ(plan.passage_id, "arch_main");
}

} // namespace drone_city_nav

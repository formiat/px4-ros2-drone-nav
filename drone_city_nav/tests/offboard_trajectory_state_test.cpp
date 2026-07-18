#include "drone_city_nav/offboard_trajectory_state.hpp"

#include <nav_msgs/msg/path.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] geometry_msgs::msg::PoseStamped
makePose(const double x_m, const double y_m, const double z_m) {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x_m;
  pose.pose.position.y = y_m;
  pose.pose.position.z = z_m;
  return pose;
}

[[nodiscard]] nav_msgs::msg::Path makePath() {
  nav_msgs::msg::Path path;
  path.header.stamp.sec = 2;
  path.header.stamp.nanosec = 3;
  path.poses = {makePose(0.0, 0.0, 10.0), makePose(4.0, 0.0, 12.0),
                makePose(4.0, 3.0, 14.0)};
  return path;
}

} // namespace

TEST(OffboardTrajectoryState, ConvertsPathAndBuildsExecutableState) {
  const nav_msgs::msg::Path path = makePath();

  const std::vector<Point2> points = pathPointsFromMessage(path);
  const std::vector<TrajectoryPointSample> samples = pathSamplesFromMessage(path);
  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(samples, VelocityFollowerConfig{});

  ASSERT_EQ(points.size(), 3U);
  ASSERT_EQ(samples.size(), 3U);
  EXPECT_EQ(messageStampNanoseconds(path.header.stamp), 2'000'000'003U);
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.samples.size(), 3U);
  EXPECT_DOUBLE_EQ(state.samples[0].z_m, 10.0);
  EXPECT_DOUBLE_EQ(state.samples[1].z_m, 12.0);
  EXPECT_DOUBLE_EQ(state.samples[2].z_m, 14.0);
  EXPECT_EQ(state.trajectory.size(), 2U);
  EXPECT_EQ(state.stats.input_points, 3U);
  EXPECT_EQ(state.stats.samples, 3U);
  EXPECT_DOUBLE_EQ(state.metrics.length_m, 7.0);
}

TEST(OffboardTrajectoryState, EmptyPathBuildsInvalidState) {
  const std::vector<Point2> points;

  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(points, VelocityFollowerConfig{});

  EXPECT_FALSE(state.valid);
  EXPECT_TRUE(state.samples.empty());
  EXPECT_TRUE(state.trajectory.empty());
  EXPECT_EQ(state.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
}

TEST(OffboardTrajectoryState, PreservesAltitudeWhenCompactingDuplicateInteriorPose) {
  nav_msgs::msg::Path path;
  path.poses = {makePose(0.0, 0.0, 10.0), makePose(4.0, 0.0, 12.0),
                makePose(4.0, 0.0, 13.0), makePose(8.0, 0.0, 14.0)};

  const std::vector<TrajectoryPointSample> samples = pathSamplesFromMessage(path);
  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(samples, VelocityFollowerConfig{});

  ASSERT_EQ(samples.size(), 3U);
  EXPECT_TRUE(state.valid);
  EXPECT_DOUBLE_EQ(samples[0].z_m, 10.0);
  EXPECT_DOUBLE_EQ(samples[1].z_m, 12.0);
  EXPECT_DOUBLE_EQ(samples[2].z_m, 14.0);
  EXPECT_DOUBLE_EQ(state.metrics.length_m, 8.0);
}

TEST(OffboardTrajectoryState, RebuildsVerticalSpeedConstraintsFromPathAltitude) {
  nav_msgs::msg::Path path;
  path.poses = {makePose(0.0, 0.0, 10.0), makePose(5.0, 0.0, 15.0),
                makePose(10.0, 0.0, 20.0)};
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 20.0;
  config.vertical_profile_max_climb_speed_mps = 2.0;
  config.vertical_profile_max_descent_speed_mps = 2.0;

  const std::vector<TrajectoryPointSample> samples = pathSamplesFromMessage(path);
  const OffboardTrajectoryState state = buildOffboardTrajectoryState(samples, config);

  ASSERT_TRUE(state.valid);
  ASSERT_EQ(state.samples.size(), 3U);
  EXPECT_TRUE(state.samples[1].vertical_constraint_active);
  EXPECT_NEAR(state.samples[1].vertical_slope_dz_ds, 1.0, 1.0e-9);
  const TrajectorySpeedSample speed_sample =
      speedProfileSampleAtS(state.speed_profile, state.samples[1].s_m);
  EXPECT_EQ(speed_sample.reason, SpeedConstraintType::kVerticalProfile);
  EXPECT_NEAR(speed_sample.geometric_limit_mps, 2.0, 1.0e-9);
}

TEST(OffboardTrajectoryState, AppliesPlannerVerticalProfileHardWindowMetadata) {
  nav_msgs::msg::Path path;
  path.poses = {makePose(0.0, 0.0, 18.0), makePose(10.0, 0.0, 10.0),
                makePose(20.0, 0.0, 10.0), makePose(30.0, 0.0, 10.0)};
  TrajectoryPlannerStats planner_stats;
  planner_stats.vertical_profile.diagnostics.push_back(VerticalProfilePassageDiagnostic{
      .structure_id = "arch_01",
      .opening_id = "low_window",
      .entry_s_m = 12.0,
      .exit_s_m = 20.0,
      .approach_start_s_m = 4.0,
      .gate_hold_start_s_m = 10.0,
      .exit_end_s_m = 20.0,
      .gate_z_m = 10.0,
      .min_z_m = 7.5,
      .max_z_m = 10.5,
      .safe_min_z_m = 8.0,
      .safe_max_z_m = 10.0,
      .reason = "profiled",
      .valid = true,
  });

  const std::vector<TrajectoryPointSample> samples = pathSamplesFromMessage(path);
  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(samples, VelocityFollowerConfig{}, &planner_stats);

  ASSERT_TRUE(state.valid);
  ASSERT_EQ(state.samples.size(), 4U);
  EXPECT_TRUE(state.samples[1].vertical_hard_window_active);
  EXPECT_TRUE(state.samples[1].vertical_constraint_active);
  EXPECT_EQ(state.samples[1].vertical_profile_passage_id, "low_window");
  EXPECT_DOUBLE_EQ(state.samples[1].vertical_safe_min_z_m, 8.0);
  EXPECT_DOUBLE_EQ(state.samples[1].vertical_safe_max_z_m, 10.0);
  EXPECT_DOUBLE_EQ(state.samples[1].vertical_gate_z_m, 10.0);
  EXPECT_FALSE(state.samples[3].vertical_hard_window_active);
}

TEST(OffboardTrajectoryState, RejectsNonFiniteAltitudeBeforeStalePoseReset) {
  nav_msgs::msg::Path invalid_path = makePath();
  invalid_path.poses[1].pose.position.z = std::numeric_limits<double>::quiet_NaN();
  const std::vector<TrajectoryPointSample> invalid_samples =
      pathSamplesFromMessage(invalid_path);
  const OffboardTrajectoryState invalid_candidate =
      buildOffboardTrajectoryState(invalid_samples, VelocityFollowerConfig{});
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_FALSE(invalid_candidate.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, invalid_candidate,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(continuity.reason, "new_trajectory_invalid");
}

TEST(OffboardTrajectoryState, RejectsInvalidCandidateBeforeStalePoseReset) {
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const std::vector<Point2> invalid_points{{10.0, 0.0}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});
  const OffboardTrajectoryState invalid_candidate =
      buildOffboardTrajectoryState(invalid_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_FALSE(invalid_candidate.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, invalid_candidate,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(continuity.reason, "new_trajectory_invalid");
}

TEST(OffboardTrajectoryState, ResetsForValidCandidateWhenPoseIsStale) {
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const std::vector<Point2> candidate_points{{0.0, 0.2}, {20.0, 0.2}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});
  const OffboardTrajectoryState candidate_state =
      buildOffboardTrajectoryState(candidate_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_TRUE(candidate_state.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, candidate_state,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kResetSmoother);
  EXPECT_STREQ(continuity.reason, "pose_stale");
}

TEST(OffboardTrajectoryState, MatchesAndMergesPlannerDiagnostics) {
  TrajectoryPlannerDiagnosticsEnvelope diagnostics;
  diagnostics.path_stamp_ns = 100U;
  diagnostics.planner_path_id = 9U;
  diagnostics.stats.corridor.mean_width_m = 12.0;
  diagnostics.stats.trajectory_optimizer.final_length_m = 34.0;
  diagnostics.stats.turn_smoothing.smoothed_corners = 2U;
  diagnostics.stats.total_duration_ms = 56.0;
  diagnostics.stats.speed_profile_duration_ms = 99.0;
  diagnostics.stats.speed_profile_min_mps = 1.0;
  diagnostics.stats.speed_profile_construction_config_fingerprint = 10U;
  diagnostics.stats.runtime_speed_policy_config_fingerprint = 11U;
  diagnostics.stats.runtime_velocity_control_config_fingerprint = 12U;
  diagnostics.stats.known_passage_validation.enabled = true;
  diagnostics.stats.known_passage_validation.valid = false;
  diagnostics.stats.known_passage_validation.structures_checked = 3U;
  diagnostics.stats.known_passage_validation.structures_intersected = 1U;
  diagnostics.stats.known_passage_validation.opening_matches = 0U;
  diagnostics.stats.known_passage_validation.violations = 1U;
  diagnostics.stats.known_passage_validation.worst_reason =
      KnownPassageValidationReason::kOpeningVolumeMiss;
  diagnostics.stats.known_passage_validation.diagnostics.push_back(
      KnownPassageValidationSpan{
          .structure_id = "arch_01",
          .opening_id = "",
          .entry_s_m = 42.0,
          .exit_s_m = 47.0,
          .overlap_m = 0.0,
          .clearance_m = -0.5,
          .reason = KnownPassageValidationReason::kOpeningVolumeMiss,
          .valid = false,
      });
  diagnostics.stats.top_speed_constraints.push_back(SpeedProfileConstraintDiagnostic{
      .sample_index = 7U,
      .s_m = 11.0,
      .radius_m = 12.0,
      .curvature_1pm = 0.08,
      .speed_limit_mps = 6.0,
      .profiled_limit_mps = 6.0,
      .source = SpeedConstraintType::kArc,
      .isolated_curvature_spike = false,
  });

  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 101U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 10U));
  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, false, 10U));

  TrajectoryPlannerStats stats;
  stats.speed_profile_duration_ms = 1.5;
  stats.speed_profile_min_mps = 4.0;
  stats.speed_profile_construction_config_fingerprint = 20U;
  stats.runtime_speed_policy_config_fingerprint = 21U;
  stats.runtime_velocity_control_config_fingerprint = 22U;
  mergePlannerDiagnosticsIntoTrajectoryStats(stats, diagnostics);

  EXPECT_DOUBLE_EQ(stats.corridor.mean_width_m, 12.0);
  EXPECT_DOUBLE_EQ(stats.trajectory_optimizer.final_length_m, 34.0);
  EXPECT_EQ(stats.turn_smoothing.smoothed_corners, 2U);
  EXPECT_DOUBLE_EQ(stats.total_duration_ms, 56.0);
  EXPECT_DOUBLE_EQ(stats.speed_profile_duration_ms, 1.5);
  EXPECT_DOUBLE_EQ(stats.speed_profile_min_mps, 4.0);
  EXPECT_EQ(stats.speed_profile_construction_config_fingerprint, 20U);
  EXPECT_EQ(stats.runtime_speed_policy_config_fingerprint, 21U);
  EXPECT_EQ(stats.runtime_velocity_control_config_fingerprint, 22U);
  EXPECT_TRUE(stats.known_passage_validation.enabled);
  EXPECT_FALSE(stats.known_passage_validation.valid);
  EXPECT_EQ(stats.known_passage_validation.structures_checked, 3U);
  EXPECT_EQ(stats.known_passage_validation.structures_intersected, 1U);
  EXPECT_EQ(stats.known_passage_validation.opening_matches, 0U);
  EXPECT_EQ(stats.known_passage_validation.violations, 1U);
  EXPECT_EQ(stats.known_passage_validation.worst_reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
  ASSERT_EQ(stats.known_passage_validation.diagnostics.size(), 1U);
  EXPECT_EQ(stats.known_passage_validation.diagnostics.front().structure_id, "arch_01");
  EXPECT_DOUBLE_EQ(stats.known_passage_validation.diagnostics.front().entry_s_m, 42.0);
  EXPECT_DOUBLE_EQ(stats.known_passage_validation.diagnostics.front().clearance_m,
                   -0.5);
  EXPECT_TRUE(stats.top_speed_constraints.empty());
}

} // namespace drone_city_nav

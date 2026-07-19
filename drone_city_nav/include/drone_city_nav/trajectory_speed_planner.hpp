#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class SpeedConstraintType {
  kNone,
  kArc,
  kVerticalProfile,
  kVerticalTrackability,
  kNoStaticObservation,
  kGoal,
};

struct TrajectorySpeedSample {
  double s_m{0.0};
  double geometric_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double profiled_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType reason{SpeedConstraintType::kNone};
  std::size_t segment_index{0U};
  double curvature_1pm{0.0};
  double radius_m{std::numeric_limits<double>::quiet_NaN()};
  double constraint_s_m{std::numeric_limits<double>::quiet_NaN()};
  double constraint_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double vertical_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double vertical_slope_dz_ds{0.0};
  double vertical_accel_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double vertical_jerk_limit_mps{std::numeric_limits<double>::quiet_NaN()};
};

struct TrajectorySpeedProfile {
  std::vector<TrajectorySpeedSample> samples;
  bool valid{false};
};

struct SpeedProfileConstraintDiagnostic {
  std::size_t sample_index{0U};
  double s_m{std::numeric_limits<double>::quiet_NaN()};
  double radius_m{std::numeric_limits<double>::quiet_NaN()};
  double curvature_1pm{0.0};
  double speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double profiled_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType source{SpeedConstraintType::kNone};
  bool isolated_curvature_spike{false};
};

struct TraversalTimeEstimate {
  bool valid{false};
  double estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t curvature_limited_samples{0U};
};

struct ScalarSpeedQuery {
  double trajectory_s_m{std::numeric_limits<double>::quiet_NaN()};
  double previous_command_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double dt_s{std::numeric_limits<double>::quiet_NaN()};
  bool vertical_trackability_speed_cap_active{false};
  double vertical_trackability_speed_limit_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double vertical_trackability_constraint_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double vertical_trackability_altitude_error_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool no_static_speed_cap_active{false};
  double no_static_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double no_static_boundary_distance_m{std::numeric_limits<double>::quiet_NaN()};
};

struct ScalarSpeedPlan {
  bool valid{false};
  SpeedConstraintType constraint_type{SpeedConstraintType::kNone};
  std::size_t constraint_index{0U};
  double profile_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double lookahead_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double lookahead_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  SpeedConstraintType lookahead_constraint_type{SpeedConstraintType::kNone};
  std::size_t lookahead_constraint_index{0U};
  double lookahead_constraint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double speed_after_lookahead_mps{std::numeric_limits<double>::quiet_NaN()};
  double accel_limited_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double final_scalar_speed_mps{0.0};
  double limiting_constraint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double limiting_curve_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double limiting_constraint_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double limiting_allowed_speed_now_mps{std::numeric_limits<double>::quiet_NaN()};
  double limiting_curvature_1pm{0.0};
  bool vertical_trackability_speed_cap_active{false};
  double vertical_trackability_speed_limit_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double vertical_trackability_constraint_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double vertical_trackability_altitude_error_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool no_static_speed_cap_active{false};
  double no_static_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double no_static_boundary_distance_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] const char*
speedConstraintTypeName(SpeedConstraintType constraint_type) noexcept;

[[nodiscard]] double
distanceFromTrajectorySToEnd(std::span<const TrajectorySegment> trajectory, double s_m);

[[nodiscard]] TrajectorySpeedProfile
buildTrajectorySpeedProfile(std::span<const TrajectorySegment> trajectory,
                            const VelocityFollowerConfig& config);

[[nodiscard]] TrajectorySpeedProfile
buildTrajectorySpeedProfile(std::span<const TrajectoryPointSample> trajectory_samples,
                            const VelocityFollowerConfig& config);

void populateTrajectoryVerticalSpeedConstraints(
    std::span<TrajectoryPointSample> trajectory_samples,
    const VelocityFollowerConfig& config);

[[nodiscard]] TrajectorySpeedSample
speedProfileSampleAtS(const TrajectorySpeedProfile& profile, double s_m);

[[nodiscard]] std::vector<SpeedProfileConstraintDiagnostic>
topSpeedProfileConstraints(const TrajectorySpeedProfile& profile,
                           std::size_t max_constraints);

[[nodiscard]] ScalarSpeedPlan planScalarSpeed(const TrajectorySpeedProfile& profile,
                                              const ScalarSpeedQuery& query,
                                              const VelocityFollowerConfig& config);

[[nodiscard]] TraversalTimeEstimate
estimateTraversalTime(std::span<const TrajectoryPointSample> trajectory_samples,
                      const VelocityFollowerConfig& config,
                      bool use_forward_backward_profile);

} // namespace drone_city_nav

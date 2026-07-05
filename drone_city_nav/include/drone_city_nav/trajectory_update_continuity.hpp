#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/types.hpp"

#include <limits>
#include <span>

namespace drone_city_nav {

enum class TrajectoryContinuityDecision {
  kPreserveSmoother,
  kResetSmoother,
  kRejectTrajectory,
};

struct TrajectoryContinuityThresholds {
  double preserve_projection_jump_m{3.0};
  double preserve_tangent_jump_rad{0.52};
  double preserve_curvature_jump_1pm{0.05};
  double preserve_speed_limit_jump_mps{6.0};
  double preserve_command_jump_mps{8.0};
  double reject_projection_jump_m{8.0};
  double reject_tangent_jump_rad{1.57};
  double reject_command_jump_mps{15.0};
};

struct TrajectoryContinuityResult {
  TrajectoryContinuityDecision decision{TrajectoryContinuityDecision::kResetSmoother};
  const char* reason{"no_previous_trajectory"};
  bool old_projection_valid{false};
  bool new_projection_valid{false};
  double projection_jump_m{std::numeric_limits<double>::quiet_NaN()};
  double tangent_jump_rad{std::numeric_limits<double>::quiet_NaN()};
  double curvature_jump_1pm{std::numeric_limits<double>::quiet_NaN()};
  double speed_limit_jump_mps{std::numeric_limits<double>::quiet_NaN()};
  double command_jump_mps{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] const char*
trajectoryContinuityDecisionName(TrajectoryContinuityDecision decision) noexcept;

[[nodiscard]] TrajectoryContinuityResult
evaluateTrajectoryContinuity(std::span<const TrajectoryPointSample> old_samples,
                             const TrajectorySpeedProfile& old_speed_profile,
                             std::span<const TrajectoryPointSample> new_samples,
                             const TrajectorySpeedProfile& new_speed_profile,
                             Point2 current_position, Point2 previous_velocity_setpoint,
                             bool previous_velocity_setpoint_valid,
                             const TrajectoryContinuityThresholds& thresholds = {});

} // namespace drone_city_nav

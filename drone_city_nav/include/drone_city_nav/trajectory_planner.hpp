#pragma once

#include "drone_city_nav/corner_rounding.hpp"
#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/racing_line.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class TrajectoryPlannerFallbackReason {
  kNone,
  kInvalidRoute,
  kMissingGrid,
  kRacingDisabled,
  kCorridorInvalid,
  kRacingLineInvalid,
  kBaselineInvalid,
};

enum class TrajectoryGridRebuildReason {
  kNone,
  kInvalidTrajectory,
  kMissingGridFallback,
  kProhibitedIntersection,
  kCorridorBoundsChanged,
};

struct TrajectoryPlannerConfig {
  bool racing_trajectory_enabled{true};
  CornerRoundingConfig baseline_rounding{};
  CorridorConfig corridor{};
  RacingLineConfig racing_line{};
  VelocityFollowerConfig speed_profile{};
  double debug_sample_step_m{1.0};
};

struct TrajectoryPlannerStats {
  std::size_t input_points{0U};
  std::size_t compact_segments{0U};
  std::size_t line_segments{0U};
  std::size_t arc_segments{0U};
  std::size_t samples{0U};
  double length_m{0.0};
  double curvature_min_1pm{0.0};
  double curvature_max_1pm{0.0};
  double curvature_mean_abs_1pm{0.0};
  double speed_profile_min_mps{0.0};
  double speed_profile_max_mps{0.0};
  double speed_profile_mean_mps{0.0};
  std::size_t speed_profile_curvature_limited_samples{0U};
  TrajectoryPlannerFallbackReason fallback_reason{
      TrajectoryPlannerFallbackReason::kNone};
  CornerRoundingStats baseline_rounding{};
  CorridorStats corridor{};
  RacingLineStats racing_line{};
};

struct TrajectoryPlannerInput {
  std::span<const Point2> route_points;
  const OccupancyGrid2D* prohibited_grid{nullptr};
};

struct TrajectoryGridRebuildDecisionInput {
  bool trajectory_valid{false};
  bool racing_trajectory_enabled{true};
  bool final_trajectory_intersects_prohibited{false};
  bool current_corridor_valid{false};
  double corridor_width_threshold_m{0.5};
  TrajectoryPlannerFallbackReason fallback_reason{
      TrajectoryPlannerFallbackReason::kNone};
  CorridorStats previous_corridor{};
  CorridorStats current_corridor{};
};

struct TrajectoryPlannerResult {
  std::vector<TrajectorySegment> compact_segments;
  std::vector<TrajectoryPointSample> samples;
  TrajectorySpeedProfile speed_profile;
  TrajectoryPlannerStats stats{};
  bool valid{false};
};

[[nodiscard]] std::string_view
trajectoryPlannerFallbackReasonName(TrajectoryPlannerFallbackReason reason) noexcept;

[[nodiscard]] std::string_view
trajectoryGridRebuildReasonName(TrajectoryGridRebuildReason reason) noexcept;

[[nodiscard]] TrajectoryGridRebuildReason
trajectoryGridRebuildReason(const TrajectoryGridRebuildDecisionInput& input) noexcept;

[[nodiscard]] TrajectoryPlannerResult
planTrajectory(const TrajectoryPlannerInput& input,
               const TrajectoryPlannerConfig& config);

} // namespace drone_city_nav

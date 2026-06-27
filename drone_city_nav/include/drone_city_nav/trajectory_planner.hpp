#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/racing_line.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/turn_smoothing.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class TrajectoryPlannerStatus {
  kOk,
  kInvalidRoute,
  kMissingGrid,
  kCorridorInvalid,
  kRacingLineInvalid,
  kInvalidTrajectory,
};

struct TrajectoryPlannerConfig {
  CorridorConfig corridor{};
  RacingLineConfig racing_line{};
  TurnSmoothingConfig turn_smoothing{};
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
  double total_duration_ms{0.0};
  double corridor_duration_ms{0.0};
  double racing_line_duration_ms{0.0};
  double turn_smoothing_duration_ms{0.0};
  double speed_profile_duration_ms{0.0};
  TrajectoryPlannerStatus status{TrajectoryPlannerStatus::kOk};
  CorridorStats corridor{};
  RacingLineStats racing_line{};
  TurnSmoothingStats turn_smoothing{};
};

struct TrajectoryPlannerInput {
  std::span<const Point2> route_points;
  const OccupancyGrid2D* prohibited_grid{nullptr};
};

struct TrajectoryPlannerResult {
  std::vector<TrajectorySegment> compact_segments;
  std::vector<CorridorSample> corridor_samples;
  std::vector<TrajectoryPointSample> samples;
  TrajectorySpeedProfile speed_profile;
  TrajectoryPlannerStats stats{};
  bool valid{false};
};

[[nodiscard]] std::string_view
trajectoryPlannerStatusName(TrajectoryPlannerStatus status) noexcept;

[[nodiscard]] TrajectoryPlannerResult
planTrajectory(const TrajectoryPlannerInput& input,
               const TrajectoryPlannerConfig& config);

[[nodiscard]] TrajectoryPlannerResult
planRacingTrajectory(const TrajectoryPlannerInput& input,
                     const TrajectoryPlannerConfig& config);

} // namespace drone_city_nav

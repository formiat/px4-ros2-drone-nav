#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/trajectory_optimizer.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/turn_smoothing.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

class ClearanceField2D;

enum class TrajectoryPlannerStatus {
  kOk,
  kInvalidRoute,
  kMissingGrid,
  kCorridorInvalid,
  kTrajectoryOptimizerInvalid,
  kInvalidTrajectory,
};

enum class TrajectoryQuality {
  kUnknown,
  kBaseline,
  kRefined,
};

struct TrajectoryPlannerConfig {
  CorridorConfig corridor{};
  TrajectoryOptimizerConfig trajectory_optimizer{};
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
  std::size_t isolated_curvature_spike_candidates{0U};
  std::size_t isolated_curvature_spikes_smoothed_geometry{0U};
  double isolated_curvature_spike_max_before_1pm{0.0};
  double isolated_curvature_spike_max_after_1pm{0.0};
  std::vector<SpeedProfileConstraintDiagnostic> top_speed_constraints;
  double total_duration_ms{0.0};
  double corridor_duration_ms{0.0};
  double trajectory_optimizer_duration_ms{0.0};
  double turn_smoothing_duration_ms{0.0};
  double speed_profile_duration_ms{0.0};
  TrajectoryPlannerStatus status{TrajectoryPlannerStatus::kOk};
  TrajectoryQuality quality{TrajectoryQuality::kUnknown};
  CorridorStats corridor{};
  TrajectoryOptimizerStats trajectory_optimizer{};
  TurnSmoothingStats turn_smoothing{};
};

struct TrajectoryPlannerInput {
  std::span<const Point2> route_points;
  const OccupancyGrid2D* prohibited_grid{nullptr};
  const ClearanceField2D* prohibited_clearance_field{nullptr};
  bool prohibited_clearance_field_cache_hit{false};
  std::span<const CorridorSample> precomputed_corridor_samples;
  const CorridorStats* precomputed_corridor_stats{nullptr};
};

struct TrajectoryPlannerResult {
  std::vector<TrajectorySegment> compact_segments;
  std::vector<CorridorSample> corridor_samples;
  std::vector<TrajectoryOptimizerWindowMetadata> trajectory_optimizer_windows;
  std::vector<TrajectoryPointSample> samples;
  TrajectorySpeedProfile speed_profile;
  TrajectoryPlannerStats stats{};
  bool valid{false};
};

enum class TrajectoryRefinementDecisionReason {
  kAccepted,
  kStaleGeneration,
  kInvalidRefined,
  kEndpointMismatch,
  kNonTraversable,
};

struct TrajectoryRefinementDecisionInput {
  std::uint64_t current_generation{0U};
  std::uint64_t snapshot_generation{0U};
  Point2 expected_start{};
  Point2 expected_goal{};
  double endpoint_tolerance_m{0.0};
  const TrajectoryPlannerResult* refined{nullptr};
  std::span<const Point2> refined_points;
  const OccupancyGrid2D* validation_grid{nullptr};
};

struct TrajectoryRefinementDecision {
  bool accepted{false};
  TrajectoryRefinementDecisionReason reason{
      TrajectoryRefinementDecisionReason::kInvalidRefined};
};

[[nodiscard]] std::string_view
trajectoryPlannerStatusName(TrajectoryPlannerStatus status) noexcept;

[[nodiscard]] std::string_view
trajectoryQualityName(TrajectoryQuality quality) noexcept;

[[nodiscard]] std::string_view
refinementDecisionReasonName(TrajectoryRefinementDecisionReason reason) noexcept;

[[nodiscard]] TrajectoryPlannerResult
planTrajectory(const TrajectoryPlannerInput& input,
               const TrajectoryPlannerConfig& config);

[[nodiscard]] TrajectoryPlannerResult
planBaselineTrajectory(const TrajectoryPlannerInput& input,
                       const TrajectoryPlannerConfig& config);

[[nodiscard]] TrajectoryPlannerResult
planOptimizedTrajectory(const TrajectoryPlannerInput& input,
                        const TrajectoryPlannerConfig& config);

[[nodiscard]] TrajectoryPlannerResult planOptimizedTrajectoryFromSnapshots(
    std::span<const Point2> route_points, const OccupancyGrid2D& prohibited_grid,
    const ClearanceField2D* prohibited_clearance_field,
    bool prohibited_clearance_field_cache_hit,
    std::span<const CorridorSample> precomputed_corridor_samples,
    const CorridorStats* precomputed_corridor_stats,
    const TrajectoryPlannerConfig& config);

[[nodiscard]] TrajectoryRefinementDecision
evaluateTrajectoryRefinement(const TrajectoryRefinementDecisionInput& input);

} // namespace drone_city_nav

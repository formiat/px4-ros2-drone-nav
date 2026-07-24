#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class RolloutTraversabilityTier {
  kPlanningClearance,
  kRuntimeProhibited,
};

enum class RolloutRejectReason {
  kNone,
  kInvalidInput,
  kOutsideGrid,
  kRawOccupied,
  kNoCandidate,
};

struct RolloutPlannerConfig {
  double horizon_m{25.0};
  double sample_step_m{1.0};
  std::size_t heading_samples{9U};
  std::size_t speed_samples{3U};
  double max_heading_offset_rad{0.9};
  double horizon_time_s{3.0};
  double minimum_speed_mps{2.0};
  double maximum_speed_mps{10.0};
  double maximum_acceleration_mps2{4.0};
  double maximum_curvature_1pm{0.20};
  double maximum_lateral_acceleration_mps2{4.0};
  std::size_t max_finalists{3U};
  double progress_weight{4.0};
  double lateral_deviation_weight{0.08};
  double heading_change_weight{2.0};
  double curvature_weight{8.0};
  double degraded_tier_penalty{100.0};
};

struct RolloutInput {
  Point2 position{};
  Point2 velocity{};
  Point2 preferred_target{};
  const OccupancyGrid2D* prohibited_grid{nullptr};
  const OccupancyGrid2D* planning_grid{nullptr};
  std::uint64_t generation{0U};
  std::uint64_t grid_revision{0U};
};

struct RolloutCandidate {
  std::vector<TrajectoryPointSample> samples;
  RolloutTraversabilityTier tier{RolloutTraversabilityTier::kRuntimeProhibited};
  double score{0.0};
  double progress_m{0.0};
  double heading_offset_rad{0.0};
  double target_speed_mps{0.0};
  double curvature_1pm{0.0};
  std::size_t deterministic_index{0U};
};

struct RolloutDiagnostics {
  std::size_t generated{0U};
  std::size_t prohibited_rejections{0U};
  std::size_t outside_grid_rejections{0U};
  std::size_t dynamic_limit_rejections{0U};
};

struct RolloutResult {
  std::vector<RolloutCandidate> ranked_candidates;
  RolloutDiagnostics diagnostics{};
  RolloutRejectReason reject_reason{RolloutRejectReason::kNone};
  std::uint64_t generation{0U};
  std::uint64_t grid_revision{0U};

  [[nodiscard]] std::span<const RolloutCandidate>
  rankedShortlist(std::size_t maximum) const noexcept;
};

class RecedingHorizonTrajectoryPlanner {
public:
  explicit RecedingHorizonTrajectoryPlanner(const RolloutPlannerConfig& config = {});

  [[nodiscard]] RolloutResult plan(const RolloutInput& input) const;
  [[nodiscard]] const RolloutPlannerConfig& config() const noexcept;

private:
  RolloutPlannerConfig config_{};
};

[[nodiscard]] const char*
rolloutTraversabilityTierName(RolloutTraversabilityTier tier) noexcept;

[[nodiscard]] const char* rolloutRejectReasonName(RolloutRejectReason reason) noexcept;

} // namespace drone_city_nav

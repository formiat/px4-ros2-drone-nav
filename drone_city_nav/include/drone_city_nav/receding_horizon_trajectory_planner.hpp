#pragma once

#include "drone_city_nav/obstacle_risk_field.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class RolloutRejectReason {
  kNone,
  kInvalidInput,
  kOutsideGrid,
  kNoCandidate,
};

enum class RolloutGridRejectReason {
  kOutsideGrid,
  kRawOccupied,
  kVehicleClearanceEnvelope,
  kTerminalResponseEnvelope,
  kTerminalStoppingEnvelope,
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
};

struct RolloutInput {
  Point2 position{};
  Point2 velocity{};
  Point2 preferred_target{};
  const OccupancyGrid2D* grid{nullptr};
  const ObstacleRiskField* risk_field{nullptr};
  double minimum_length_m{0.0};
  double minimum_path_clearance_m{0.0};
  double minimum_terminal_clearance_m{0.0};
  double terminal_braking_deceleration_mps2{0.0};
  double terminal_braking_margin_m{0.0};
  bool stationary_restart{false};
  std::uint64_t generation{0U};
  std::uint64_t grid_revision{0U};
};

struct RolloutCandidate {
  std::vector<TrajectoryPointSample> samples;
  double score{0.0};
  double progress_m{0.0};
  double heading_offset_rad{0.0};
  double target_speed_mps{0.0};
  double terminal_stopping_distance_m{0.0};
  double curvature_1pm{0.0};
  double progress_cost{0.0};
  double lateral_deviation_cost{0.0};
  double heading_change_cost{0.0};
  double curvature_cost{0.0};
  PathRiskScore risk{};
  std::size_t deterministic_index{0U};
};

struct RolloutGridRejectionDiagnostic {
  RolloutGridRejectReason reason{RolloutGridRejectReason::kRawOccupied};
  std::size_t deterministic_index{0U};
  std::size_t segment_index{0U};
  Point2 position{};
  std::optional<GridIndex> cell;
};

struct RolloutDiagnostics {
  std::size_t generated{0U};
  std::size_t grid_rejections{0U};
  std::size_t outside_grid_rejections{0U};
  std::size_t dynamic_limit_rejections{0U};
  std::size_t acceleration_rejections{0U};
  std::size_t curvature_rejections{0U};
  std::size_t lateral_acceleration_rejections{0U};
  std::optional<RolloutGridRejectionDiagnostic> first_grid_rejection;
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

[[nodiscard]] const char* rolloutRejectReasonName(RolloutRejectReason reason) noexcept;
[[nodiscard]] const char*
rolloutGridRejectReasonName(RolloutGridRejectReason reason) noexcept;

} // namespace drone_city_nav

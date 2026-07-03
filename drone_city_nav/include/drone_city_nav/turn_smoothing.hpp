#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

struct TurnSmoothingConfig {
  double trigger_heading_delta_rad{0.65};
  double trigger_min_radius_m{16.0};
  double trigger_speed_limit_mps{12.0};
  double entry_distance_m{45.0};
  double exit_distance_m{45.0};
  double sample_step_m{1.0};
  double outer_bias_ratio{0.45};
  double min_outer_shift_m{2.0};
  double max_outer_shift_m{12.0};
  double max_length_ratio{1.25};
  double min_heading_improvement_rad{0.05};
  std::size_t max_passes{8U};
};

struct TurnSmoothingCornerDiagnostic {
  bool accepted{false};
  std::string reject_reason{"none"};
  double corner_s_m{std::numeric_limits<double>::quiet_NaN()};
  double entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double shift_scale{std::numeric_limits<double>::quiet_NaN()};
  double relaxed_angle_deg{std::numeric_limits<double>::quiet_NaN()};
  double score{std::numeric_limits<double>::quiet_NaN()};
  double min_radius_before_m{std::numeric_limits<double>::quiet_NaN()};
  double min_radius_after_m{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_before_mps{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_after_mps{std::numeric_limits<double>::quiet_NaN()};
  double local_time_before_s{std::numeric_limits<double>::quiet_NaN()};
  double local_time_after_s{std::numeric_limits<double>::quiet_NaN()};
  double curvature_jump_before_1pm{std::numeric_limits<double>::quiet_NaN()};
  double curvature_jump_after_1pm{std::numeric_limits<double>::quiet_NaN()};
  double heading_delta_before_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_delta_after_rad{std::numeric_limits<double>::quiet_NaN()};
};

struct TurnSmoothingCandidateDiagnostic {
  std::string decision{"rejected"};
  std::string reject_reason{"none"};
  std::size_t pass{0U};
  std::size_t attempt_index{0U};
  std::size_t corner_index{0U};
  double corner_s_m{std::numeric_limits<double>::quiet_NaN()};
  double entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double shift_scale{std::numeric_limits<double>::quiet_NaN()};
  double applied_shift_m{std::numeric_limits<double>::quiet_NaN()};
  double relaxed_angle_deg{std::numeric_limits<double>::quiet_NaN()};
  double score{std::numeric_limits<double>::quiet_NaN()};
  double min_radius_before_m{std::numeric_limits<double>::quiet_NaN()};
  double min_radius_after_m{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_before_mps{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_after_mps{std::numeric_limits<double>::quiet_NaN()};
  double local_time_before_s{std::numeric_limits<double>::quiet_NaN()};
  double local_time_after_s{std::numeric_limits<double>::quiet_NaN()};
  double curvature_jump_before_1pm{std::numeric_limits<double>::quiet_NaN()};
  double curvature_jump_after_1pm{std::numeric_limits<double>::quiet_NaN()};
  double heading_delta_before_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_delta_after_rad{std::numeric_limits<double>::quiet_NaN()};
};

struct TurnSmoothingStats {
  std::size_t input_samples{0U};
  std::size_t output_samples{0U};
  std::size_t detected_corners{0U};
  std::size_t attempted_corners{0U};
  std::size_t candidate_attempts{0U};
  std::size_t relaxed_candidate_attempts{0U};
  std::size_t bezier_cache_hits{0U};
  std::size_t bezier_cache_misses{0U};
  std::size_t before_metrics_cache_hits{0U};
  std::size_t before_metrics_cache_misses{0U};
  std::size_t traversability_cache_hits{0U};
  std::size_t traversability_cache_misses{0U};
  std::size_t smoothed_corners{0U};
  std::size_t rejected_prohibited{0U};
  std::size_t rejected_corridor{0U};
  std::size_t rejected_length{0U};
  std::size_t rejected_not_improved{0U};
  std::size_t rejected_curvature_regression{0U};
  std::size_t rejected_radius_regression{0U};
  std::size_t rejected_speed_regression{0U};
  double max_heading_delta_before_rad{0.0};
  double max_heading_delta_after_rad{0.0};
  double max_curvature_jump_before_1pm{0.0};
  double max_curvature_jump_after_1pm{0.0};
  double min_inner_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double max_applied_outer_shift_m{0.0};
  double accepted_entry_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_exit_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_shift_scale{std::numeric_limits<double>::quiet_NaN()};
  double accepted_relaxed_angle_deg{std::numeric_limits<double>::quiet_NaN()};
  double accepted_score{std::numeric_limits<double>::quiet_NaN()};
  double accepted_min_radius_before_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_min_radius_after_m{std::numeric_limits<double>::quiet_NaN()};
  double accepted_min_speed_before_mps{std::numeric_limits<double>::quiet_NaN()};
  double accepted_min_speed_after_mps{std::numeric_limits<double>::quiet_NaN()};
  double accepted_local_time_before_s{std::numeric_limits<double>::quiet_NaN()};
  double accepted_local_time_after_s{std::numeric_limits<double>::quiet_NaN()};
  double candidate_build_duration_ms{0.0};
  double candidate_replace_duration_ms{0.0};
  double collision_check_duration_ms{0.0};
  double metrics_duration_ms{0.0};
  double shape_diagnostics_duration_ms{0.0};
  double speed_profile_duration_ms{0.0};
  std::vector<TurnSmoothingCornerDiagnostic> corner_diagnostics;
  std::vector<TurnSmoothingCandidateDiagnostic> candidate_diagnostics;
};

struct TurnSmoothingResult {
  std::vector<TrajectoryPointSample> samples;
  TurnSmoothingStats stats{};
  bool changed{false};
  bool valid{false};
};

[[nodiscard]] TurnSmoothingResult
smoothTrajectoryTurns(std::span<const TrajectoryPointSample> samples,
                      std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TurnSmoothingConfig& config,
                      const VelocityFollowerConfig& speed_config);

} // namespace drone_city_nav

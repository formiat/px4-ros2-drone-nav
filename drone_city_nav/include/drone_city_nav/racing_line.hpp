#pragma once

#include "drone_city_nav/corridor.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RacingLineConfig {
  bool enabled{true};
  double optimizer_sample_step_m{0.0};
  std::size_t max_iterations{80U};
  double initial_offset_step_m{2.0};
  double min_offset_step_m{0.1};
  double cooling_ratio{0.5};
  double weight_length{1.0};
  double weight_curvature{25.0};
  double weight_curvature_change{10.0};
  double weight_offset_change{1.0};
  double weight_offset_second_change{10.0};
  double weight_center_bias{0.02};
  double weight_time{0.0};
  double max_length_ratio{1.25};
  std::size_t regularization_iterations{2U};
  double regularization_max_time_regression_s{0.5};
};

struct RacingLineStats {
  std::size_t input_samples{0U};
  std::size_t optimizer_samples{0U};
  std::size_t output_samples{0U};
  std::size_t iterations{0U};
  std::size_t candidate_evaluations{0U};
  std::size_t collision_rejections{0U};
  double initial_cost{0.0};
  double final_cost{0.0};
  double centerline_length_m{0.0};
  double final_length_m{0.0};
  double estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t curvature_limited_samples{0U};
  double centerline_estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double centerline_min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double centerline_max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t centerline_curvature_limited_samples{0U};
  double best_candidate_estimated_time_s{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_score{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_min_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double best_candidate_max_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  std::size_t best_candidate_curvature_limited_samples{0U};
  double time_gain_s{std::numeric_limits<double>::quiet_NaN()};
  double regularization_time_delta_s{std::numeric_limits<double>::quiet_NaN()};
  bool regularization_applied{false};
  std::size_t regularization_iterations{0U};
  double pre_regularization_max_curvature_jump_1pm{
      std::numeric_limits<double>::quiet_NaN()};
  double post_regularization_max_curvature_jump_1pm{
      std::numeric_limits<double>::quiet_NaN()};
  double max_abs_offset_m{0.0};
  double max_abs_curvature_1pm{0.0};
  double mean_abs_curvature_1pm{0.0};
};

struct RacingLineResult {
  std::vector<TrajectoryPointSample> samples;
  RacingLineStats stats{};
  bool valid{false};
};

[[nodiscard]] RacingLineResult
optimizeRacingLine(std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const RacingLineConfig& config,
                   const VelocityFollowerConfig& speed_config);

} // namespace drone_city_nav

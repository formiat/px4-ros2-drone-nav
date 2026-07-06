#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_passage_validation.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

struct VerticalProfileConfig {
  bool enabled{true};
  double gate_clearance_margin_m{0.5};
  double max_vertical_speed_mps{2.5};
  double max_vertical_accel_mps2{2.0};
  double max_vertical_jerk_mps3{6.0};
  double max_climb_angle_rad{0.20943951023931953};
  double min_transition_distance_m{6.0};
  double max_transition_distance_m{80.0};
  std::size_t max_diagnostics{8U};
};

struct VerticalProfilePassageDiagnostic {
  std::string structure_id;
  std::string opening_id;
  double entry_s_m{0.0};
  double exit_s_m{0.0};
  double approach_start_s_m{0.0};
  double exit_end_s_m{0.0};
  double gate_z_m{std::numeric_limits<double>::quiet_NaN()};
  double min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double max_z_m{std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
  bool valid{false};
};

struct VerticalProfileStats {
  bool enabled{false};
  bool active{false};
  bool applied{false};
  bool valid{true};
  std::size_t passages_matched{0U};
  std::size_t passages_profiled{0U};
  std::size_t infeasible_count{0U};
  double min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double max_z_m{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_dz_ds{0.0};
  double max_abs_d2z_ds2{0.0};
  double max_abs_d3z_ds3{0.0};
  double min_vertical_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  std::vector<VerticalProfilePassageDiagnostic> diagnostics;
};

struct VerticalProfileResult {
  bool valid{true};
  VerticalProfileStats stats{};
};

[[nodiscard]] const char* verticalProfileStatusName(bool valid) noexcept;

[[nodiscard]] VerticalProfileResult
applyVerticalProfile(std::span<TrajectoryPointSample> samples,
                     const KnownPassageMap* map,
                     const KnownPassageValidationConfig& validation_config,
                     const VerticalProfileConfig& config, double cruise_altitude_m);

} // namespace drone_city_nav

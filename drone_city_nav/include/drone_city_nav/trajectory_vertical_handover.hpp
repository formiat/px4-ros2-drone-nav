#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"

#include <limits>
#include <span>

namespace drone_city_nav {

struct VerticalTrajectoryHandoverState {
  double current_altitude_m{std::numeric_limits<double>::quiet_NaN()};
  double current_vertical_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double current_horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool altitude_valid{false};
  bool vertical_velocity_valid{false};
};

struct VerticalTrajectoryHandoverResult {
  bool applied{false};
  const char* reason{"not_applied"};
  double candidate_s_m{std::numeric_limits<double>::quiet_NaN()};
  double join_s_m{std::numeric_limits<double>::quiet_NaN()};
  double anchor_z_m{std::numeric_limits<double>::quiet_NaN()};
  double join_z_m{std::numeric_limits<double>::quiet_NaN()};
  double target_z_before_m{std::numeric_limits<double>::quiet_NaN()};
  double target_z_after_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] VerticalTrajectoryHandoverResult
reanchorTrajectoryVerticalPrefix(std::span<const TrajectoryPointSample> current_samples,
                                 std::span<TrajectoryPointSample> candidate_samples,
                                 Point2 current_position,
                                 const VerticalTrajectoryHandoverState& state);

} // namespace drone_city_nav

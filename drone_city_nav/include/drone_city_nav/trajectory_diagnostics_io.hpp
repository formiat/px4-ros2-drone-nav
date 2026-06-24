#pragma once

#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <cstddef>
#include <string>

namespace drone_city_nav {

[[nodiscard]] std::string finalTrajectorySamplesCsvHeader();

[[nodiscard]] std::string
finalTrajectorySamplesCsvRow(std::size_t sample_index,
                             const TrajectoryPointSample& sample,
                             const TrajectorySpeedSample& speed_sample,
                             double time_from_start_s, double time_to_finish_s);

[[nodiscard]] std::string
finalTrajectoryDiagnosticsSummaryJson(const TrajectoryPlannerStats& stats,
                                      const TrajectoryShapeDiagnostics& shape);

} // namespace drone_city_nav

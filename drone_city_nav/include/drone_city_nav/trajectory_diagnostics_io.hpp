#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace drone_city_nav {

struct TrajectoryPlannerDiagnosticsEnvelope {
  std::uint64_t planner_path_id{0U};
  std::uint64_t path_stamp_ns{0U};
  TrajectoryPlannerStats stats{};
};

[[nodiscard]] std::string finalTrajectorySamplesCsvHeader();

[[nodiscard]] std::string
finalTrajectorySamplesCsvRow(std::size_t sample_index,
                             const TrajectoryPointSample& sample,
                             const TrajectorySpeedSample& speed_sample,
                             double time_from_start_s, double time_to_finish_s);

[[nodiscard]] std::string
racingLineDiagnosticsJsonFields(const TrajectoryPlannerStats& stats);

[[nodiscard]] std::string
finalTrajectoryDiagnosticsSummaryJson(const TrajectoryPlannerStats& stats,
                                      const TrajectoryShapeDiagnostics& shape);

[[nodiscard]] std::string
trajectoryPlannerDiagnosticsJson(std::uint64_t planner_path_id,
                                 std::uint64_t path_stamp_ns,
                                 const TrajectoryPlannerStats& stats);

[[nodiscard]] std::optional<TrajectoryPlannerDiagnosticsEnvelope>
parseTrajectoryPlannerDiagnosticsJson(const std::string& json);

} // namespace drone_city_nav

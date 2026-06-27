#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct FinalTrajectorySamplesCsvInput {
  std::string_view source_label;
  std::uint64_t local_path_update_id{0U};
  std::uint64_t planner_path_id{0U};
  bool trajectory_valid{false};
  TrajectoryPlannerStatus trajectory_status{
      TrajectoryPlannerStatus::kInvalidTrajectory};
  std::span<const TrajectoryPointSample> samples;
  const TrajectorySpeedProfile* speed_profile{nullptr};
};

[[nodiscard]] std::vector<double>
finalTrajectoryProfiledTimesFromStart(std::span<const TrajectoryPointSample> samples,
                                      const TrajectorySpeedProfile& speed_profile);

bool writeFinalTrajectorySamplesCsv(std::ostream& stream,
                                    const FinalTrajectorySamplesCsvInput& input);

bool writeFinalTrajectorySamplesCsvFile(const std::filesystem::path& path,
                                        const FinalTrajectorySamplesCsvInput& input);

bool writeFinalTrajectorySummaryJson(
    std::ostream& stream, const TrajectoryPlannerStats& stats,
    const TrajectoryShapeDiagnostics& shape_diagnostics);

bool writeFinalTrajectorySummaryJsonFile(
    const std::filesystem::path& path, const TrajectoryPlannerStats& stats,
    const TrajectoryShapeDiagnostics& shape_diagnostics);

} // namespace drone_city_nav

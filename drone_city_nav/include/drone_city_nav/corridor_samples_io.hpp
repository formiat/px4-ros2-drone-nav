#pragma once

#include "drone_city_nav/trajectory_planner.hpp"

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <string_view>

namespace drone_city_nav {

void writeCsvNumberOrEmpty(std::ostream& stream, double value);

bool writeCorridorSamplesCsv(std::ostream& stream,
                             const TrajectoryPlannerResult& result,
                             std::string_view source_label,
                             std::uint64_t candidate_path_id);

bool writeCorridorSamplesCsvFile(const std::filesystem::path& path,
                                 const TrajectoryPlannerResult& result,
                                 std::string_view source_label,
                                 std::uint64_t candidate_path_id);

} // namespace drone_city_nav

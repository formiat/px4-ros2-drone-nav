#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::size_t
countIsolatedCurvatureSpikes(std::span<const TrajectoryPointSample> samples) noexcept;

[[nodiscard]] double
maxIsolatedCurvatureSpike(std::span<const TrajectoryPointSample> samples) noexcept;

[[nodiscard]] std::size_t
smoothIsolatedCurvatureSpikeGeometry(std::vector<TrajectoryPointSample>& samples,
                                     const OccupancyGrid2D& grid);

} // namespace drone_city_nav

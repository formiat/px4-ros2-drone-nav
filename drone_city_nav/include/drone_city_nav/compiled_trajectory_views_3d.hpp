#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include <vector>

namespace drone_city_nav {

// Visualization-only planar projection. It is derived from the sealed route
// on demand and therefore cannot become a second geometry authority.
[[nodiscard]] std::vector<Point2>
projectCompiledTrajectoryTo2D(const CompiledTrajectory3D& trajectory);

} // namespace drone_city_nav

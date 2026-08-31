#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include <optional>
#include <vector>

namespace drone_city_nav {

// Visualization-only planar projection. It is derived from the sealed route
// on demand and therefore cannot become a second geometry authority.
[[nodiscard]] std::vector<Point2>
projectCompiledTrajectoryTo2D(const CompiledTrajectory3D& trajectory);

// Returns the sealed time-profile remainder at a route station. At an exact
// stop-turn sample the result conservatively includes the stationary turn;
// inside an outgoing segment it uses that segment's departure time.
[[nodiscard]] std::optional<double>
remainingCompiledTrajectoryTime3D(const CompiledTrajectory3D& trajectory,
                                  double station_m) noexcept;

} // namespace drone_city_nav

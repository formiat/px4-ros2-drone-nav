#include "px4_offboard_node.hpp"

namespace drone_city_nav {

NoStaticSpeedConstraint Px4OffboardNode::noStaticSpeedConstraint() const {
  NoStaticSpeedConstraint constraint{};
  if (!velocity_follower_config_.no_static_speed_policy.enabled) {
    return constraint;
  }
  if (!prohibitedGridFresh()) {
    constraint.boundary = NoStaticSpeedBoundary::kStaleGrid;
    return constraint;
  }

  const std::optional<OccupancyGrid2D> grid = currentProhibitedGrid();
  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(final_trajectory_samples_, current_position_);
  if (!grid.has_value() || !projection.has_value()) {
    constraint.boundary = NoStaticSpeedBoundary::kStaleGrid;
    return constraint;
  }

  const auto inspect_segment =
      [&](const Point2 segment_start,
          const Point2 segment_end) -> std::optional<NoStaticSpeedConstraint> {
    const std::optional<GridIndex> start_cell = grid->worldToCell(segment_start);
    const std::optional<GridIndex> end_cell = grid->worldToCell(segment_end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      return NoStaticSpeedConstraint{.observation_valid = true,
                                     .boundary_distance_m = 0.0,
                                     .boundary = NoStaticSpeedBoundary::kUnknown};
    }
    for (const GridIndex cell : grid->cellsOnLine(*start_cell, *end_cell)) {
      NoStaticSpeedBoundary boundary = NoStaticSpeedBoundary::kDisabled;
      if (grid->isProhibited(cell)) {
        boundary = NoStaticSpeedBoundary::kProhibited;
      } else if (grid->state(cell) == CellState::kUnknown) {
        boundary = NoStaticSpeedBoundary::kUnknown;
      }
      if (boundary != NoStaticSpeedBoundary::kDisabled) {
        return NoStaticSpeedConstraint{
            .observation_valid = true,
            .boundary_distance_m = distance(current_position_, grid->cellCenter(cell)),
            .boundary = boundary,
        };
      }
    }
    return std::nullopt;
  };

  if (const std::optional<NoStaticSpeedConstraint> boundary =
          inspect_segment(current_position_, projection->point);
      boundary.has_value()) {
    return *boundary;
  }

  Point2 segment_start = projection->point;
  constexpr double kStationToleranceM = 1.0e-6;
  for (const TrajectoryPointSample& sample : final_trajectory_samples_) {
    if (sample.s_m <= projection->s_m + kStationToleranceM) {
      continue;
    }
    if (const std::optional<NoStaticSpeedConstraint> boundary =
            inspect_segment(segment_start, sample.point);
        boundary.has_value()) {
      return *boundary;
    }
    segment_start = sample.point;
  }

  constraint.observation_valid = true;
  constraint.boundary_distance_m = std::numeric_limits<double>::infinity();
  return constraint;
}

} // namespace drone_city_nav

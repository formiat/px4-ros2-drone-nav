#include "drone_city_nav/planning_grid_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameBounds(const GridBounds& lhs, const GridBounds& rhs) noexcept {
  return lhs.origin_x == rhs.origin_x && lhs.origin_y == rhs.origin_y &&
         lhs.resolution_m == rhs.resolution_m && lhs.width_cells == rhs.width_cells &&
         lhs.height_cells == rhs.height_cells;
}

[[nodiscard]] bool sameFingerprint(const OccupancyGridFingerprint& lhs,
                                   const OccupancyGridFingerprint& rhs) noexcept {
  return sameBounds(lhs.bounds, rhs.bounds) && lhs.cells_hash == rhs.cells_hash &&
         lhs.inflated_hash == rhs.inflated_hash;
}

[[nodiscard]] bool
preparedBuildAvailable(const PlanningGridPreparationInput& input) noexcept {
  return input.build_result != nullptr &&
         input.build_result->status == PlanningGridStatus::kReady &&
         input.build_result->grid.has_value() &&
         input.build_result->planning_grid.has_value() &&
         std::isfinite(input.relaxation_center.x) &&
         std::isfinite(input.relaxation_center.y) &&
         std::isfinite(input.mission_goal.x) && std::isfinite(input.mission_goal.y) &&
         std::isfinite(input.relaxation_radius_m) && input.relaxation_radius_m >= 0.0 &&
         std::isfinite(input.clearance_max_distance_m) &&
         input.clearance_max_distance_m >= 0.0;
}

} // namespace

std::optional<PreparedPlanningGridSnapshot>
PlanningGridSnapshotBuilder::prepare(const PlanningGridPreparationInput& input) {
  if (!preparedBuildAvailable(input)) {
    return std::nullopt;
  }

  const PlanningGridBuildResult& build = *input.build_result;
  OccupancyGrid2D runtime_grid = build.grid.value();
  OccupancyGrid2D planning_grid = build.planning_grid.value();
  if (!sameBounds(runtime_grid.bounds(), planning_grid.bounds())) {
    return std::nullopt;
  }

  DirectedInflationEscapeResult directed_escape =
      directed_escape_planner_.update(planning_grid, input.relaxation_center,
                                      input.mission_goal, input.directed_escape);
  if (directed_escape.applied) {
    OccupancyGrid2D escaped_planning_grid = planning_grid;
    directed_escape.relaxation = applyDirectedInflationEscape(
        escaped_planning_grid, directed_escape, input.directed_escape.tunnel_width_m);
    directed_escape.connected =
        !directed_escape.centerline.empty() &&
        std::ranges::all_of(
            directed_escape.centerline, [&escaped_planning_grid](const Point2 point) {
              const std::optional<GridIndex> cell =
                  escaped_planning_grid.worldToCell(point);
              return cell.has_value() && !escaped_planning_grid.isProhibited(*cell);
            });
    if (directed_escape.connected) {
      planning_grid = std::move(escaped_planning_grid);
    } else {
      directed_escape.applied = false;
      directed_escape.state = DirectedInflationEscapeState::kFailed;
    }
  }

  const LocalInflationRelaxationStats runtime_relaxation =
      runtime_grid.clearInflationWithinRadius(input.relaxation_center,
                                              input.relaxation_radius_m);
  const LocalInflationRelaxationStats planning_relaxation =
      planning_grid.clearInflationWithinRadius(input.relaxation_center,
                                               input.relaxation_radius_m);
  ClearanceField2D runtime_clearance = ClearanceField2D::build(
      runtime_grid, input.clearance_max_distance_m, ClearanceSource::kProhibited);
  ClearanceField2D planning_clearance = ClearanceField2D::build(
      planning_grid, input.clearance_max_distance_m, ClearanceSource::kProhibited);

  PlanningGridVersion version{
      .build_revision = next_revision_,
      .memory_producer_instance_id = build.applied_memory_producer_instance_id,
      .memory_sequence = build.applied_memory_sequence,
      .lidar_update_ns = build.applied_lidar_update_ns,
      .config_fingerprint = input.config_fingerprint,
      .runtime_prohibited = runtime_grid.prohibitedFingerprint(),
      .planning_clearance = planning_grid.prohibitedFingerprint(),
  };
  ++next_revision_;
  return PreparedPlanningGridSnapshot{
      .runtime_prohibited_grid = std::move(runtime_grid),
      .planning_clearance_grid = std::move(planning_grid),
      .runtime_clearance = std::move(runtime_clearance),
      .planning_clearance = std::move(planning_clearance),
      .version = version,
      .runtime_relaxation = runtime_relaxation,
      .planning_relaxation = planning_relaxation,
      .directed_escape = std::move(directed_escape),
  };
}

std::uint64_t PlanningGridSnapshotBuilder::nextRevision() const noexcept {
  return next_revision_;
}

bool planningGridVersionsEqual(const PlanningGridVersion& lhs,
                               const PlanningGridVersion& rhs) noexcept {
  return lhs.build_revision == rhs.build_revision &&
         lhs.memory_producer_instance_id == rhs.memory_producer_instance_id &&
         lhs.memory_sequence == rhs.memory_sequence &&
         lhs.lidar_update_ns == rhs.lidar_update_ns &&
         lhs.config_fingerprint == rhs.config_fingerprint &&
         sameFingerprint(lhs.runtime_prohibited, rhs.runtime_prohibited) &&
         sameFingerprint(lhs.planning_clearance, rhs.planning_clearance);
}

} // namespace drone_city_nav

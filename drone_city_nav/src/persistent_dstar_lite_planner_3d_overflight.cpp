#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {

namespace {

// Horizontal metres a climb or a descent takes per metre of height, steepest
// last: a slant keeps the leg's speed, and a vertical line is flown from rest.
// A gentler one than two to one spends its last hundred metres under the
// crowns' tops, where the trees not yet seen stand (r3261).
constexpr std::array<double, 3U> kSlantRunsPerMetre{2.0, 1.0, 0.0};

// The horizontal distance from a point to the leg's ground track.
[[nodiscard]] double trackDistanceM(const double x, const double y, const Point3& start,
                                    const Point3& goal) noexcept {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  const double along =
      length_squared > 0.0
          ? std::clamp(((x - start.x) * dx + (y - start.y) * dy) / length_squared, 0.0,
                       1.0)
          : 0.0;
  return std::hypot(x - start.x - along * dx, y - start.y - along * dy);
}

// The top of the highest occupied voxel the memory holds within `corridor_m`
// of the leg's ground track.
[[nodiscard]] std::optional<double>
highestObservedObstacleZM(const ObservedOccupancyGrid3D& occupancy, const Point3& start,
                          const Point3& goal, const double corridor_m) noexcept {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const GridBounds3D& bounds = occupancy.bounds();
  const double chunk_m = static_cast<double>(kChunkSize) * bounds.resolution_m;
  // A chunk's centre lies within half its diagonal of each of its cells.
  const double chunk_reach_m = corridor_m + chunk_m * std::numbers::sqrt2 / 2.0;
  std::optional<int> top_cell;
  for (const auto& [index, storage] : occupancy.chunks()) {
    if (top_cell.has_value() && (index.z + 1) * kChunkSize - 1 <= *top_cell) {
      continue;
    }
    if (trackDistanceM(bounds.origin_x + (static_cast<double>(index.x) + 0.5) * chunk_m,
                       bounds.origin_y + (static_cast<double>(index.y) + 0.5) * chunk_m,
                       start, goal) > chunk_reach_m) {
      continue;
    }
    const OccupancyGrid3D::Chunk& occupied = storage->occupied;
    for (int layer = kChunkSize - 1; layer >= 0; --layer) {
      const int cell_z = index.z * kChunkSize + layer;
      if (top_cell.has_value() && cell_z <= *top_cell) {
        break;
      }
      bool layer_occupied{false};
      for (int row = 0; row < kChunkSize && !layer_occupied; ++row) {
        for (int column = 0; column < kChunkSize && !layer_occupied; ++column) {
          const GridIndex3D cell{index.x * kChunkSize + column,
                                 index.y * kChunkSize + row, cell_z};
          const std::size_t bit = ObservedOccupancyGrid3D::localBitIndex(cell);
          layer_occupied =
              (occupied.at(bit / 64U) & (std::uint64_t{1U} << (bit % 64U))) != 0U &&
              trackDistanceM(bounds.origin_x + (static_cast<double>(cell.x) + 0.5) *
                                                   bounds.resolution_m,
                             bounds.origin_y + (static_cast<double>(cell.y) + 0.5) *
                                                   bounds.resolution_m,
                             start, goal) <= corridor_m;
        }
      }
      if (layer_occupied) {
        top_cell = cell_z;
        break;
      }
    }
  }
  if (!top_cell.has_value()) {
    return std::nullopt;
  }
  return bounds.origin_z + static_cast<double>(*top_cell + 1) * bounds.resolution_m;
}

} // namespace

// The open sky's leg (specification K76): over everything the memory holds
// along it, within the planner's clearance reach of its ground track (a crown
// 40 m aside of the leg raised every leg of r3273 to 49 m, and its descents to
// the goals took 20 s each). The level is the highest obstacle seen there plus
// the margin, under the envelope's
// ceiling and no lower than either end; the vehicle climbs to it along the leg
// and comes down along it to the goal, at the gentlest slant the raw world
// clears. Unknown space on the way is free (I1): what stands in it is seen
// from the sensor's range, and the route is taken again from where the vehicle
// then is. Nothing is searched: with the leg held the lattice searches idle.
std::optional<std::vector<Point3>>
PersistentDStarLitePlanner3DImpl::overflightPath(const Point3& start,
                                                 const Point3& goal) const {
  if (!(config_.overflight_margin_m > 0.0) || world_.observed_occupancy == nullptr) {
    return std::nullopt;
  }
  const double horizontal_m = std::hypot(goal.x - start.x, goal.y - start.y);
  if (!(horizontal_m > config_.minimum_horizontal_step_m)) {
    return std::nullopt;
  }
  const double ceiling_m =
      config_.flight_envelope.maximum_target_z_m - config_.minimum_vertical_step_m;
  const std::optional<double> top_m = highestObservedObstacleZM(
      *world_.observed_occupancy, start, goal, config_.clearance_ranking_distance_m);
  const double level_m = std::min(
      ceiling_m,
      std::max({start.z, goal.z,
                top_m.has_value() ? *top_m + config_.overflight_margin_m : start.z}));
  const double direction_x = (goal.x - start.x) / horizontal_m;
  const double direction_y = (goal.y - start.y) / horizontal_m;
  // On the way down already (inside the gentlest descent's cone over the
  // goal): the rest of the descent is taken from where the vehicle is, level
  // at its height and then down, before the leg is climbed again. A descent a
  // crown had closed sent the vehicle 25 m back up to the level and cost it
  // 20 s (r3261).
  const double remaining_descent_m = start.z - goal.z;
  if (remaining_descent_m > 0.0 &&
      horizontal_m <= kSlantRunsPerMetre.front() * remaining_descent_m +
                          config_.minimum_horizontal_step_m) {
    for (const double descent_run : kSlantRunsPerMetre) {
      const double descent_run_m =
          std::min(horizontal_m, descent_run * remaining_descent_m);
      std::vector<Point3> path{start};
      if (horizontal_m - descent_run_m > 1.0e-6) {
        path.push_back(Point3{goal.x - direction_x * descent_run_m,
                              goal.y - direction_y * descent_run_m, start.z});
      }
      path.push_back(goal);
      if (lattice_.pathTraversable(path)) {
        return path;
      }
    }
  }
  const double climb_m = std::max(0.0, level_m - start.z);
  const double descent_m = std::max(0.0, level_m - goal.z);
  for (const double descent_run : kSlantRunsPerMetre) {
    for (const double climb_run : kSlantRunsPerMetre) {
      const double climb_run_m = climb_run * climb_m;
      const double descent_run_m = descent_run * descent_m;
      // A height that is not there has one slant, not three.
      if (climb_run_m + descent_run_m > horizontal_m ||
          (!(climb_m > 0.0) && climb_run != kSlantRunsPerMetre.front()) ||
          (!(descent_m > 0.0) && descent_run != kSlantRunsPerMetre.front())) {
        continue;
      }
      std::vector<Point3> path{start};
      const auto append = [&path](const Point3& point) {
        if (distance3D(path.back(), point) > 1.0e-6) {
          path.push_back(point);
        }
      };
      append(Point3{start.x + direction_x * climb_run_m,
                    start.y + direction_y * climb_run_m, level_m});
      append(Point3{goal.x - direction_x * descent_run_m,
                    goal.y - direction_y * descent_run_m, level_m});
      append(goal);
      if (path.size() >= 2U && lattice_.pathTraversable(path)) {
        return path;
      }
    }
  }
  return std::nullopt;
}

bool PersistentDStarLitePlanner3DImpl::holdOverflight(
    const PersistentPlannerRequest3D& request, PlannerUpdate3D& update) {
  if (coordinator_.incumbent() == nullptr) {
    if (std::optional<std::vector<Point3>> path =
            overflightPath(request.start, exact_goal_)) {
      if (std::optional<SpatialRouteCandidate3D> candidate = makeCandidate(
              std::move(*path), SpatialRouteCandidateSource3D::kOverflight,
              request.velocity)) {
        update.improved_incumbent = coordinator_.consider(std::move(*candidate));
      }
    }
  }
  return coordinator_.incumbent() != nullptr &&
         coordinator_.incumbent()->source == SpatialRouteCandidateSource3D::kOverflight;
}

} // namespace drone_city_nav::detail

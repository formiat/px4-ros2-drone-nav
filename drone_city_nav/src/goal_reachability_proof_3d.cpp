#include "drone_city_nav/goal_reachability_proof_3d.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
constexpr std::size_t kVoxelsPerChunk{
    static_cast<std::size_t>(kChunkSize * kChunkSize * kChunkSize)};

// Visited voxels, allocated a chunk at a time: the flood touches a small part
// of a grid whose dense mask would be tens of megabytes.
class VisitedVoxels final {
public:
  [[nodiscard]] bool markVisited(const GridIndex3D index) {
    const OccupancyChunkIndex3D chunk_index{index.x / kChunkSize, index.y / kChunkSize,
                                            index.z / kChunkSize};
    std::unique_ptr<std::bitset<kVoxelsPerChunk>>& chunk = chunks_[chunk_index];
    if (chunk == nullptr) {
      chunk = std::make_unique<std::bitset<kVoxelsPerChunk>>();
    }
    constexpr std::size_t kSide{static_cast<std::size_t>(kChunkSize)};
    const std::size_t bit = (static_cast<std::size_t>(index.z % kChunkSize) * kSide +
                             static_cast<std::size_t>(index.y % kChunkSize)) *
                                kSide +
                            static_cast<std::size_t>(index.x % kChunkSize);
    if (chunk->test(bit)) {
      return false;
    }
    chunk->set(bit);
    return true;
  }

private:
  std::unordered_map<OccupancyChunkIndex3D,
                     std::unique_ptr<std::bitset<kVoxelsPerChunk>>,
                     OccupancyChunkIndex3DHash>
      chunks_;
};

constexpr std::array<std::array<int, 3>, 6> kNeighbours{
    {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};

// The space the planner flies in: the grid and the layers of the envelope.
struct FlightSpace {
  const ObservedOccupancyGrid3D& grid;
  int lowest_z;
  int highest_z;

  [[nodiscard]] bool contains(const GridIndex3D cell) const noexcept {
    const GridBounds3D& bounds = grid.bounds();
    return cell.x >= 0 && cell.y >= 0 && cell.z >= lowest_z &&
           cell.x < bounds.width_cells && cell.y < bounds.height_cells &&
           cell.z <= highest_z;
  }

  // Free, and none of its 26 neighbours occupied: the body fits there.
  [[nodiscard]] bool fitsBody(const GridIndex3D cell) const noexcept {
    for (int z = -1; z <= 1; ++z) {
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          const GridIndex3D near{cell.x + x, cell.y + y, cell.z + z};
          if (grid.contains(near) && grid.isOccupied(near)) {
            return false;
          }
        }
      }
    }
    return true;
  }
};

// The flood from the goal's capture region through the voxels the body fits
// in; the region is closed when the flood ends short of the vehicle.
void floodFromGoal(const FlightSpace& space, const Point3& vehicle, const Point3& goal,
                   const double capture_radius_m, const std::size_t voxel_budget,
                   GoalReachabilityProof3D& proof) {
  constexpr double kVehicleReachM{1.0};
  const GridBounds3D& bounds = space.grid.bounds();
  const std::optional<GridIndex3D> centre = space.grid.worldToCell(goal);
  if (!centre.has_value()) {
    return;
  }
  VisitedVoxels visited;
  std::vector<GridIndex3D> frontier;
  std::vector<GridIndex3D> next;
  const int reach = static_cast<int>(std::ceil(capture_radius_m / bounds.resolution_m));
  for (int z = centre->z - reach; z <= centre->z + reach; ++z) {
    for (int y = centre->y - reach; y <= centre->y + reach; ++y) {
      for (int x = centre->x - reach; x <= centre->x + reach; ++x) {
        const GridIndex3D cell{x, y, z};
        if (space.contains(cell) &&
            squaredDistance(space.grid.cellCenter(cell), goal) <=
                capture_radius_m * capture_radius_m &&
            space.fitsBody(cell) && visited.markVisited(cell)) {
          frontier.push_back(cell);
        }
      }
    }
  }
  proof.goal_region_voxels = frontier.size();
  while (!frontier.empty()) {
    next.clear();
    for (const GridIndex3D cell : frontier) {
      if (squaredDistance(space.grid.cellCenter(cell), vehicle) <=
          kVehicleReachM * kVehicleReachM) {
        return;
      }
      for (const std::array<int, 3>& step : kNeighbours) {
        const GridIndex3D neighbour{cell.x + step[0], cell.y + step[1],
                                    cell.z + step[2]};
        if (!space.contains(neighbour) || !space.fitsBody(neighbour) ||
            !visited.markVisited(neighbour)) {
          continue;
        }
        if (++proof.goal_region_voxels > voxel_budget) {
          return;
        }
        next.push_back(neighbour);
      }
    }
    std::swap(frontier, next);
  }
  proof.goal_region_closed = true;
}

} // namespace

GoalReachabilityProof3D
proveGoalUnreachable3D(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                       const Point3& goal, const FlightEnvelopeConfig& envelope,
                       const double capture_radius_m, const std::size_t voxel_budget) {
  GoalReachabilityProof3D proof;
  const GridBounds3D& bounds = grid.bounds();
  // The layers the envelope's band touches, its floor and ceiling included.
  const auto layer = [&bounds](const double z_m) {
    return static_cast<int>(std::floor((z_m - bounds.origin_z) / bounds.resolution_m));
  };
  const FlightSpace space{
      .grid = grid,
      .lowest_z = std::max(layer(envelope.minimum_target_z_m), 0),
      .highest_z = std::min(layer(envelope.maximum_target_z_m), bounds.depth_cells - 1),
  };
  const std::optional<GridIndex3D> start = grid.worldToCell(vehicle);
  if (!start.has_value() || !space.contains(*start)) {
    return proof;
  }
  proof.vehicle_inside_grid = true;
  // The planner flies to the goal's height clamped into the envelope.
  const Point3 flown_goal{goal.x, goal.y,
                          clampToFlightEnvelope(goal.z, envelope).value_or(goal.z)};
  const std::optional<GridIndex3D> goal_cell = grid.worldToCell(flown_goal);
  const auto same = [](const GridIndex3D first, const GridIndex3D second) noexcept {
    return first.x == second.x && first.y == second.y && first.z == second.z;
  };
  VisitedVoxels visited;
  std::vector<GridIndex3D> frontier{*start};
  std::vector<GridIndex3D> next;
  static_cast<void>(visited.markVisited(*start));
  proof.component_voxels = 1U;
  proof.goal_inside = goal_cell.has_value() && same(*goal_cell, *start);
  while (!frontier.empty() && !proof.goal_inside && !proof.budget_exhausted) {
    next.clear();
    for (const GridIndex3D cell : frontier) {
      for (const std::array<int, 3>& step : kNeighbours) {
        const GridIndex3D neighbour{cell.x + step[0], cell.y + step[1],
                                    cell.z + step[2]};
        if (!space.contains(neighbour) || grid.isOccupied(neighbour) ||
            !visited.markVisited(neighbour)) {
          continue;
        }
        ++proof.component_voxels;
        proof.goal_inside =
            proof.goal_inside || (goal_cell.has_value() && same(*goal_cell, neighbour));
        proof.budget_exhausted =
            proof.budget_exhausted || proof.component_voxels > voxel_budget;
        next.push_back(neighbour);
      }
    }
    std::swap(frontier, next);
  }
  if (!proof.provenUnreachable()) {
    floodFromGoal(space, vehicle, flown_goal, capture_radius_m, voxel_budget, proof);
  }
  return proof;
}

const char*
goalReachabilityProofVerdict(const GoalReachabilityProof3D& proof) noexcept {
  if (!proof.vehicle_inside_grid) {
    return "vehicle_outside_grid";
  }
  if (proof.goal_region_closed) {
    return "goal_region_closed";
  }
  if (proof.goal_inside) {
    return "goal_reachable";
  }
  if (proof.budget_exhausted) {
    return "budget_exhausted";
  }
  return "proven_unreachable";
}

} // namespace drone_city_nav

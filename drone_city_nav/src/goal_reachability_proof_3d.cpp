#include "drone_city_nav/goal_reachability_proof_3d.hpp"

#include <array>
#include <bitset>
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

} // namespace

GoalReachabilityProof3D proveGoalUnreachable3D(const ObservedOccupancyGrid3D& grid,
                                               const Point3& vehicle,
                                               const Point3& goal,
                                               const std::size_t voxel_budget) {
  GoalReachabilityProof3D proof;
  const std::optional<GridIndex3D> start = grid.worldToCell(vehicle);
  if (!start.has_value()) {
    return proof;
  }
  proof.vehicle_inside_grid = true;
  const std::optional<GridIndex3D> goal_cell = grid.worldToCell(goal);
  const GridBounds3D& bounds = grid.bounds();
  const auto same = [](const GridIndex3D first, const GridIndex3D second) noexcept {
    return first.x == second.x && first.y == second.y && first.z == second.z;
  };
  VisitedVoxels visited;
  std::vector<GridIndex3D> frontier{*start};
  std::vector<GridIndex3D> next;
  static_cast<void>(visited.markVisited(*start));
  proof.component_voxels = 1U;
  if (goal_cell.has_value() && same(*goal_cell, *start)) {
    proof.goal_inside = true;
    return proof;
  }
  constexpr std::array<std::array<int, 3>, 6> kNeighbours{
      {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
  while (!frontier.empty()) {
    next.clear();
    for (const GridIndex3D cell : frontier) {
      for (const std::array<int, 3>& step : kNeighbours) {
        const GridIndex3D neighbour{cell.x + step[0], cell.y + step[1],
                                    cell.z + step[2]};
        if (neighbour.x < 0 || neighbour.y < 0 || neighbour.z < 0 ||
            neighbour.x >= bounds.width_cells || neighbour.y >= bounds.height_cells ||
            neighbour.z >= bounds.depth_cells) {
          proof.touches_grid_edge = true;
          return proof;
        }
        if (grid.isOccupied(neighbour) || !visited.markVisited(neighbour)) {
          continue;
        }
        ++proof.component_voxels;
        if (goal_cell.has_value() && same(*goal_cell, neighbour)) {
          proof.goal_inside = true;
          return proof;
        }
        if (proof.component_voxels > voxel_budget) {
          proof.budget_exhausted = true;
          return proof;
        }
        next.push_back(neighbour);
      }
    }
    std::swap(frontier, next);
  }
  return proof;
}

const char*
goalReachabilityProofVerdict(const GoalReachabilityProof3D& proof) noexcept {
  if (!proof.vehicle_inside_grid) {
    return "vehicle_outside_grid";
  }
  if (proof.goal_inside) {
    return "goal_reachable";
  }
  if (proof.touches_grid_edge) {
    return "open_at_grid_edge";
  }
  if (proof.budget_exhausted) {
    return "budget_exhausted";
  }
  return "proven_unreachable";
}

} // namespace drone_city_nav

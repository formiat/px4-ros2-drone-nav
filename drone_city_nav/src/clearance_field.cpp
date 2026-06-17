#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr std::array<GridIndex, 8> kNeighborOffsets{{
    {-1, -1},
    {0, -1},
    {1, -1},
    {-1, 0},
    {1, 0},
    {-1, 1},
    {0, 1},
    {1, 1},
}};

struct ClearanceNode {
  GridIndex cell{};
  double distance_m{0.0};
};

struct CompareClearanceNode {
  [[nodiscard]] bool operator()(const ClearanceNode& lhs,
                                const ClearanceNode& rhs) const noexcept {
    return lhs.distance_m > rhs.distance_m;
  }
};

[[nodiscard]] double stepDistanceM(const GridIndex offset,
                                   const double resolution_m) noexcept {
  const double cell_distance =
      (offset.x != 0 && offset.y != 0) ? std::numbers::sqrt2 : 1.0;
  return cell_distance * resolution_m;
}

[[nodiscard]] bool isSourceCell(const OccupancyGrid2D& grid, const GridIndex cell,
                                const ClearanceSource source) {
  switch (source) {
    case ClearanceSource::kOccupied:
      return grid.isOccupied(cell);
    case ClearanceSource::kProhibited:
      return grid.isProhibited(cell);
  }
  return false;
}

[[nodiscard]] std::size_t linearIndex(const GridBounds& bounds, const GridIndex cell) {
  if (cell.x < 0 || cell.y < 0 || cell.x >= bounds.width_cells ||
      cell.y >= bounds.height_cells) {
    throw std::out_of_range{"Clearance field cell is outside grid bounds"};
  }
  return static_cast<std::size_t>(cell.y) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

} // namespace

ClearanceField2D ClearanceField2D::build(const OccupancyGrid2D& grid,
                                         const double max_distance_m,
                                         const ClearanceSource source) {
  ClearanceField2D field{};
  field.bounds_ = grid.bounds();
  field.max_distance_m_ = std::max(0.0, max_distance_m);
  field.distance_m_.assign(grid.cellCount(), std::numeric_limits<double>::infinity());
  if (field.max_distance_m_ <= 0.0) {
    return field;
  }

  // Weighted 8-neighbor propagation is a bounded metric approximation: diagonal
  // steps cost sqrt(2) cells, so clearance is expressed in meters instead of
  // raw grid hops.
  std::priority_queue<ClearanceNode, std::vector<ClearanceNode>, CompareClearanceNode>
      queue;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (!isSourceCell(grid, cell, source)) {
        continue;
      }
      field.distance_m_[grid.linearIndex(cell)] = 0.0;
      queue.push(ClearanceNode{cell, 0.0});
    }
  }

  while (!queue.empty()) {
    const ClearanceNode current = queue.top();
    queue.pop();
    if (current.distance_m > field.max_distance_m_) {
      continue;
    }
    const std::size_t current_index = grid.linearIndex(current.cell);
    if (current.distance_m > field.distance_m_[current_index]) {
      continue;
    }

    for (const GridIndex offset : kNeighborOffsets) {
      const GridIndex next{current.cell.x + offset.x, current.cell.y + offset.y};
      if (!grid.contains(next)) {
        continue;
      }
      const double next_distance_m =
          current.distance_m + stepDistanceM(offset, grid.resolution());
      if (next_distance_m > field.max_distance_m_) {
        continue;
      }
      const std::size_t next_index = grid.linearIndex(next);
      if (field.distance_m_[next_index] <= next_distance_m) {
        continue;
      }
      field.distance_m_[next_index] = next_distance_m;
      queue.push(ClearanceNode{next, next_distance_m});
    }
  }

  return field;
}

const GridBounds& ClearanceField2D::bounds() const noexcept {
  return bounds_;
}

double ClearanceField2D::maxDistanceM() const noexcept {
  return max_distance_m_;
}

bool ClearanceField2D::contains(const GridIndex cell) const noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.x < bounds_.width_cells &&
         cell.y < bounds_.height_cells;
}

double ClearanceField2D::distanceAt(const GridIndex cell) const {
  return distance_m_.at(linearIndex(bounds_, cell));
}

std::span<const double> ClearanceField2D::distancesM() const noexcept {
  return distance_m_;
}

} // namespace drone_city_nav

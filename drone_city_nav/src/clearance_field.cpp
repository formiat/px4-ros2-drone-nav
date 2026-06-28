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

[[nodiscard]] bool sameBounds(const GridBounds& lhs, const GridBounds& rhs) noexcept {
  return lhs.origin_x == rhs.origin_x && lhs.origin_y == rhs.origin_y &&
         lhs.resolution_m == rhs.resolution_m && lhs.width_cells == rhs.width_cells &&
         lhs.height_cells == rhs.height_cells;
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

ClearanceFieldCacheLookup
ClearanceFieldCache::getOrBuild(const OccupancyGrid2D& grid,
                                const double max_distance_m,
                                const ClearanceSource source) {
  if (matches(grid, max_distance_m, source) && field_.has_value()) {
    return ClearanceFieldCacheLookup{&*field_, true};
  }

  const OccupancyGridFingerprint fingerprint = grid.prohibitedFingerprint();
  bounds_ = fingerprint.bounds;
  max_distance_m_ = std::max(0.0, max_distance_m);
  source_ = source;
  cells_hash_ = fingerprint.cells_hash;
  inflated_hash_ = fingerprint.inflated_hash;
  const std::span<const CellState> cells = grid.cells();
  cells_snapshot_.assign(cells.begin(), cells.end());
  const std::span<const std::uint8_t> inflated_cells = grid.inflatedCells();
  inflated_snapshot_.assign(inflated_cells.begin(), inflated_cells.end());
  field_ = ClearanceField2D::build(grid, max_distance_m_, source);
  return ClearanceFieldCacheLookup{&*field_, false};
}

void ClearanceFieldCache::clear() noexcept {
  bounds_ = GridBounds{};
  max_distance_m_ = 0.0;
  source_ = ClearanceSource::kOccupied;
  cells_hash_ = 0U;
  inflated_hash_ = 0U;
  cells_snapshot_.clear();
  inflated_snapshot_.clear();
  field_.reset();
}

bool ClearanceFieldCache::matches(const OccupancyGrid2D& grid,
                                  const double max_distance_m,
                                  const ClearanceSource source) const noexcept {
  if (!field_.has_value() || source_ != source ||
      max_distance_m_ != std::max(0.0, max_distance_m) ||
      !sameBounds(bounds_, grid.bounds())) {
    return false;
  }

  const OccupancyGridFingerprint fingerprint = grid.prohibitedFingerprint();
  if (cells_hash_ != fingerprint.cells_hash ||
      inflated_hash_ != fingerprint.inflated_hash) {
    return false;
  }

  const std::span<const CellState> cells = grid.cells();
  const std::span<const std::uint8_t> inflated_cells = grid.inflatedCells();
  return std::ranges::equal(cells_snapshot_, cells) &&
         std::ranges::equal(inflated_snapshot_, inflated_cells);
}

} // namespace drone_city_nav

#include "drone_city_nav/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validBounds(const GridBounds &bounds) noexcept {
  return bounds.resolution_m > 0.0 && bounds.width_cells > 0 &&
         bounds.height_cells > 0;
}

} // namespace

OccupancyGrid2D::OccupancyGrid2D(GridBounds bounds) : bounds_{bounds} {
  if (!validBounds(bounds_)) {
    throw std::invalid_argument{
        "OccupancyGrid2D requires positive resolution, width, and height"};
  }

  const auto width = static_cast<std::size_t>(bounds_.width_cells);
  const auto height = static_cast<std::size_t>(bounds_.height_cells);
  cells_.assign(width * height, CellState::kUnknown);
  inflated_.assign(width * height, 0U);
}

const GridBounds &OccupancyGrid2D::bounds() const noexcept { return bounds_; }

int OccupancyGrid2D::width() const noexcept { return bounds_.width_cells; }

int OccupancyGrid2D::height() const noexcept { return bounds_.height_cells; }

double OccupancyGrid2D::resolution() const noexcept {
  return bounds_.resolution_m;
}

double OccupancyGrid2D::originX() const noexcept { return bounds_.origin_x; }

double OccupancyGrid2D::originY() const noexcept { return bounds_.origin_y; }

std::size_t OccupancyGrid2D::cellCount() const noexcept {
  return cells_.size();
}

bool OccupancyGrid2D::contains(const GridIndex cell) const noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.x < bounds_.width_cells &&
         cell.y < bounds_.height_cells;
}

std::optional<GridIndex>
OccupancyGrid2D::worldToCell(const Point2 point) const noexcept {
  const auto x = static_cast<int>(
      std::floor((point.x - bounds_.origin_x) / bounds_.resolution_m));
  const auto y = static_cast<int>(
      std::floor((point.y - bounds_.origin_y) / bounds_.resolution_m));
  const GridIndex cell{x, y};
  if (!contains(cell)) {
    return std::nullopt;
  }
  return cell;
}

Point2 OccupancyGrid2D::cellCenter(const GridIndex cell) const noexcept {
  return Point2{bounds_.origin_x +
                    (static_cast<double>(cell.x) + 0.5) * bounds_.resolution_m,
                bounds_.origin_y +
                    (static_cast<double>(cell.y) + 0.5) * bounds_.resolution_m};
}

std::size_t OccupancyGrid2D::linearIndex(const GridIndex cell) const {
  if (!contains(cell)) {
    throw std::out_of_range{"Grid cell is outside occupancy grid bounds"};
  }
  const auto x = static_cast<std::size_t>(cell.x);
  const auto y = static_cast<std::size_t>(cell.y);
  return y * static_cast<std::size_t>(bounds_.width_cells) + x;
}

CellState OccupancyGrid2D::state(const GridIndex cell) const {
  return cells_.at(linearIndex(cell));
}

bool OccupancyGrid2D::isOccupied(const GridIndex cell) const {
  return state(cell) == CellState::kOccupied;
}

bool OccupancyGrid2D::isInflated(const GridIndex cell) const {
  return inflated_.at(linearIndex(cell)) != 0U;
}

bool OccupancyGrid2D::isBlocked(const GridIndex cell) const {
  return isOccupied(cell) || isInflated(cell);
}

std::span<const CellState> OccupancyGrid2D::cells() const noexcept {
  return cells_;
}

void OccupancyGrid2D::reset(const CellState value) {
  std::fill(cells_.begin(), cells_.end(), value);
  std::fill(inflated_.begin(), inflated_.end(), 0U);
}

void OccupancyGrid2D::setFree(const GridIndex cell) {
  if (!contains(cell)) {
    return;
  }

  cells_[linearIndex(cell)] = CellState::kFree;
}

void OccupancyGrid2D::setOccupied(const GridIndex cell) {
  if (!contains(cell)) {
    return;
  }
  cells_[linearIndex(cell)] = CellState::kOccupied;
}

void OccupancyGrid2D::markRay(const Point2 start, const Point2 end,
                              const bool endpoint_occupied) {
  const auto start_cell = worldToCell(start);
  const auto end_cell = worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return;
  }

  const auto ray_cells = cellsOnLine(*start_cell, *end_cell);
  if (ray_cells.empty()) {
    return;
  }

  const auto free_end =
      endpoint_occupied ? ray_cells.size() - 1U : ray_cells.size();
  for (std::size_t i = 0; i < free_end; ++i) {
    setFree(ray_cells[i]);
  }

  if (endpoint_occupied) {
    setOccupied(ray_cells.back());
  }
}

void OccupancyGrid2D::rebuildInflation(const double radius_m) {
  std::fill(inflated_.begin(), inflated_.end(), 0U);
  if (radius_m <= 0.0) {
    return;
  }

  const int radius_cells =
      static_cast<int>(std::ceil(radius_m / bounds_.resolution_m));
  const double radius_with_margin = radius_m + (0.5 * bounds_.resolution_m);
  const double radius_sq = radius_with_margin * radius_with_margin;

  for (int y = 0; y < bounds_.height_cells; ++y) {
    for (int x = 0; x < bounds_.width_cells; ++x) {
      const GridIndex obstacle{x, y};
      if (!isOccupied(obstacle)) {
        continue;
      }

      const Point2 obstacle_center = cellCenter(obstacle);
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const GridIndex candidate{x + dx, y + dy};
          if (!contains(candidate)) {
            continue;
          }
          if (squaredDistance(obstacle_center, cellCenter(candidate)) <=
              radius_sq) {
            inflated_[linearIndex(candidate)] = 1U;
          }
        }
      }
    }
  }
}

std::vector<GridIndex> OccupancyGrid2D::cellsOnLine(const GridIndex start,
                                                    const GridIndex end) const {
  std::vector<GridIndex> cells;

  int x0 = start.x;
  int y0 = start.y;
  const int x1 = end.x;
  const int y1 = end.y;

  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    const GridIndex cell{x0, y0};
    if (contains(cell)) {
      cells.push_back(cell);
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }

  return cells;
}

std::optional<GridIndex>
OccupancyGrid2D::nearestUnblocked(const GridIndex seed,
                                  const int max_radius_cells) const {
  if (!contains(seed)) {
    return std::nullopt;
  }
  if (!isBlocked(seed)) {
    return seed;
  }

  const int bounded_radius = std::max(0, max_radius_cells);
  for (int radius = 1; radius <= bounded_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        const GridIndex candidate{seed.x + dx, seed.y + dy};
        if (contains(candidate) && !isBlocked(candidate)) {
          return candidate;
        }
      }
    }
  }

  return std::nullopt;
}

} // namespace drone_city_nav

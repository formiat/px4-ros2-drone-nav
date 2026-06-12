#include "drone_city_nav/obstacle_memory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace drone_city_nav {
namespace {

struct ClippedSegment {
  Point2 end{};
  bool clipped{false};
};

[[nodiscard]] bool finitePose(const Pose2& pose) noexcept {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.yaw_rad);
}

[[nodiscard]] bool validMemoryConfig(const ObstacleMemoryConfig& config) noexcept {
  return config.max_lidar_range_m > 0.0 && config.scan_stride > 0 &&
         config.hit_weight > 0 && config.miss_weight > 0 &&
         config.min_score < config.max_score &&
         config.free_score < config.occupied_score;
}

[[nodiscard]] Point2 directionFromAngle(const double angle_rad,
                                        const bool swap_xy) noexcept {
  const double scan_x = std::cos(angle_rad);
  const double scan_y = std::sin(angle_rad);
  if (swap_xy) {
    return Point2{scan_y, scan_x};
  }
  return Point2{scan_x, scan_y};
}

[[nodiscard]] std::optional<ClippedSegment>
clipSegmentToGrid(const OccupancyGrid2D& grid, const Point2 start,
                  const Point2 end) noexcept {
  const double min_x = grid.originX();
  const double min_y = grid.originY();
  const double max_x =
      grid.originX() + static_cast<double>(grid.width()) * grid.resolution();
  const double max_y =
      grid.originY() + static_cast<double>(grid.height()) * grid.resolution();
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  double t0 = 0.0;
  double t1 = 1.0;

  const auto clip_axis = [&t0, &t1](const double p, const double q) noexcept {
    if (p == 0.0) {
      return q >= 0.0;
    }
    const double r = q / p;
    if (p < 0.0) {
      if (r > t1) {
        return false;
      }
      t0 = std::max(t0, r);
      return true;
    }
    if (r < t0) {
      return false;
    }
    t1 = std::min(t1, r);
    return true;
  };

  if (!clip_axis(-dx, start.x - min_x) || !clip_axis(dx, max_x - start.x) ||
      !clip_axis(-dy, start.y - min_y) || !clip_axis(dy, max_y - start.y)) {
    return std::nullopt;
  }
  if (t1 < 0.0 || t0 > 1.0 || t0 > t1) {
    return std::nullopt;
  }

  constexpr double kBoundaryEpsilon = 1.0e-9;
  Point2 clipped_end{start.x + t1 * dx, start.y + t1 * dy};
  clipped_end.x = std::clamp(clipped_end.x, min_x, max_x - kBoundaryEpsilon);
  clipped_end.y = std::clamp(clipped_end.y, min_y, max_y - kBoundaryEpsilon);
  return ClippedSegment{clipped_end, t1 < 1.0 - kBoundaryEpsilon};
}

[[nodiscard]] bool rangeIsPositiveInfinity(const float value) noexcept {
  return std::isinf(value) && value > 0.0F;
}

[[nodiscard]] GridCellCounts countCells(const OccupancyGrid2D& grid) {
  GridCellCounts counts{};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (grid.isInflated(cell)) {
        ++counts.inflated_cells;
      }
      switch (grid.state(cell)) {
        case CellState::kUnknown:
          ++counts.unknown_cells;
          break;
        case CellState::kFree:
          ++counts.free_cells;
          break;
        case CellState::kOccupied:
          ++counts.occupied_cells;
          break;
      }
    }
  }
  return counts;
}

} // namespace

ObstacleMemoryGrid::ObstacleMemoryGrid(const GridBounds& bounds)
    : raw_grid_{bounds},
      inflated_grid_{bounds},
      scores_(raw_grid_.cellCount(), 0) {
}

ObstacleMemoryStats
ObstacleMemoryGrid::integrateScan(const Pose2& pose, const LaserScan2DView& scan,
                                  const ObstacleMemoryConfig& config) {
  ObstacleMemoryStats stats{};
  if (!finitePose(pose) || !validMemoryConfig(config) ||
      !std::isfinite(scan.angle_increment_rad) || scan.angle_increment_rad == 0.0 ||
      scan.ranges.empty() || !raw_grid_.worldToCell(pose.position).has_value()) {
    return stats;
  }

  const double scan_range_max = std::min(scan.range_max_m, config.max_lidar_range_m);
  if (!(scan_range_max > scan.range_min_m)) {
    return stats;
  }

  const auto stride = static_cast<std::size_t>(std::max(1, config.scan_stride));
  for (std::size_t i = 0U; i < scan.ranges.size(); i += stride) {
    const float raw_range = scan.ranges[i];
    const bool finite_range = std::isfinite(raw_range);
    if ((!finite_range && !rangeIsPositiveInfinity(raw_range)) ||
        (finite_range && static_cast<double>(raw_range) < scan.range_min_m)) {
      ++stats.invalid_ranges;
      continue;
    }

    const bool hit = finite_range && static_cast<double>(raw_range) <
                                         scan_range_max - config.range_hit_epsilon_m;
    const double range_m = hit ? static_cast<double>(raw_range) : scan_range_max;
    const double angle_rad = pose.yaw_rad + scan.scan_yaw_offset_rad +
                             scan.angle_min_rad +
                             static_cast<double>(i) * scan.angle_increment_rad;
    const Point2 direction =
        directionFromAngle(angle_rad, scan.swap_lidar_xy_to_local_frame);
    const Point2 endpoint{pose.position.x + range_m * direction.x,
                          pose.position.y + range_m * direction.y};
    const auto clipped = clipSegmentToGrid(raw_grid_, pose.position, endpoint);
    if (!clipped.has_value()) {
      ++stats.invalid_ranges;
      continue;
    }

    const auto start_cell = raw_grid_.worldToCell(pose.position);
    const auto clipped_end_cell = raw_grid_.worldToCell(clipped->end);
    if (!start_cell.has_value() || !clipped_end_cell.has_value()) {
      ++stats.invalid_ranges;
      continue;
    }

    ++stats.processed_beams;
    if (hit) {
      ++stats.hit_beams;
    }
    if (clipped->clipped) {
      ++stats.clipped_rays;
    }

    const auto endpoint_cell = raw_grid_.worldToCell(endpoint);
    const bool hit_endpoint_inside = hit && endpoint_cell.has_value();
    const std::vector<GridIndex> ray_cells =
        raw_grid_.cellsOnLine(*start_cell, *clipped_end_cell);
    const std::size_t free_end = hit_endpoint_inside && !ray_cells.empty()
                                     ? ray_cells.size() - 1U
                                     : ray_cells.size();
    for (std::size_t ray_index = 0U; ray_index < free_end; ++ray_index) {
      applyMiss(ray_cells[ray_index], config);
      ++stats.free_cells_updated;
    }

    if (!hit) {
      continue;
    }
    if (!endpoint_cell.has_value()) {
      ++stats.outside_hit_endpoints;
      continue;
    }

    const GridIndex endpoint_grid_cell =
        endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
    applyHit(endpoint_grid_cell, config);
    ++stats.occupied_cells_updated;

    if (config.hit_obstacle_depth_m <= 0.0) {
      continue;
    }

    const Point2 depth_start = endpoint;
    const Point2 depth_end{depth_start.x + config.hit_obstacle_depth_m * direction.x,
                           depth_start.y + config.hit_obstacle_depth_m * direction.y};
    const auto clipped_depth = clipSegmentToGrid(raw_grid_, depth_start, depth_end);
    if (!clipped_depth.has_value()) {
      continue;
    }
    const auto depth_end_cell = raw_grid_.worldToCell(clipped_depth->end);
    if (!depth_end_cell.has_value()) {
      continue;
    }
    const GridIndex depth_grid_cell = depth_end_cell.value();
    const std::vector<GridIndex> depth_cells =
        raw_grid_.cellsOnLine(endpoint_grid_cell, depth_grid_cell);
    for (const GridIndex cell : depth_cells) {
      applyHit(cell, config);
      ++stats.obstacle_depth_cells;
    }
  }

  return stats;
}

void ObstacleMemoryGrid::rebuildInflation(const double radius_m) {
  inflated_grid_ = raw_grid_;
  inflated_grid_.rebuildInflation(radius_m);
}

const OccupancyGrid2D& ObstacleMemoryGrid::rawGrid() const noexcept {
  return raw_grid_;
}

const OccupancyGrid2D& ObstacleMemoryGrid::inflatedGrid() const noexcept {
  return inflated_grid_;
}

OccupancyGrid2D ObstacleMemoryGrid::localWindow(const Point2 center,
                                                const double radius_m) const {
  const double bounded_radius = std::max(radius_m, raw_grid_.resolution());
  const int width_cells = std::max(
      1, static_cast<int>(std::ceil((2.0 * bounded_radius) / raw_grid_.resolution())));
  OccupancyGrid2D window{GridBounds{center.x - bounded_radius,
                                    center.y - bounded_radius, raw_grid_.resolution(),
                                    width_cells, width_cells}};

  for (int y = 0; y < window.height(); ++y) {
    for (int x = 0; x < window.width(); ++x) {
      const GridIndex local_cell{x, y};
      const auto source_cell = raw_grid_.worldToCell(window.cellCenter(local_cell));
      if (!source_cell.has_value()) {
        continue;
      }
      switch (raw_grid_.state(*source_cell)) {
        case CellState::kUnknown:
          break;
        case CellState::kFree:
          window.setFree(local_cell);
          break;
        case CellState::kOccupied:
          window.setOccupied(local_cell);
          break;
      }
    }
  }

  return window;
}

GridCellCounts ObstacleMemoryGrid::countRawCells() const {
  return countCells(raw_grid_);
}

GridCellCounts ObstacleMemoryGrid::countInflatedCells() const {
  return countCells(inflated_grid_);
}

void ObstacleMemoryGrid::applyMiss(const GridIndex cell,
                                   const ObstacleMemoryConfig& config) {
  if (!raw_grid_.contains(cell)) {
    return;
  }
  const std::size_t index = raw_grid_.linearIndex(cell);
  scores_[index] = std::clamp(scores_[index] - config.miss_weight, config.min_score,
                              config.max_score);
  syncCellState(cell, config);
}

void ObstacleMemoryGrid::applyHit(const GridIndex cell,
                                  const ObstacleMemoryConfig& config) {
  if (!raw_grid_.contains(cell)) {
    return;
  }
  const std::size_t index = raw_grid_.linearIndex(cell);
  scores_[index] = std::clamp(scores_[index] + config.hit_weight, config.min_score,
                              config.max_score);
  syncCellState(cell, config);
}

void ObstacleMemoryGrid::syncCellState(const GridIndex cell,
                                       const ObstacleMemoryConfig& config) {
  const int score = scores_.at(raw_grid_.linearIndex(cell));
  if (score >= config.occupied_score) {
    raw_grid_.setOccupied(cell);
    return;
  }
  if (score <= config.free_score) {
    raw_grid_.setFree(cell);
    return;
  }
  raw_grid_.setUnknown(cell);
}

} // namespace drone_city_nav

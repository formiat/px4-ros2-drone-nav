#include "drone_city_nav/obstacle_memory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr std::size_t kMaxRetainedKnownStaticHitDiagnostics{16U};

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
         config.min_score < config.max_score && config.free_score >= config.min_score &&
         config.occupied_score <= config.max_score &&
         config.free_score < config.occupied_score;
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
      scores_(raw_grid_.cellCount(), 0) {
}

ObstacleMemoryStats
ObstacleMemoryGrid::integrateScan(const Pose2& pose, const LaserScan2DView& scan,
                                  const ObstacleMemoryConfig& config,
                                  const KnownStaticLidarHitClassifier* classifier) {
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

  const LidarProjectionPose projection_pose{
      pose.position,  scan.origin_altitude_m, pose.yaw_rad,       scan.roll_rad,
      scan.pitch_rad, scan.altitude_valid,    scan.attitude_valid};
  const LidarProjectionConfig projection_config{
      config.max_lidar_range_m,      config.range_hit_epsilon_m,
      scan.scan_yaw_offset_rad,      scan.lidar_z_offset_m,
      scan.min_projected_altitude_m, scan.max_projected_altitude_m,
      scan.compensate_attitude,      scan.lidar_mount_roll_rad,
      scan.lidar_mount_pitch_rad,    scan.lidar_mount_yaw_rad};

  const auto stride = static_cast<std::size_t>(std::max(1, config.scan_stride));
  for (std::size_t i = 0U; i < scan.ranges.size(); i += stride) {
    const float raw_range = scan.ranges[i];
    if (!lidarRawRangeUsable(raw_range, scan.range_min_m)) {
      ++stats.invalid_ranges;
      continue;
    }

    const LidarBeamProjection projection = projectLidarBeam(
        projection_pose, projection_config, scan.range_min_m, scan_range_max,
        scan.angle_min_rad, scan.angle_increment_rad, i, raw_range);
    if (projection.status == LidarBeamProjectionStatus::kAltitudeRejected) {
      ++stats.altitude_rejected_beams;
      continue;
    }
    if (projection.status != LidarBeamProjectionStatus::kAccepted) {
      ++stats.invalid_ranges;
      continue;
    }

    const auto clipped =
        clipSegmentToGrid(raw_grid_, pose.position, projection.endpoint);
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
    if (projection.hit) {
      ++stats.hit_beams;
    }
    if (clipped->clipped) {
      ++stats.clipped_rays;
    }

    const auto endpoint_cell = raw_grid_.worldToCell(projection.endpoint);
    const bool hit_endpoint_inside = projection.hit && endpoint_cell.has_value();
    const std::vector<GridIndex> ray_cells =
        raw_grid_.cellsOnLine(*start_cell, *clipped_end_cell);
    const std::size_t free_end = hit_endpoint_inside && !ray_cells.empty()
                                     ? ray_cells.size() - 1U
                                     : ray_cells.size();
    for (std::size_t ray_index = 0U; ray_index < free_end; ++ray_index) {
      applyMiss(ray_cells[ray_index], config);
      ++stats.free_cells_updated;
    }

    if (!projection.hit) {
      continue;
    }
    if (!endpoint_cell.has_value()) {
      ++stats.outside_hit_endpoints;
      continue;
    }

    if (classifier != nullptr) {
      const KnownStaticLidarHitResult classification =
          classifier->classify(projection.ray_origin_map_m,
                               projection.ray_direction_map, projection.used_range_m);
      recordKnownStaticLidarHit(classification, stats.known_static_lidar);
      if (classification.classification ==
          KnownStaticLidarHitClassification::kExpectedStatic) {
        continue;
      }
      if (stats.retained_known_static_hits.size() <
          kMaxRetainedKnownStaticHitDiagnostics) {
        if (const std::optional<KnownStaticLidarHitProvenance> provenance =
                makeKnownStaticLidarHitProvenance(
                    classification,
                    Point3{projection.endpoint.x, projection.endpoint.y,
                           projection.endpoint_altitude_m},
                    endpoint_cell->x, endpoint_cell->y);
            provenance.has_value()) {
          stats.retained_known_static_hits.push_back(*provenance);
        }
      }
    }

    const GridIndex endpoint_grid_cell =
        endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)
    applyHit(endpoint_grid_cell, config);
    ++stats.occupied_cells_updated;
  }

  return stats;
}

void ObstacleMemoryGrid::reset() {
  raw_grid_ = OccupancyGrid2D{raw_grid_.bounds()};
  std::fill(scores_.begin(), scores_.end(), 0);
}

const OccupancyGrid2D& ObstacleMemoryGrid::rawGrid() const noexcept {
  return raw_grid_;
}

GridCellCounts ObstacleMemoryGrid::countRawCells() const {
  return countCells(raw_grid_);
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

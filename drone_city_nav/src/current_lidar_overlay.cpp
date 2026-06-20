#include "drone_city_nav/current_lidar_overlay.hpp"

#include "drone_city_nav/grid_overlay.hpp"

#include <algorithm>

namespace drone_city_nav {

std::size_t markCurrentLidarObstacle(OccupancyGrid2D& grid, const Point2 endpoint) {
  const auto endpoint_cell = grid.worldToCell(endpoint);
  if (!endpoint_cell.has_value()) {
    return 0U;
  }

  grid.setOccupied(*endpoint_cell);
  return 1U;
}

CurrentLidarOverlayStats
overlayCurrentLidarHits(OccupancyGrid2D& grid, const LidarScanView& scan,
                        const LidarProjectionPose& projection_pose,
                        const LidarProjectionConfig& projection_config) {
  CurrentLidarOverlayStats stats{};
  stats.used = true;

  const double scan_range_max =
      std::min(scan.range_max_m, projection_config.max_lidar_range_m);
  if (!(scan_range_max > 0.0) || scan.angle_increment_rad == 0.0) {
    return stats;
  }

  OccupancyGrid2D current_lidar_grid{grid.bounds()};
  for (std::size_t i = 0U; i < scan.ranges.size(); ++i) {
    const float raw_range = scan.ranges[i];
    if (!lidarRawRangeUsable(raw_range, scan.range_min_m)) {
      continue;
    }
    ++stats.processed_beams;

    const LidarBeamProjection projection = projectLidarBeam(
        projection_pose, projection_config, scan.range_min_m, scan_range_max,
        scan.angle_min_rad, scan.angle_increment_rad, i, raw_range);
    if (projection.status == LidarBeamProjectionStatus::kAltitudeRejected) {
      ++stats.altitude_rejected_beams;
      continue;
    }
    if (projection.status != LidarBeamProjectionStatus::kAccepted || !projection.hit) {
      continue;
    }

    ++stats.hit_beams;
    const std::size_t occupied_cells =
        markCurrentLidarObstacle(current_lidar_grid, projection.endpoint);
    if (occupied_cells == 0U) {
      ++stats.outside_hits;
    } else {
      stats.occupied_cells += occupied_cells;
    }
  }

  const GridOverlayStats overlay_stats =
      overlayCurrentLidarCells(grid, current_lidar_grid);
  stats.occupied_cells = overlay_stats.source_occupied_cells;
  stats.overlay_occupied_cells_applied = overlay_stats.occupied_cells_applied;
  stats.overlay_occupied_cells_preserved = overlay_stats.occupied_cells_preserved;
  return stats;
}

} // namespace drone_city_nav

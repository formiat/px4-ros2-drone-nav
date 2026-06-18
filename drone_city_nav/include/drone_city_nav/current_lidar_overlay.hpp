#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav {

struct LidarScanView {
  std::span<const float> ranges;
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
};

struct CurrentLidarOverlayStats {
  bool enabled{false};
  bool used{false};
  bool fresh{false};
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t altitude_rejected_beams{0U};
  std::size_t occupied_cells{0U};
  std::size_t overlay_occupied_cells_applied{0U};
  std::size_t overlay_occupied_cells_preserved{0U};
  std::size_t outside_hits{0U};
};

[[nodiscard]] std::size_t markCurrentLidarObstacle(OccupancyGrid2D& grid,
                                                   Point2 endpoint,
                                                   Point2 depth_endpoint,
                                                   double obstacle_depth_m);

[[nodiscard]] CurrentLidarOverlayStats
overlayCurrentLidarHits(OccupancyGrid2D& grid, const LidarScanView& scan,
                        const LidarProjectionPose& projection_pose,
                        const LidarProjectionConfig& projection_config,
                        double obstacle_depth_m);

} // namespace drone_city_nav

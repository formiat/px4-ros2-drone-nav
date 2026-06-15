#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

struct LaserScan2DView {
  std::span<const float> ranges{};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double scan_yaw_offset_rad{0.0};
  double origin_altitude_m{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double lidar_z_offset_m{0.0};
  double min_projected_altitude_m{0.0};
  double max_projected_altitude_m{100000.0};
  bool swap_lidar_xy_to_local_frame{false};
  bool altitude_valid{false};
  bool attitude_valid{false};
  bool compensate_attitude{false};
  double lidar_mount_roll_rad{0.0};
  double lidar_mount_pitch_rad{0.0};
  double lidar_mount_yaw_rad{0.0};
};

struct ObstacleMemoryConfig {
  double max_lidar_range_m{35.0};
  double range_hit_epsilon_m{0.05};
  double hit_obstacle_depth_m{0.0};
  int scan_stride{1};
  int hit_weight{4};
  int miss_weight{1};
  int min_score{-8};
  int max_score{12};
  int occupied_score{3};
  int free_score{-1};
};

struct ObstacleMemoryStats {
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t invalid_ranges{0U};
  std::size_t altitude_rejected_beams{0U};
  std::size_t clipped_rays{0U};
  std::size_t outside_hit_endpoints{0U};
  std::size_t free_cells_updated{0U};
  std::size_t occupied_cells_updated{0U};
  std::size_t obstacle_depth_cells{0U};
};

struct GridCellCounts {
  std::size_t unknown_cells{0U};
  std::size_t free_cells{0U};
  std::size_t occupied_cells{0U};
  std::size_t inflated_cells{0U};
};

class ObstacleMemoryGrid {
public:
  explicit ObstacleMemoryGrid(const GridBounds& bounds);

  [[nodiscard]] ObstacleMemoryStats integrateScan(const Pose2& pose,
                                                  const LaserScan2DView& scan,
                                                  const ObstacleMemoryConfig& config);

  void rebuildInflation(double radius_m);

  [[nodiscard]] const OccupancyGrid2D& rawGrid() const noexcept;
  [[nodiscard]] const OccupancyGrid2D& inflatedGrid() const noexcept;
  [[nodiscard]] GridCellCounts countRawCells() const;
  [[nodiscard]] GridCellCounts countInflatedCells() const;

private:
  void applyMiss(GridIndex cell, const ObstacleMemoryConfig& config);
  void applyHit(GridIndex cell, const ObstacleMemoryConfig& config);
  void syncCellState(GridIndex cell, const ObstacleMemoryConfig& config);

  OccupancyGrid2D raw_grid_;
  OccupancyGrid2D inflated_grid_;
  std::vector<int> scores_;
};

} // namespace drone_city_nav

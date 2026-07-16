#pragma once

#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <string>
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
  bool altitude_valid{false};
  bool attitude_valid{false};
  bool compensate_attitude{true};
  double lidar_mount_roll_rad{0.0};
  double lidar_mount_pitch_rad{0.0};
  double lidar_mount_yaw_rad{0.0};
};

struct ObstacleMemoryConfig {
  double max_lidar_range_m{35.0};
  double range_hit_epsilon_m{0.05};
  int scan_stride{1};
  int hit_weight{4};
  int miss_weight{1};
  int min_score{-8};
  int max_score{12};
  int occupied_score{3};
  int free_score{-1};
};

struct ObstacleMemoryOccupiedTransition {
  std::size_t beam_index{0U};
  GridIndex cell{};
  Point3 ray_origin_map_m{};
  Point3 ray_direction_map{};
  Point3 endpoint_map_m{};
  double measured_range_m{std::numeric_limits<double>::quiet_NaN()};
  int score_before{0};
  int score_after{0};
  bool classifier_applied{false};
  KnownStaticLidarHitClassification classification{
      KnownStaticLidarHitClassification::kAmbiguous};
  bool volume_matched{false};
  bool confident_face_interior{false};
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
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
  std::size_t newly_occupied_cells{0U};
  KnownStaticLidarHitStats known_static_lidar{};
  std::vector<KnownStaticLidarHitProvenance> retained_known_static_hits;
  std::vector<ObstacleMemoryOccupiedTransition> occupied_transitions;
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

  [[nodiscard]] ObstacleMemoryStats
  integrateScan(const Pose2& pose, const LaserScan2DView& scan,
                const ObstacleMemoryConfig& config,
                const KnownStaticLidarHitClassifier* classifier = nullptr);

  void reset();

  [[nodiscard]] const OccupancyGrid2D& rawGrid() const noexcept;
  [[nodiscard]] GridCellCounts countRawCells() const;

private:
  void applyMiss(GridIndex cell, const ObstacleMemoryConfig& config);
  void applyHit(GridIndex cell, const ObstacleMemoryConfig& config);
  void syncCellState(GridIndex cell, const ObstacleMemoryConfig& config);

  OccupancyGrid2D raw_grid_;
  std::vector<int> scores_;
};

} // namespace drone_city_nav

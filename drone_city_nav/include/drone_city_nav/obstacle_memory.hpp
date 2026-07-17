#pragma once

#include "drone_city_nav/lidar_beam_observation.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
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
  LaserScanTiming timing{};
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

struct AcceptedObstacleMemoryHit {
  LidarBeamObservation beam;
  KnownStaticClassificationSnapshot known_static;
  LidarIngestionDecisionSnapshot ingestion_decision;
};

struct MemoryCellProvenance {
  GridIndex cell{};
  AcceptedObstacleMemoryHit occupancy_trigger;
  AcceptedObstacleMemoryHit last_hit;
  std::optional<double> min_endpoint_z_m;
  std::optional<double> max_endpoint_z_m;
  std::uint64_t accepted_hit_count{0U};
};

struct ObstacleMemoryOccupiedTransition {
  int score_before{0};
  int score_after{0};
  MemoryCellProvenance provenance;
  LidarIngestionDecision trigger_decision{};
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
  LidarIngestionDecisionStats ingestion_decisions{};
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
                const KnownStaticLidarHitClassifier* classifier = nullptr,
                const GroundLidarRejectionConfig* ground_config = nullptr);

  void reset();

  [[nodiscard]] const OccupancyGrid2D& rawGrid() const noexcept;
  [[nodiscard]] const std::unordered_map<std::size_t, MemoryCellProvenance>&
  activeProvenance() const noexcept;
  [[nodiscard]] GridCellCounts countRawCells() const;

private:
  void applyMiss(GridIndex cell, const ObstacleMemoryConfig& config);
  [[nodiscard]] std::optional<ObstacleMemoryOccupiedTransition>
  applyAcceptedHit(GridIndex cell, const AcceptedObstacleMemoryHit& hit,
                   const LidarIngestionDecision& decision,
                   const ObstacleMemoryConfig& config);
  void syncCellState(GridIndex cell, const ObstacleMemoryConfig& config);

  OccupancyGrid2D raw_grid_;
  std::vector<int> scores_;
  std::unordered_map<std::size_t, MemoryCellProvenance> active_provenance_;
};

} // namespace drone_city_nav

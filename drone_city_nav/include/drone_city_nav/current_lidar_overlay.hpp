#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/lidar_beam_observation.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

struct LidarScanView {
  std::span<const float> ranges;
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  LaserScanTiming timing{};
  std::span<const LidarProjectionPose> beam_projection_poses{};
};

struct CurrentLidarAcceptedHitProvenance {
  GridIndex cell{};
  LidarBeamObservation observation{};
  LidarIngestionDecisionSnapshot ingestion_decision{};
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
  std::size_t timestamp_aligned_beams{0U};
  std::size_t ambiguous_hits_pending_confirmation{0U};
  std::size_t ambiguous_hits_confirmed{0U};
  KnownStaticLidarHitStats known_static_lidar{};
  LidarIngestionDecisionStats ingestion_decisions{};
  std::vector<CurrentLidarAcceptedHitProvenance> accepted_hits;
  std::vector<KnownStaticLidarHitProvenance> retained_known_static_hits;
};

[[nodiscard]] std::size_t markCurrentLidarObstacle(OccupancyGrid2D& grid,
                                                   Point2 endpoint);

[[nodiscard]] CurrentLidarOverlayStats
overlayCurrentLidarHits(OccupancyGrid2D& grid, const LidarScanView& scan,
                        const LidarProjectionPose& projection_pose,
                        const LidarProjectionConfig& projection_config,
                        const KnownStaticLidarHitClassifier* classifier = nullptr,
                        const GroundLidarRejectionConfig* ground_config = nullptr,
                        AmbiguousLidarHitTracker* ambiguous_hit_tracker = nullptr);

[[nodiscard]] const CurrentLidarAcceptedHitProvenance*
findCurrentLidarAcceptedHitProvenance(const CurrentLidarOverlayStats& stats,
                                      GridIndex cell) noexcept;

[[nodiscard]] std::string
formatCurrentLidarAcceptedHitDiagnostic(const CurrentLidarOverlayStats& stats,
                                        GridIndex cell);

} // namespace drone_city_nav

#include "drone_city_nav/current_lidar_overlay.hpp"

#include "drone_city_nav/grid_overlay.hpp"

#include <algorithm>
#include <sstream>

namespace drone_city_nav {
namespace {

constexpr std::size_t kMaxRetainedKnownStaticHitDiagnostics{16U};

void recordAcceptedHit(const GridIndex cell, const LidarBeamObservation& observation,
                       const LidarIngestionDecision& decision,
                       CurrentLidarOverlayStats& stats) {
  const auto existing =
      std::find_if(stats.accepted_hits.begin(), stats.accepted_hits.end(),
                   [cell](const CurrentLidarAcceptedHitProvenance& provenance) {
                     return provenance.cell.x == cell.x && provenance.cell.y == cell.y;
                   });
  const CurrentLidarAcceptedHitProvenance provenance{
      .cell = cell,
      .observation = observation,
      .ingestion_decision = makeLidarIngestionDecisionSnapshot(decision),
  };
  if (existing == stats.accepted_hits.end()) {
    stats.accepted_hits.push_back(provenance);
  } else {
    *existing = provenance;
  }
}

} // namespace

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
                        const LidarProjectionConfig& projection_config,
                        const KnownStaticLidarHitClassifier* classifier,
                        const GroundLidarRejectionConfig* ground_config,
                        AmbiguousLidarHitTracker* ambiguous_hit_tracker) {
  CurrentLidarOverlayStats stats{};
  stats.used = true;

  const double scan_range_max =
      std::min(scan.range_max_m, projection_config.max_lidar_range_m);
  if (!(scan_range_max > 0.0) || scan.angle_increment_rad == 0.0) {
    return stats;
  }

  OccupancyGrid2D current_lidar_grid{grid.bounds()};
  const bool aligned_poses_available =
      scan.beam_projection_poses.size() == scan.ranges.size();
  for (std::size_t i = 0U; i < scan.ranges.size(); ++i) {
    const float raw_range = scan.ranges[i];
    if (!lidarRawRangeUsable(raw_range, scan.range_min_m)) {
      continue;
    }
    ++stats.processed_beams;

    const LidarProjectionPose& beam_pose =
        aligned_poses_available ? scan.beam_projection_poses[i] : projection_pose;
    if (aligned_poses_available) {
      ++stats.timestamp_aligned_beams;
    }
    const LidarBeamProjection projection =
        projectLidarBeam(beam_pose, projection_config, scan.range_min_m, scan_range_max,
                         scan.angle_min_rad, scan.angle_increment_rad, i, raw_range);
    if (projection.status != LidarBeamProjectionStatus::kAccepted &&
        projection.status != LidarBeamProjectionStatus::kAltitudeRejected) {
      continue;
    }
    const LidarBeamObservation observation = makeLidarBeamObservation(
        scan.timing, i, projection, scan_range_max, beam_pose, projection_config,
        aligned_poses_available, scan.projection_pose_source);
    LidarIngestionDecision decision = resolveAmbiguousKnownStaticIngestion(
        observation, evaluateLidarIngestion(observation, classifier, ground_config),
        ambiguous_hit_tracker);
    decision = normalizeAcceptedLidarIngestionDecision(observation, decision,
                                                       stats.ingestion_decisions);
    const bool altitude_rejected =
        projection.status == LidarBeamProjectionStatus::kAltitudeRejected;
    recordLidarIngestionDecision(observation, decision, altitude_rejected,
                                 stats.ingestion_decisions);
    if (decision.reason == LidarIngestionReason::kAmbiguousKnownStatic) {
      ++stats.ambiguous_hits_pending_confirmation;
    } else if (decision.ambiguous_resolution != AmbiguousLidarHitResolution::kPending) {
      ++stats.ambiguous_hits_confirmed;
    }
    if (projection.status == LidarBeamProjectionStatus::kAltitudeRejected) {
      ++stats.altitude_rejected_beams;
      continue;
    }
    if (!projection.hit ||
        decision.action != LidarIngestionAction::kIntegrateFreeAndHit) {
      if (decision.known_static_result_available) {
        recordKnownStaticLidarHit(decision.known_static_result,
                                  stats.known_static_lidar, false);
      }
      continue;
    }

    ++stats.hit_beams;
    const auto endpoint_cell = current_lidar_grid.worldToCell(projection.endpoint);
    if (endpoint_cell.has_value()) {
      recordAcceptedHit(*endpoint_cell, observation, decision, stats);
    }
    if (decision.known_static_result_available) {
      const KnownStaticLidarHitResult& classification = decision.known_static_result;
      recordKnownStaticLidarHit(classification, stats.known_static_lidar);
      if (endpoint_cell.has_value() && stats.retained_known_static_hits.size() <
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

const CurrentLidarAcceptedHitProvenance*
findCurrentLidarAcceptedHitProvenance(const CurrentLidarOverlayStats& stats,
                                      const GridIndex cell) noexcept {
  const auto iter =
      std::find_if(stats.accepted_hits.begin(), stats.accepted_hits.end(),
                   [cell](const CurrentLidarAcceptedHitProvenance& provenance) {
                     return provenance.cell.x == cell.x && provenance.cell.y == cell.y;
                   });
  return iter == stats.accepted_hits.end() ? nullptr : &*iter;
}

std::string
formatCurrentLidarAcceptedHitDiagnostic(const CurrentLidarOverlayStats& stats,
                                        const GridIndex cell) {
  const CurrentLidarAcceptedHitProvenance* provenance =
      findCurrentLidarAcceptedHitProvenance(stats, cell);
  if (provenance == nullptr) {
    return "current_lidar_hit[unavailable]";
  }

  const LidarBeamObservation& observation = provenance->observation;
  const LidarBeamProjection& projection = observation.projection;
  const LidarIngestionDecisionSnapshot& decision = provenance->ingestion_decision;
  std::ostringstream stream;
  stream << "current_lidar_hit[cell=(" << provenance->cell.x << ", "
         << provenance->cell.y << ") beam=" << observation.beam_index
         << " action=" << lidarIngestionActionName(decision.action)
         << " reason=" << lidarIngestionReasonName(decision.reason)
         << " surface=" << lidarExpectedSurfaceKindName(decision.expected_surface)
         << " endpoint=(" << projection.endpoint_map_m.x << ", "
         << projection.endpoint_map_m.y << ", " << projection.endpoint_map_m.z
         << ") measured_range=" << observation.measured_range_m
         << " expected_range=" << decision.expected_range_m
         << " delta=" << decision.range_delta_m << " ray_origin=("
         << projection.ray_origin_map_m.x << ", " << projection.ray_origin_map_m.y
         << ", " << projection.ray_origin_map_m.z << ") ray_dir=("
         << projection.ray_direction_map.x << ", " << projection.ray_direction_map.y
         << ", " << projection.ray_direction_map.z << ") source_attitude=(valid="
         << (observation.source_attitude_valid ? "true" : "false")
         << " roll=" << observation.source_roll_rad
         << " pitch=" << observation.source_pitch_rad
         << " tilt=" << observation.source_tilt_rad << ") applied_attitude=(applied="
         << (projection.attitude_compensation_applied ? "true" : "false")
         << " roll=" << projection.applied_roll_rad
         << " pitch=" << projection.applied_pitch_rad
         << " tilt=" << projection.applied_tilt_rad << ")]";
  return stream.str();
}

} // namespace drone_city_nav

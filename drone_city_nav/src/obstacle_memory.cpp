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

[[nodiscard]] MemoryCellProvenance
makeCellProvenance(const GridIndex cell, const AcceptedObstacleMemoryHit& hit) {
  MemoryCellProvenance provenance{
      .cell = cell,
      .occupancy_trigger = hit,
      .last_hit = hit,
      .min_endpoint_z_m = std::nullopt,
      .max_endpoint_z_m = std::nullopt,
      .accepted_hit_count = 1U,
  };
  if (hit.beam.projection.endpoint_xyz_valid &&
      std::isfinite(hit.beam.projection.endpoint_map_m.z)) {
    provenance.min_endpoint_z_m = hit.beam.projection.endpoint_map_m.z;
    provenance.max_endpoint_z_m = hit.beam.projection.endpoint_map_m.z;
  }
  return provenance;
}

void updateCellProvenance(MemoryCellProvenance& provenance,
                          const AcceptedObstacleMemoryHit& hit) {
  provenance.last_hit = hit;
  if (provenance.accepted_hit_count < std::numeric_limits<std::uint64_t>::max()) {
    ++provenance.accepted_hit_count;
  }
  if (!hit.beam.projection.endpoint_xyz_valid ||
      !std::isfinite(hit.beam.projection.endpoint_map_m.z)) {
    return;
  }
  const double endpoint_z_m = hit.beam.projection.endpoint_map_m.z;
  provenance.min_endpoint_z_m =
      provenance.min_endpoint_z_m.has_value()
          ? std::min(*provenance.min_endpoint_z_m, endpoint_z_m)
          : endpoint_z_m;
  provenance.max_endpoint_z_m =
      provenance.max_endpoint_z_m.has_value()
          ? std::max(*provenance.max_endpoint_z_m, endpoint_z_m)
          : endpoint_z_m;
}

} // namespace

ObstacleMemoryGrid::ObstacleMemoryGrid(const GridBounds& bounds)
    : raw_grid_{bounds},
      scores_(raw_grid_.cellCount(), 0) {
}

ObstacleMemoryStats
ObstacleMemoryGrid::integrateScan(const Pose2& pose, const LaserScan2DView& scan,
                                  const ObstacleMemoryConfig& config,
                                  const KnownStaticLidarHitClassifier* classifier,
                                  const GroundLidarRejectionConfig* ground_config) {
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

  const LidarProjectionPose fallback_projection_pose{
      pose.position,  scan.origin_altitude_m, pose.yaw_rad,       scan.roll_rad,
      scan.pitch_rad, scan.altitude_valid,    scan.attitude_valid};
  const LidarProjectionConfig projection_config{
      config.max_lidar_range_m,      config.range_hit_epsilon_m,
      scan.scan_yaw_offset_rad,      scan.lidar_z_offset_m,
      scan.min_projected_altitude_m, scan.max_projected_altitude_m,
      scan.compensate_attitude,      scan.lidar_mount_roll_rad,
      scan.lidar_mount_pitch_rad,    scan.lidar_mount_yaw_rad};

  const auto stride = static_cast<std::size_t>(std::max(1, config.scan_stride));
  const bool aligned_poses_available =
      scan.beam_projection_poses.size() == scan.ranges.size();
  for (std::size_t i = 0U; i < scan.ranges.size(); i += stride) {
    const float raw_range = scan.ranges[i];
    if (!lidarRawRangeUsable(raw_range, scan.range_min_m)) {
      ++stats.invalid_ranges;
      continue;
    }

    const LidarProjectionPose& projection_pose = aligned_poses_available
                                                     ? scan.beam_projection_poses[i]
                                                     : fallback_projection_pose;
    if (aligned_poses_available) {
      ++stats.timestamp_aligned_beams;
    }
    const LidarBeamProjection projection = projectLidarBeam(
        projection_pose, projection_config, scan.range_min_m, scan_range_max,
        scan.angle_min_rad, scan.angle_increment_rad, i, raw_range);
    if (projection.status != LidarBeamProjectionStatus::kAccepted &&
        projection.status != LidarBeamProjectionStatus::kAltitudeRejected) {
      ++stats.invalid_ranges;
      continue;
    }
    const LidarBeamObservation observation = makeLidarBeamObservation(
        scan.timing, i, projection, scan_range_max, projection_pose, projection_config,
        aligned_poses_available);
    LidarIngestionDecision decision = resolveAmbiguousKnownStaticIngestion(
        observation, evaluateLidarIngestion(observation, classifier, ground_config),
        &ambiguous_hit_tracker_);
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
    if (decision.action == LidarIngestionAction::kSuppressAllUpdates) {
      if (decision.known_static_result_available) {
        recordKnownStaticLidarHit(decision.known_static_result,
                                  stats.known_static_lidar, false);
      }
      continue;
    }
    const auto clipped =
        clipSegmentToGrid(raw_grid_, projection_pose.position, projection.endpoint);
    if (!clipped.has_value()) {
      ++stats.invalid_ranges;
      continue;
    }

    const auto start_cell = raw_grid_.worldToCell(projection_pose.position);
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

    if (decision.action == LidarIngestionAction::kIntegrateFreeOnly ||
        !projection.hit) {
      if (decision.known_static_result_available) {
        recordKnownStaticLidarHit(decision.known_static_result,
                                  stats.known_static_lidar);
      }
      continue;
    }
    if (!endpoint_cell.has_value()) {
      ++stats.outside_hit_endpoints;
      continue;
    }
    const GridIndex endpoint_grid_cell =
        endpoint_cell.value(); // NOLINT(bugprone-unchecked-optional-access)

    const bool classifier_applied = decision.known_static_result_available;
    const KnownStaticLidarHitResult classification = decision.known_static_result;
    if (classifier_applied) {
      recordKnownStaticLidarHit(classification, stats.known_static_lidar);
      if (stats.retained_known_static_hits.size() <
          kMaxRetainedKnownStaticHitDiagnostics) {
        if (const std::optional<KnownStaticLidarHitProvenance> provenance =
                makeKnownStaticLidarHitProvenance(
                    classification,
                    Point3{projection.endpoint.x, projection.endpoint.y,
                           projection.endpoint_altitude_m},
                    endpoint_grid_cell.x, endpoint_grid_cell.y);
            provenance.has_value()) {
          stats.retained_known_static_hits.push_back(*provenance);
        }
      }
    }

    const AcceptedObstacleMemoryHit accepted_hit{
        .beam = observation,
        .known_static =
            makeKnownStaticClassificationSnapshot(classifier_applied, classification),
        .ingestion_decision = makeLidarIngestionDecisionSnapshot(decision),
    };
    const std::optional<ObstacleMemoryOccupiedTransition> transition =
        applyAcceptedHit(endpoint_grid_cell, accepted_hit, decision, config);
    ++stats.occupied_cells_updated;
    if (transition.has_value()) {
      ++stats.newly_occupied_cells;
      stats.occupied_transitions.push_back(*transition);
    }
  }

  return stats;
}

void ObstacleMemoryGrid::reset() {
  raw_grid_ = OccupancyGrid2D{raw_grid_.bounds()};
  std::fill(scores_.begin(), scores_.end(), 0);
  active_provenance_.clear();
  ambiguous_hit_tracker_.clear();
}

void ObstacleMemoryGrid::configureAmbiguousHitTracking(
    const AmbiguousLidarHitTrackerConfig& config) {
  ambiguous_hit_tracker_.configure(config);
}

const OccupancyGrid2D& ObstacleMemoryGrid::rawGrid() const noexcept {
  return raw_grid_;
}

const std::unordered_map<std::size_t, MemoryCellProvenance>&
ObstacleMemoryGrid::activeProvenance() const noexcept {
  return active_provenance_;
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

std::optional<ObstacleMemoryOccupiedTransition> ObstacleMemoryGrid::applyAcceptedHit(
    const GridIndex cell, const AcceptedObstacleMemoryHit& hit,
    const LidarIngestionDecision& decision, const ObstacleMemoryConfig& config) {
  if (!raw_grid_.contains(cell)) {
    return std::nullopt;
  }
  const std::size_t index = raw_grid_.linearIndex(cell);
  const int score_before = scores_[index];
  const bool occupied_before = raw_grid_.isOccupied(cell);
  scores_[index] = std::clamp(scores_[index] + config.hit_weight, config.min_score,
                              config.max_score);
  syncCellState(cell, config);
  if (!raw_grid_.isOccupied(cell)) {
    return std::nullopt;
  }

  auto provenance = active_provenance_.find(index);
  if (!occupied_before || provenance == active_provenance_.end()) {
    provenance =
        active_provenance_.insert_or_assign(index, makeCellProvenance(cell, hit)).first;
  } else {
    updateCellProvenance(provenance->second, hit);
  }
  if (occupied_before) {
    return std::nullopt;
  }
  return ObstacleMemoryOccupiedTransition{
      .score_before = score_before,
      .score_after = scores_[index],
      .provenance = provenance->second,
      .trigger_decision = decision,
  };
}

void ObstacleMemoryGrid::syncCellState(const GridIndex cell,
                                       const ObstacleMemoryConfig& config) {
  const int score = scores_.at(raw_grid_.linearIndex(cell));
  if (score >= config.occupied_score) {
    raw_grid_.setOccupied(cell);
    return;
  }
  active_provenance_.erase(raw_grid_.linearIndex(cell));
  if (score <= config.free_score) {
    raw_grid_.setFree(cell);
    return;
  }
  raw_grid_.setUnknown(cell);
}

} // namespace drone_city_nav

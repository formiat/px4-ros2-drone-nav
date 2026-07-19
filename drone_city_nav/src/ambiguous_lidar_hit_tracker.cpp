#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance(const Point3& lhs, const Point3& rhs) noexcept {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

[[nodiscard]] double directionAngle(const Point3& lhs, const Point3& rhs) noexcept {
  const double dot = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  return std::acos(std::clamp(dot, -1.0, 1.0));
}

} // namespace

UncertainLidarHitTracker::UncertainLidarHitTracker(
    const UncertainLidarHitTrackerConfig& config)
    : config_{config} {
  configure(config);
}

void UncertainLidarHitTracker::configure(const UncertainLidarHitTrackerConfig& config) {
  config_ = config;
  config_.required_independent_scans =
      std::max<std::size_t>(1U, config_.required_independent_scans);
  config_.max_scan_gap_ns = std::max<std::int64_t>(1, config_.max_scan_gap_ns);
  config_.retention_ns = std::max(config_.max_scan_gap_ns, config_.retention_ns);
  if (!std::isfinite(config_.endpoint_voxel_size_m) ||
      config_.endpoint_voxel_size_m <= 0.0) {
    config_.endpoint_voxel_size_m = 0.5;
  }
  if (!std::isfinite(config_.min_viewpoint_translation_m) ||
      config_.min_viewpoint_translation_m < 0.0) {
    config_.min_viewpoint_translation_m = 0.5;
  }
  if (!std::isfinite(config_.min_viewpoint_direction_change_rad) ||
      config_.min_viewpoint_direction_change_rad < 0.0) {
    config_.min_viewpoint_direction_change_rad = 0.0523598776;
  }
  clear();
}

UncertainLidarHitConfirmation
UncertainLidarHitTracker::observe(const UncertainLidarHitObservation& observation) {
  if (observation.scan_stamp_ns <= 0 ||
      observation.kind == UncertainLidarHitKind::kNone ||
      observation.association_id.empty() || observation.part_id.empty()) {
    return {};
  }
  const std::size_t expired_candidates = expire(observation.scan_stamp_ns);
  const Key base_key = baseKeyFor(observation);
  Evidence& evidence = evidence_[matchingKeyFor(observation, base_key)];
  double viewpoint_translation_m = 0.0;
  double viewpoint_direction_change_rad = 0.0;
  bool new_scan_vote = false;
  const auto add_observation = [&observation, &evidence] {
    const bool first_observation = evidence.independent_scans == 0U;
    ++evidence.independent_scans;
    if (observation.evidence == UncertainLidarHitEvidence::kExpectedSurfaceAttached) {
      ++evidence.expected_surface_observations;
      ++evidence.consecutive_expected_surface_observations;
      evidence.consecutive_detached_obstacle_observations = 0U;
    } else {
      ++evidence.detached_obstacle_observations;
      ++evidence.consecutive_detached_obstacle_observations;
      evidence.consecutive_expected_surface_observations = 0U;
    }
    evidence.last_scan_stamp_ns = observation.scan_stamp_ns;
    evidence.last_endpoint_map_m = observation.endpoint_map_m;
    evidence.last_ray_origin_map_m = observation.ray_origin_map_m;
    evidence.last_ray_direction_map = observation.ray_direction_map;
    evidence.min_endpoint_surface_distance_m =
        first_observation ? observation.endpoint_surface_distance_m
                          : std::min(evidence.min_endpoint_surface_distance_m,
                                     observation.endpoint_surface_distance_m);
    evidence.max_endpoint_surface_distance_m =
        first_observation ? observation.endpoint_surface_distance_m
                          : std::max(evidence.max_endpoint_surface_distance_m,
                                     observation.endpoint_surface_distance_m);
    evidence.last_distance_before_surface_m = observation.distance_before_surface_m;
    evidence.last_range_residual_m = observation.range_residual_m;
  };
  if (evidence.last_scan_stamp_ns == 0 ||
      observation.scan_stamp_ns - evidence.last_scan_stamp_ns >
          config_.max_scan_gap_ns) {
    evidence = Evidence{.first_scan_stamp_ns = observation.scan_stamp_ns};
    add_observation();
    new_scan_vote = true;
  } else if (observation.scan_stamp_ns > evidence.last_scan_stamp_ns) {
    viewpoint_translation_m =
        distance(observation.ray_origin_map_m, evidence.last_ray_origin_map_m);
    viewpoint_direction_change_rad =
        directionAngle(observation.ray_direction_map, evidence.last_ray_direction_map);
    if (viewpoint_translation_m >= config_.min_viewpoint_translation_m ||
        viewpoint_direction_change_rad >= config_.min_viewpoint_direction_change_rad) {
      add_observation();
      new_scan_vote = true;
    }
  }

  UncertainLidarHitResolution resolution = UncertainLidarHitResolution::kPending;
  if (evidence.consecutive_expected_surface_observations >=
      config_.required_independent_scans) {
    resolution = UncertainLidarHitResolution::kConfirmedExpectedSurface;
  } else if (evidence.consecutive_detached_obstacle_observations >=
             config_.required_independent_scans) {
    resolution = UncertainLidarHitResolution::kConfirmedObstacle;
  }
  return UncertainLidarHitConfirmation{
      .independent_scans = evidence.independent_scans,
      .expected_surface_observations = evidence.expected_surface_observations,
      .detached_obstacle_observations = evidence.detached_obstacle_observations,
      .expired_candidates = expired_candidates,
      .viewpoint_translation_m = viewpoint_translation_m,
      .viewpoint_direction_change_rad = viewpoint_direction_change_rad,
      .resolution = resolution,
      .new_scan_vote = new_scan_vote,
  };
}

std::size_t UncertainLidarHitTracker::expire(const std::int64_t scan_stamp_ns) {
  if (scan_stamp_ns <= 0) {
    return 0U;
  }
  const std::size_t before = evidence_.size();
  std::erase_if(evidence_, [this, scan_stamp_ns](const auto& item) {
    return scan_stamp_ns - item.second.last_scan_stamp_ns > config_.retention_ns;
  });
  return before - evidence_.size();
}

void UncertainLidarHitTracker::clear() noexcept {
  evidence_.clear();
}

std::size_t UncertainLidarHitTracker::candidateCount() const noexcept {
  return evidence_.size();
}

std::size_t
UncertainLidarHitTracker::KeyHash::operator()(const Key& key) const noexcept {
  std::size_t hash = std::hash<unsigned>{}(static_cast<unsigned>(key.kind));
  const auto combine = [&hash](const std::size_t value) {
    hash ^= value + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
  };
  combine(std::hash<std::string>{}(key.association_id));
  combine(std::hash<std::string>{}(key.part_id));
  combine(std::hash<int>{}(key.voxel_x));
  combine(std::hash<int>{}(key.voxel_y));
  combine(std::hash<int>{}(key.voxel_z));
  return hash;
}

UncertainLidarHitTracker::Key UncertainLidarHitTracker::baseKeyFor(
    const UncertainLidarHitObservation& observation) const {
  const auto voxel = [this](const double coordinate) {
    return static_cast<int>(std::floor(coordinate / config_.endpoint_voxel_size_m));
  };
  return Key{observation.kind,
             std::string{observation.association_id},
             std::string{observation.part_id},
             voxel(observation.endpoint_map_m.x),
             voxel(observation.endpoint_map_m.y),
             voxel(observation.endpoint_map_m.z)};
}

UncertainLidarHitTracker::Key UncertainLidarHitTracker::matchingKeyFor(
    const UncertainLidarHitObservation& observation, const Key& base_key) const {
  double best_distance_m = std::numeric_limits<double>::infinity();
  const Key* best_key = nullptr;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        Key candidate = base_key;
        candidate.voxel_x += dx;
        candidate.voxel_y += dy;
        candidate.voxel_z += dz;
        const auto found = evidence_.find(candidate);
        if (found == evidence_.end()) {
          continue;
        }
        const double endpoint_distance_m =
            distance(observation.endpoint_map_m, found->second.last_endpoint_map_m);
        if (endpoint_distance_m <= config_.endpoint_voxel_size_m &&
            endpoint_distance_m < best_distance_m) {
          best_distance_m = endpoint_distance_m;
          best_key = &found->first;
        }
      }
    }
  }
  return best_key != nullptr ? *best_key : base_key;
}

const char* uncertainLidarHitKindName(const UncertainLidarHitKind kind) noexcept {
  switch (kind) {
    case UncertainLidarHitKind::kNone:
      return "none";
    case UncertainLidarHitKind::kKnownStaticBoundary:
      return "known_static_boundary";
    case UncertainLidarHitKind::kGroundCandidate:
      return "ground_candidate";
    case UncertainLidarHitKind::kProjectionUncertainUnknown:
      return "projection_uncertain_unknown";
  }
  return "unknown";
}

const char*
uncertainLidarHitEvidenceName(const UncertainLidarHitEvidence evidence) noexcept {
  switch (evidence) {
    case UncertainLidarHitEvidence::kExpectedSurfaceAttached:
      return "expected_surface_attached";
    case UncertainLidarHitEvidence::kDetachedObstacle:
      return "detached_obstacle";
  }
  return "unknown";
}

const char*
uncertainLidarHitResolutionName(const UncertainLidarHitResolution resolution) noexcept {
  switch (resolution) {
    case UncertainLidarHitResolution::kPending:
      return "pending";
    case UncertainLidarHitResolution::kConfirmedExpectedSurface:
      return "confirmed_expected_surface";
    case UncertainLidarHitResolution::kConfirmedObstacle:
      return "confirmed_obstacle";
  }
  return "unknown";
}

} // namespace drone_city_nav

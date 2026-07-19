#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance(const Point3& lhs, const Point3& rhs) noexcept {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

[[nodiscard]] double directionAngle(const Point3& lhs, const Point3& rhs) noexcept {
  const double dot = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  return std::acos(std::clamp(dot, -1.0, 1.0));
}

[[nodiscard]] bool staticAttached(const KnownStaticEndpointRelation relation) noexcept {
  return relation == KnownStaticEndpointRelation::kInsideSolid ||
         relation == KnownStaticEndpointRelation::kNearSurface ||
         relation == KnownStaticEndpointRelation::kInsideOpeningBoundary;
}

} // namespace

AmbiguousLidarHitTracker::AmbiguousLidarHitTracker(
    const AmbiguousLidarHitTrackerConfig& config)
    : config_{config} {
  configure(config);
}

void AmbiguousLidarHitTracker::configure(const AmbiguousLidarHitTrackerConfig& config) {
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

AmbiguousLidarHitConfirmation
AmbiguousLidarHitTracker::observe(const AmbiguousStaticHitObservation& observation) {
  if (observation.scan_stamp_ns <= 0 || observation.structure_id.empty() ||
      observation.part_id.empty()) {
    return {};
  }
  const std::size_t expired_candidates = prune(observation.scan_stamp_ns);
  Evidence& evidence = evidence_[keyFor(observation)];
  double viewpoint_translation_m = 0.0;
  double viewpoint_direction_change_rad = 0.0;
  bool new_scan_vote = false;
  const auto add_observation = [&observation, &evidence] {
    const bool first_observation = evidence.independent_scans == 0U;
    ++evidence.independent_scans;
    if (staticAttached(observation.endpoint_relation)) {
      ++evidence.static_attached_observations;
      ++evidence.consecutive_static_attached_observations;
      evidence.consecutive_detached_obstacle_observations = 0U;
    } else {
      ++evidence.detached_obstacle_observations;
      ++evidence.consecutive_detached_obstacle_observations;
      evidence.consecutive_static_attached_observations = 0U;
    }
    evidence.opening_boundary_observed =
        evidence.opening_boundary_observed ||
        observation.endpoint_relation ==
            KnownStaticEndpointRelation::kInsideOpeningBoundary;
    evidence.last_scan_stamp_ns = observation.scan_stamp_ns;
    evidence.last_endpoint_map_m = observation.endpoint_map_m;
    evidence.last_ray_origin_map_m = observation.ray_origin_map_m;
    evidence.last_ray_direction_map = observation.ray_direction_map;
    evidence.min_endpoint_solid_distance_m =
        first_observation ? observation.endpoint_solid_distance_m
                          : std::min(evidence.min_endpoint_solid_distance_m,
                                     observation.endpoint_solid_distance_m);
    evidence.max_endpoint_solid_distance_m =
        first_observation ? observation.endpoint_solid_distance_m
                          : std::max(evidence.max_endpoint_solid_distance_m,
                                     observation.endpoint_solid_distance_m);
    evidence.last_distance_before_solid_m = observation.distance_before_solid_m;
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

  AmbiguousLidarHitResolution resolution = AmbiguousLidarHitResolution::kPending;
  if (evidence.consecutive_static_attached_observations >=
      config_.required_independent_scans) {
    resolution = AmbiguousLidarHitResolution::kConfirmedStaticAttached;
  } else if (evidence.consecutive_detached_obstacle_observations >=
             config_.required_independent_scans) {
    resolution = AmbiguousLidarHitResolution::kConfirmedDetachedObstacle;
  }
  return AmbiguousLidarHitConfirmation{
      .independent_scans = evidence.independent_scans,
      .static_attached_observations = evidence.static_attached_observations,
      .detached_obstacle_observations = evidence.detached_obstacle_observations,
      .expired_candidates = expired_candidates,
      .viewpoint_translation_m = viewpoint_translation_m,
      .viewpoint_direction_change_rad = viewpoint_direction_change_rad,
      .resolution = resolution,
      .new_scan_vote = new_scan_vote,
      .opening_boundary_observed = evidence.opening_boundary_observed,
  };
}

void AmbiguousLidarHitTracker::clear() noexcept {
  evidence_.clear();
}

std::size_t AmbiguousLidarHitTracker::candidateCount() const noexcept {
  return evidence_.size();
}

std::size_t
AmbiguousLidarHitTracker::KeyHash::operator()(const Key& key) const noexcept {
  std::size_t hash = std::hash<std::string>{}(key.structure_id);
  const auto combine = [&hash](const std::size_t value) {
    hash ^= value + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
  };
  combine(std::hash<std::string>{}(key.part_id));
  combine(std::hash<int>{}(key.voxel_x));
  combine(std::hash<int>{}(key.voxel_y));
  combine(std::hash<int>{}(key.voxel_z));
  return hash;
}

AmbiguousLidarHitTracker::Key AmbiguousLidarHitTracker::keyFor(
    const AmbiguousStaticHitObservation& observation) const {
  const auto voxel = [this](const double coordinate) {
    return static_cast<int>(std::floor(coordinate / config_.endpoint_voxel_size_m));
  };
  return Key{std::string{observation.structure_id}, std::string{observation.part_id},
             voxel(observation.endpoint_map_m.x), voxel(observation.endpoint_map_m.y),
             voxel(observation.endpoint_map_m.z)};
}

std::size_t AmbiguousLidarHitTracker::prune(const std::int64_t scan_stamp_ns) {
  const std::size_t before = evidence_.size();
  std::erase_if(evidence_, [this, scan_stamp_ns](const auto& item) {
    return scan_stamp_ns - item.second.last_scan_stamp_ns > config_.retention_ns;
  });
  return before - evidence_.size();
}

const char*
ambiguousLidarHitResolutionName(const AmbiguousLidarHitResolution resolution) noexcept {
  switch (resolution) {
    case AmbiguousLidarHitResolution::kPending:
      return "pending";
    case AmbiguousLidarHitResolution::kConfirmedStaticAttached:
      return "confirmed_static_attached";
    case AmbiguousLidarHitResolution::kConfirmedDetachedObstacle:
      return "confirmed_detached_obstacle";
  }
  return "unknown";
}

} // namespace drone_city_nav

#include "drone_city_nav/observation_frontier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis{14695981039346656037ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

struct GridIndex3DHash {
  [[nodiscard]] std::size_t operator()(const GridIndex3D index) const noexcept {
    std::size_t seed = std::hash<int>{}(index.x);
    const auto combine = [&seed](const int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(index.y);
    combine(index.z);
    return seed;
  }
};

struct SupportedObservationRay {
  Vec3 direction{};
  Point3 boundary_point{};
  std::vector<GridIndex3D> unknown_cells;
  double known_free_ray_m{0.0};
  double traversable_ray_m{0.0};
};

struct DirectionalObservationCluster {
  Vec3 direction{};
  Point3 boundary_centroid{};
  std::unordered_set<GridIndex3D, GridIndex3DHash> information_gain_cells;
  std::vector<std::size_t> ray_indices;
  std::size_t supporting_rays{0U};
  double minimum_known_free_ray_m{0.0};
};

struct ResolvedObservationPose {
  Point3 pose{};
  double advance_m{0.0};
  double direction_alignment{-1.0};
};

[[nodiscard]] auto clusterRank(const DirectionalObservationCluster& cluster,
                               const ObservedSpaceValidationPolicy validation_policy) {
  return std::tuple{cluster.information_gain_cells.size(), cluster.supporting_rays,
                    validation_policy ==
                            ObservedSpaceValidationPolicy::kRequireKnownFree
                        ? cluster.minimum_known_free_ray_m
                        : 0.0};
}

[[nodiscard]] Vec3 normalized(const Vec3& direction) noexcept {
  const double norm = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                direction.z * direction.z);
  if (!(norm > 1.0e-9) || !std::isfinite(norm)) {
    return {};
  }
  return Vec3{direction.x / norm, direction.y / norm, direction.z / norm};
}

[[nodiscard]] const std::array<Vec3, 26>& candidateObservationDirections() {
  static const std::array<Vec3, 26> directions = [] {
    std::array<Vec3, 26> result{};
    std::size_t index = 0U;
    for (int z = -1; z <= 1; ++z) {
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          if (x == 0 && y == 0 && z == 0) {
            continue;
          }
          result.at(index++) = normalized(Vec3{
              static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
        }
      }
    }
    return result;
  }();
  return directions;
}

[[nodiscard]] double snapLinearAngle(const double angle, const double minimum,
                                     const double maximum,
                                     const std::size_t samples) noexcept {
  if (samples <= 1U) {
    return minimum;
  }
  const double step = (maximum - minimum) / static_cast<double>(samples - 1U);
  const double index = std::clamp(std::round((angle - minimum) / step), 0.0,
                                  static_cast<double>(samples - 1U));
  return minimum + index * step;
}

[[nodiscard]] std::vector<Vec3>
sensorAlignedObservationDirections(const SensorObservabilityConfig& config) {
  std::vector<Vec3> result;
  result.reserve(candidateObservationDirections().size());
  for (const Vec3& candidate : candidateObservationDirections()) {
    const double horizontal_norm = std::hypot(candidate.x, candidate.y);
    if (!(horizontal_norm > 1.0e-9)) {
      continue;
    }
    const double elevation = std::atan2(candidate.z, horizontal_norm);
    if (elevation < config.vertical_min_angle_rad ||
        elevation > config.vertical_max_angle_rad) {
      continue;
    }
    const double azimuth = std::atan2(candidate.y, candidate.x);
    const double sensor_azimuth =
        snapLinearAngle(azimuth, config.horizontal_min_angle_rad,
                        config.horizontal_max_angle_rad, config.horizontal_samples);
    const double sensor_elevation =
        snapLinearAngle(elevation, config.vertical_min_angle_rad,
                        config.vertical_max_angle_rad, config.vertical_samples);
    const double horizontal_scale = std::cos(sensor_elevation);
    const Vec3 direction{horizontal_scale * std::cos(sensor_azimuth),
                         horizontal_scale * std::sin(sensor_azimuth),
                         std::sin(sensor_elevation)};
    const bool duplicate = std::ranges::any_of(result, [&](const Vec3& current) {
      return std::abs(current.x - direction.x) <= 1.0e-9 &&
             std::abs(current.y - direction.y) <= 1.0e-9 &&
             std::abs(current.z - direction.z) <= 1.0e-9;
    });
    if (!duplicate) {
      result.push_back(direction);
    }
  }
  return result;
}

[[nodiscard]] std::vector<DirectionalObservationCluster>
makeDirectionalClusters(const std::span<const SupportedObservationRay> rays,
                        const double half_angle_rad,
                        const ObservedSpaceValidationPolicy validation_policy) {
  std::vector<DirectionalObservationCluster> result;
  result.reserve(rays.size());
  const double minimum_dot = std::cos(half_angle_rad);
  std::vector<bool> available(rays.size(), true);
  while (std::ranges::any_of(available, std::identity{})) {
    DirectionalObservationCluster best;
    for (std::size_t seed_index = 0U; seed_index < rays.size(); ++seed_index) {
      if (!available[seed_index]) {
        continue;
      }
      const SupportedObservationRay& seed = rays[seed_index];
      DirectionalObservationCluster candidate;
      Vec3 weighted_direction{};
      Point3 boundary_sum{};
      double minimum_support_m = std::numeric_limits<double>::infinity();
      for (std::size_t ray_index = 0U; ray_index < rays.size(); ++ray_index) {
        if (!available[ray_index]) {
          continue;
        }
        const SupportedObservationRay& ray = rays[ray_index];
        const double alignment = seed.direction.x * ray.direction.x +
                                 seed.direction.y * ray.direction.y +
                                 seed.direction.z * ray.direction.z;
        if (alignment + 1.0e-12 < minimum_dot) {
          continue;
        }
        candidate.ray_indices.push_back(ray_index);
        ++candidate.supporting_rays;
        boundary_sum.x += ray.boundary_point.x;
        boundary_sum.y += ray.boundary_point.y;
        boundary_sum.z += ray.boundary_point.z;
        minimum_support_m = std::min(minimum_support_m, ray.known_free_ray_m);
        for (const GridIndex3D cell : ray.unknown_cells) {
          static_cast<void>(candidate.information_gain_cells.insert(cell));
        }
        const double ray_weight = static_cast<double>(ray.unknown_cells.size());
        weighted_direction.x += ray.direction.x * ray_weight;
        weighted_direction.y += ray.direction.y * ray_weight;
        weighted_direction.z += ray.direction.z * ray_weight;
      }
      if (candidate.supporting_rays == 0U) {
        continue;
      }
      const double inverse_count = 1.0 / static_cast<double>(candidate.supporting_rays);
      candidate.boundary_centroid =
          Point3{boundary_sum.x * inverse_count, boundary_sum.y * inverse_count,
                 boundary_sum.z * inverse_count};
      candidate.direction = normalized(weighted_direction);
      candidate.minimum_known_free_ray_m =
          std::isfinite(minimum_support_m) ? minimum_support_m : 0.0;
      if (clusterRank(candidate, validation_policy) >
          clusterRank(best, validation_policy)) {
        best = std::move(candidate);
      }
    }
    if (best.ray_indices.empty()) {
      break;
    }
    for (const std::size_t ray_index : best.ray_indices) {
      available[ray_index] = false;
    }
    result.push_back(std::move(best));
  }
  return result;
}

[[nodiscard]] std::optional<ResolvedObservationPose>
resolveObservationPose(const ObservedOccupancyGrid3D& occupancy, const Point3& origin,
                       const std::span<const SupportedObservationRay> rays,
                       const DirectionalObservationCluster& cluster,
                       const SensorObservabilityConfig& config,
                       const ObservedSpaceValidationPolicy validation_policy) noexcept {
  const double resolution_m = occupancy.bounds().resolution_m;
  if (!(resolution_m > 0.0)) {
    return std::nullopt;
  }
  const double pose_step_m =
      std::min(resolution_m, std::max(1.0e-3, config.footprint.sweep_step_m));
  std::optional<ResolvedObservationPose> best;
  for (const std::size_t ray_index : cluster.ray_indices) {
    if (ray_index >= rays.size()) {
      continue;
    }
    const SupportedObservationRay& ray = rays[ray_index];
    // A permissive frontier crosses the observed boundary without turning the
    // entire sensor ray into one long-lived exploration objective.
    const double maximum_advance_m =
        validation_policy == ObservedSpaceValidationPolicy::kAllowUnknown
            ? std::min(ray.traversable_ray_m,
                       ray.known_free_ray_m + config.minimum_observation_pose_advance_m)
            : ray.known_free_ray_m;
    double advance_m = std::floor(maximum_advance_m / pose_step_m) * pose_step_m;
    for (; advance_m + 1.0e-9 >= config.minimum_observation_pose_advance_m;
         advance_m -= pose_step_m) {
      const Point3 candidate{origin.x + ray.direction.x * advance_m,
                             origin.y + ray.direction.y * advance_m,
                             origin.z + ray.direction.z * advance_m};
      const SweptFootprintResult validation = validateObservedSweptFootprint(
          occupancy, origin, FootprintBodyAxis{}, candidate, FootprintBodyAxis{},
          config.footprint, validation_policy);
      if (!validation.accepted()) {
        continue;
      }
      const double direction_alignment = ray.direction.x * cluster.direction.x +
                                         ray.direction.y * cluster.direction.y +
                                         ray.direction.z * cluster.direction.z;
      if (!best.has_value() || advance_m > best->advance_m + 1.0e-9 ||
          (std::abs(advance_m - best->advance_m) <= 1.0e-9 &&
           (direction_alignment > best->direction_alignment + 1.0e-9 ||
            (std::abs(direction_alignment - best->direction_alignment) <= 1.0e-9 &&
             std::tie(candidate.x, candidate.y, candidate.z) <
                 std::tie(best->pose.x, best->pose.y, best->pose.z))))) {
        best = ResolvedObservationPose{.pose = candidate,
                                       .advance_m = advance_m,
                                       .direction_alignment = direction_alignment};
      }
      break;
    }
  }
  return best;
}

void hashInteger(std::uint64_t& hash, const std::int64_t value) noexcept {
  const std::uint64_t bits = static_cast<std::uint64_t>(value);
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (bits >> shift) & 0xFFU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t revisionRotatedRank(const std::uint64_t stable_value,
                                                const std::uint64_t revision) noexcept {
  std::uint64_t value = stable_value ^ (revision + 0x9e3779b97f4a7c15ULL +
                                        (stable_value << 6U) + (stable_value >> 2U));
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

[[nodiscard]] ObservationFrontierId makeCellId(const ObservedOccupancyGrid3D& occupancy,
                                               const Point3& pose) noexcept {
  const std::optional<GridIndex3D> cell = occupancy.worldToCell(pose);
  if (!cell.has_value()) {
    return {};
  }
  std::uint64_t hash = kFnvOffsetBasis;
  hashInteger(hash, cell->x);
  hashInteger(hash, cell->y);
  hashInteger(hash, cell->z);
  return ObservationFrontierId{hash};
}

[[nodiscard]] ObservationFrontierId
makeFrontierId(const Point3& pose, const double identity_resolution_m) noexcept {
  if (!std::isfinite(identity_resolution_m) || !(identity_resolution_m > 0.0)) {
    return {};
  }
  const std::array quantized{
      std::floor(pose.x / identity_resolution_m),
      std::floor(pose.y / identity_resolution_m),
      std::floor(pose.z / identity_resolution_m),
  };
  if (std::ranges::any_of(quantized, [](const double coordinate) {
        return !std::isfinite(coordinate) ||
               coordinate <
                   static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
               coordinate >
                   static_cast<double>(std::numeric_limits<std::int64_t>::max());
      })) {
    return {};
  }
  std::uint64_t hash = kFnvOffsetBasis;
  for (const double coordinate : quantized) {
    hashInteger(hash, static_cast<std::int64_t>(coordinate));
  }
  return ObservationFrontierId{hash};
}

[[nodiscard]] bool
betterFrontierRepresentative(const ObservationFrontier& candidate,
                             const ObservationFrontier& current,
                             const ObservedSpaceValidationPolicy validation_policy) {
  const auto rank = [validation_policy](const ObservationFrontier& frontier) {
    return std::tuple{frontier.information_gain_voxels, frontier.supporting_rays,
                      validation_policy ==
                              ObservedSpaceValidationPolicy::kRequireKnownFree
                          ? frontier.minimum_known_free_ray_m
                          : 0.0};
  };
  if (rank(candidate) != rank(current)) {
    return rank(candidate) > rank(current);
  }
  return std::tuple{
             candidate.observation_pose.x,     candidate.observation_pose.y,
             candidate.observation_pose.z,     candidate.supporting_viewpoint.x,
             candidate.supporting_viewpoint.y, candidate.supporting_viewpoint.z} <
         std::tuple{current.observation_pose.x,     current.observation_pose.y,
                    current.observation_pose.z,     current.supporting_viewpoint.x,
                    current.supporting_viewpoint.y, current.supporting_viewpoint.z};
}

[[nodiscard]] std::size_t
minimumFootprintInformationGainVoxels(const SweptFootprintConfig& footprint,
                                      const double resolution_m) noexcept {
  if (!(resolution_m > 0.0) || !std::isfinite(resolution_m)) {
    return 0U;
  }
  const double radius_m = std::max(0.0, footprint.radius_m);
  const double axial_extent_m =
      std::max(0.0, footprint.lower_extent_m) + std::max(0.0, footprint.upper_extent_m);
  const double circular_projection_m2 = std::numbers::pi * radius_m * radius_m;
  const double lateral_projection_m2 = 2.0 * radius_m * axial_extent_m;
  const double required_projection_m2 =
      std::max(circular_projection_m2, lateral_projection_m2);
  if (!(required_projection_m2 > 0.0) || !std::isfinite(required_projection_m2)) {
    return 1U;
  }
  const double required_voxels =
      std::ceil(required_projection_m2 / (resolution_m * resolution_m));
  if (!std::isfinite(required_voxels) ||
      required_voxels >= static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    return std::numeric_limits<std::size_t>::max();
  }
  return std::max<std::size_t>(1U, static_cast<std::size_t>(required_voxels));
}

[[nodiscard]] ObservationFrontierStatus
footprintStatus(const SweptFootprintStatus status) noexcept {
  switch (status) {
    case SweptFootprintStatus::kValid:
      return ObservationFrontierStatus::kAccepted;
    case SweptFootprintStatus::kOutsideGrid:
      return ObservationFrontierStatus::kOutsideMap;
    case SweptFootprintStatus::kUnknownSpace:
    case SweptFootprintStatus::kInvalidEsdf:
      return ObservationFrontierStatus::kFootprintNotObserved;
    case SweptFootprintStatus::kRawCollision:
      return ObservationFrontierStatus::kRawCollision;
  }
  return ObservationFrontierStatus::kFootprintNotObserved;
}

[[nodiscard]] bool bit(const OccupancyGrid3D::Chunk& words,
                       const std::size_t index) noexcept {
  return (words.at(index / 64U) & (std::uint64_t{1U} << (index % 64U))) != 0U;
}

[[nodiscard]] bool isSampledFreeCell(const ObservedOccupancyChunk3D& chunk,
                                     const std::size_t bit_index) noexcept {
  return bit(chunk.observed, bit_index) && !bit(chunk.occupied, bit_index);
}

[[nodiscard]] bool hasSupportedUnknownBoundaryImpl(
    const ObservedOccupancyGrid3D& occupancy, const GridIndex3D origin,
    const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy) noexcept {
  const double resolution_m = occupancy.bounds().resolution_m;
  if (!(resolution_m > 0.0)) {
    return false;
  }
  const int known_free_steps =
      validation_policy == ObservedSpaceValidationPolicy::kRequireKnownFree
          ? std::max(1, static_cast<int>(
                            std::ceil(config.minimum_known_free_ray_m / resolution_m)))
          : 0;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        bool supported = true;
        for (int step = 1; step <= known_free_steps; ++step) {
          const GridIndex3D sample{origin.x + dx * step, origin.y + dy * step,
                                   origin.z + dz * step};
          if (!occupancy.isKnownFree(sample)) {
            supported = false;
            break;
          }
        }
        if (!supported) {
          continue;
        }
        const GridIndex3D boundary{origin.x + dx * (known_free_steps + 1),
                                   origin.y + dy * (known_free_steps + 1),
                                   origin.z + dz * (known_free_steps + 1)};
        if (occupancy.contains(boundary) &&
            occupancy.state(boundary) == ObservedVoxelState::kUnknown) {
          return true;
        }
      }
    }
  }
  return false;
}

struct BoundaryCandidate {
  GridIndex3D cell{};
  ObservationFrontierId stable_id{};
  double goal_distance_m{0.0};
  std::uint64_t sampling_rank{0U};
};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] std::optional<BoundaryCandidate>
makeViewpointCandidate(const ObservedOccupancyGrid3D& occupancy, const GridIndex3D cell,
                       const std::uint64_t map_revision,
                       const SensorObservabilityConfig& config,
                       const ObservedSpaceValidationPolicy validation_policy,
                       const std::optional<Point3> mission_goal) {
  if (!occupancy.contains(cell) || !occupancy.isKnownFree(cell)) {
    return std::nullopt;
  }
  const Point3 center = occupancy.cellCenter(cell);
  // A sampled cell is only a possible sensor viewpoint. Validate the complete
  // physical footprint before spending the bounded ray budget on it.
  if (!validateObservedFootprintAt(occupancy, center, FootprintBodyAxis{},
                                   config.footprint, validation_policy)
           .accepted()) {
    return std::nullopt;
  }
  const ObservationFrontierId stable_id = makeCellId(occupancy, center);
  return BoundaryCandidate{
      .cell = cell,
      .stable_id = stable_id,
      .goal_distance_m =
          mission_goal.has_value() ? distance3D(center, *mission_goal) : 0.0,
      .sampling_rank = revisionRotatedRank(stable_id.value, map_revision),
  };
}

[[nodiscard]] std::optional<BoundaryCandidate>
makeBoundaryCandidate(const ObservedOccupancyGrid3D& occupancy, const GridIndex3D cell,
                      const std::uint64_t map_revision,
                      const SensorObservabilityConfig& config,
                      const ObservedSpaceValidationPolicy validation_policy,
                      const std::optional<Point3> mission_goal) {
  if (!observationPoseHasSupportedUnknownBoundary(occupancy, cell, config,
                                                  validation_policy)) {
    return std::nullopt;
  }
  return makeViewpointCandidate(occupancy, cell, map_revision, config,
                                validation_policy, mission_goal);
}

[[nodiscard]] ObservationFrontierDiscovery evaluateBoundaryCandidates(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t map_revision,
    const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy,
    std::vector<BoundaryCandidate> boundary_candidates,
    const std::size_t sampled_free_voxels, const std::size_t maximum_evaluations) {
  ObservationFrontierDiscovery result;
  result.sampled_free_voxels = sampled_free_voxels;
  result.boundary_candidates = boundary_candidates.size();
  result.evaluation_budget_exhausted = boundary_candidates.size() > maximum_evaluations;
  // Goal distance is only a budget-ordering term. Every reachable candidate
  // remains eligible as map revisions rotate the deterministic tie-breaker.
  std::ranges::sort(boundary_candidates, [](const BoundaryCandidate& lhs,
                                            const BoundaryCandidate& rhs) {
    return std::tuple{lhs.goal_distance_m, lhs.sampling_rank, lhs.stable_id.value,
                      lhs.cell.z,          lhs.cell.y,        lhs.cell.x} <
           std::tuple{rhs.goal_distance_m, rhs.sampling_rank, rhs.stable_id.value,
                      rhs.cell.z,          rhs.cell.y,        rhs.cell.x};
  });
  const std::size_t evaluation_count =
      std::min(boundary_candidates.size(), maximum_evaluations);
  std::unordered_map<std::uint64_t, ObservationFrontier> frontier_by_id;
  for (std::size_t index = 0U; index < evaluation_count; ++index) {
    ++result.evaluated_candidates;
    const BoundaryCandidate& candidate = boundary_candidates[index];
    result.evaluation_sample_fingerprint ^=
        candidate.stable_id.value + 0x9e3779b97f4a7c15ULL +
        (result.evaluation_sample_fingerprint << 6U) +
        (result.evaluation_sample_fingerprint >> 2U);
    ObservationFrontierSetEvaluation evaluation =
        evaluateObservationFrontiers(occupancy, occupancy.cellCenter(candidate.cell),
                                     map_revision, config, validation_policy);
    const std::size_t status_index =
        static_cast<std::size_t>(evaluation.evidence.status);
    if (status_index < result.evaluation_status_counts.size()) {
      ++result.evaluation_status_counts.at(status_index);
    }
    for (const ObservationFrontier& frontier : evaluation.frontiers) {
      auto [found, inserted] = frontier_by_id.try_emplace(frontier.id.value, frontier);
      if (!inserted &&
          betterFrontierRepresentative(frontier, found->second, validation_policy)) {
        found->second = frontier;
      }
    }
  }
  result.frontiers.reserve(frontier_by_id.size());
  for (auto& [id, frontier] : frontier_by_id) {
    static_cast<void>(id);
    result.frontiers.push_back(frontier);
  }
  std::ranges::sort(result.frontiers, {}, [](const ObservationFrontier& frontier) {
    return frontier.id.value;
  });
  return result;
}

} // namespace

bool observationPoseHasSupportedUnknownBoundary(
    const ObservedOccupancyGrid3D& occupancy, const GridIndex3D observation_cell,
    const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy) noexcept {
  return hasSupportedUnknownBoundaryImpl(occupancy, observation_cell, config,
                                         validation_policy);
}

bool sensorObservabilityConfigIsValid(
    const SensorObservabilityConfig& config) noexcept {
  return std::isfinite(config.maximum_observation_range_m) &&
         config.maximum_observation_range_m > 0.0 &&
         std::isfinite(config.minimum_known_free_ray_m) &&
         config.minimum_known_free_ray_m >= 0.0 &&
         std::isfinite(config.minimum_observation_pose_advance_m) &&
         config.minimum_observation_pose_advance_m > 0.0 &&
         config.minimum_observation_pose_advance_m <=
             config.maximum_observation_range_m &&
         std::isfinite(config.frontier_identity_resolution_m) &&
         config.frontier_identity_resolution_m > 0.0 &&
         std::isfinite(config.horizontal_min_angle_rad) &&
         std::isfinite(config.horizontal_max_angle_rad) &&
         config.horizontal_min_angle_rad < config.horizontal_max_angle_rad &&
         std::isfinite(config.vertical_min_angle_rad) &&
         std::isfinite(config.vertical_max_angle_rad) &&
         config.vertical_min_angle_rad < config.vertical_max_angle_rad &&
         config.vertical_min_angle_rad > -std::numbers::pi / 2.0 &&
         config.vertical_max_angle_rad < std::numbers::pi / 2.0 &&
         std::isfinite(config.directional_cluster_half_angle_rad) &&
         config.directional_cluster_half_angle_rad > 0.0 &&
         config.directional_cluster_half_angle_rad <= std::numbers::pi &&
         config.horizontal_samples > 1U && config.vertical_samples > 1U &&
         config.minimum_supporting_rays > 0U &&
         config.minimum_information_gain_voxels > 0U;
}

ObservationFrontierEvaluation evaluateObservationFrontier(
    const ObservedOccupancyGrid3D& occupancy, const Point3& observation_pose,
    const std::uint64_t map_revision, const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy) {
  ObservationFrontierSetEvaluation set = evaluateObservationFrontiers(
      occupancy, observation_pose, map_revision, config, validation_policy);
  ObservationFrontierEvaluation result{.evidence = set.evidence};
  if (set.accepted()) {
    result.frontier = set.frontiers.front();
  }
  return result;
}

ObservationFrontierSetEvaluation evaluateObservationFrontiers(
    const ObservedOccupancyGrid3D& occupancy, const Point3& observation_pose,
    const std::uint64_t map_revision, const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy) {
  ObservationFrontierSetEvaluation result;
  const SweptFootprintResult footprint =
      validateObservedFootprintAt(occupancy, observation_pose, FootprintBodyAxis{},
                                  config.footprint, validation_policy);
  result.evidence.status = footprintStatus(footprint.status);
  result.evidence.footprint_validation_accepted = footprint.accepted();
  if (!footprint.accepted() || !sensorObservabilityConfigIsValid(config)) {
    return result;
  }

  const double step_m = occupancy.bounds().resolution_m;
  if (!(step_m > 0.0)) {
    result.evidence.status = ObservationFrontierStatus::kOutsideMap;
    return result;
  }
  result.evidence.required_information_gain_voxels =
      std::max(config.minimum_information_gain_voxels,
               minimumFootprintInformationGainVoxels(config.footprint, step_m));
  std::vector<SupportedObservationRay> supported_rays;
  const std::size_t ray_sample_count =
      static_cast<std::size_t>(std::floor(config.maximum_observation_range_m / step_m));
  for (const Vec3& direction : sensorAlignedObservationDirections(config)) {
    ++result.evidence.tested_rays;
    bool found_unknown = false;
    std::optional<Point3> boundary_point;
    double known_free_ray_m = 0.0;
    double traversable_ray_m = 0.0;
    std::vector<GridIndex3D> ray_unknown_cells;
    for (std::size_t sample_index = 1U; sample_index <= ray_sample_count;
         ++sample_index) {
      const double distance_m = static_cast<double>(sample_index) * step_m;
      const Point3 sample{observation_pose.x + direction.x * distance_m,
                          observation_pose.y + direction.y * distance_m,
                          observation_pose.z + direction.z * distance_m};
      const std::optional<GridIndex3D> cell = occupancy.worldToCell(sample);
      if (!cell.has_value()) {
        break;
      }
      const ObservedVoxelState state = occupancy.state(*cell);
      if (state == ObservedVoxelState::kOccupied) {
        break;
      }
      traversable_ray_m = distance_m;
      if (state == ObservedVoxelState::kFree && !found_unknown) {
        known_free_ray_m = distance_m;
        continue;
      }
      if (state == ObservedVoxelState::kUnknown) {
        if (validation_policy == ObservedSpaceValidationPolicy::kRequireKnownFree &&
            known_free_ray_m + 1.0e-9 < config.minimum_known_free_ray_m) {
          break;
        }
        if (!found_unknown) {
          boundary_point = occupancy.cellCenter(*cell);
        }
        found_unknown = true;
        ray_unknown_cells.push_back(*cell);
      }
    }
    if (!found_unknown || ray_unknown_cells.empty()) {
      continue;
    }
    supported_rays.push_back(SupportedObservationRay{
        .direction = direction,
        .boundary_point = *boundary_point,
        .unknown_cells = std::move(ray_unknown_cells),
        .known_free_ray_m = known_free_ray_m,
        .traversable_ray_m = traversable_ray_m,
    });
  }

  std::vector<DirectionalObservationCluster> clusters = makeDirectionalClusters(
      supported_rays, config.directional_cluster_half_angle_rad, validation_policy);
  const auto best_cluster = std::ranges::max_element(
      clusters, {}, [validation_policy](const DirectionalObservationCluster& cluster) {
        return clusterRank(cluster, validation_policy);
      });
  if (best_cluster != clusters.end()) {
    result.evidence.information_gain_voxels =
        best_cluster->information_gain_cells.size();
    result.evidence.minimum_known_free_ray_m = best_cluster->minimum_known_free_ray_m;
    result.evidence.observation_direction = best_cluster->direction;
    result.evidence.boundary_centroid = best_cluster->boundary_centroid;
    result.evidence.supporting_rays = best_cluster->supporting_rays;
  }
  if (result.evidence.information_gain_voxels == 0U) {
    result.evidence.status = ObservationFrontierStatus::kNoUnknownBoundary;
    return result;
  }
  std::unordered_map<std::uint64_t, ObservationFrontier> frontier_by_id;
  for (const DirectionalObservationCluster& cluster : clusters) {
    if (cluster.supporting_rays < config.minimum_supporting_rays ||
        cluster.information_gain_cells.size() <
            result.evidence.required_information_gain_voxels) {
      continue;
    }
    const std::optional<ResolvedObservationPose> resolved_pose =
        resolveObservationPose(occupancy, observation_pose, supported_rays, cluster,
                               config, validation_policy);
    if (!resolved_pose.has_value()) {
      continue;
    }
    ObservationFrontier frontier{
        .id = makeFrontierId(cluster.boundary_centroid,
                             config.frontier_identity_resolution_m),
        .supporting_viewpoint = observation_pose,
        .observation_pose = resolved_pose->pose,
        .boundary_centroid = cluster.boundary_centroid,
        .observation_direction = cluster.direction,
        .supporting_map_revision = map_revision,
        .supporting_rays = cluster.supporting_rays,
        .information_gain_voxels = cluster.information_gain_cells.size(),
        .required_information_gain_voxels =
            result.evidence.required_information_gain_voxels,
        .minimum_known_free_ray_m = cluster.minimum_known_free_ray_m,
    };
    auto [found, inserted] = frontier_by_id.try_emplace(frontier.id.value, frontier);
    if (!inserted &&
        betterFrontierRepresentative(frontier, found->second, validation_policy)) {
      found->second = frontier;
    }
  }
  result.frontiers.reserve(frontier_by_id.size());
  for (auto& [id, frontier] : frontier_by_id) {
    static_cast<void>(id);
    result.frontiers.push_back(frontier);
  }
  std::ranges::sort(result.frontiers,
                    [validation_policy](const ObservationFrontier& lhs,
                                        const ObservationFrontier& rhs) {
                      if (betterFrontierRepresentative(lhs, rhs, validation_policy)) {
                        return true;
                      }
                      if (betterFrontierRepresentative(rhs, lhs, validation_policy)) {
                        return false;
                      }
                      return lhs.id < rhs.id;
                    });
  if (!result.frontiers.empty()) {
    result.evidence.status = ObservationFrontierStatus::kAccepted;
  } else if (result.evidence.supporting_rays < config.minimum_supporting_rays) {
    result.evidence.status = ObservationFrontierStatus::kInsufficientRaySupport;
  } else {
    result.evidence.status = ObservationFrontierStatus::kInsufficientInformationGain;
  }
  return result;
}

ObservationFrontierDiscovery discoverObservationFrontiers(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t map_revision,
    const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy,
    const std::size_t cell_stride, const std::size_t maximum_evaluations,
    const std::optional<ObservationFrontierDiscoveryRegion> region) {
  if (cell_stride == 0U || maximum_evaluations == 0U ||
      !sensorObservabilityConfigIsValid(config) ||
      (region.has_value() &&
       (!finitePoint(region->center) || !std::isfinite(region->maximum_distance_m) ||
        !(region->maximum_distance_m > 0.0) ||
        (region->mission_goal.has_value() && !finitePoint(*region->mission_goal))))) {
    return {};
  }

  using ChunkEntry = std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>;
  std::vector<ChunkEntry> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, storage] : occupancy.chunks()) {
    const ObservedOccupancyGrid3D::Chunk& chunk = storage.get();
    chunks.emplace_back(index, &chunk);
  }
  std::ranges::sort(chunks, {}, [](const ChunkEntry& entry) {
    return std::tuple{entry.first.z, entry.first.y, entry.first.x};
  });

  std::vector<BoundaryCandidate> boundary_candidates;
  std::size_t sampled_free_voxels = 0U;

  for (const auto& [chunk_index, chunk] : chunks) {
    for (std::size_t bit_index = 0U; bit_index < OccupancyGrid3D::kVoxelsPerChunk;
         ++bit_index) {
      if (!isSampledFreeCell(*chunk, bit_index)) {
        continue;
      }
      const int local_x =
          static_cast<int>(bit_index % ObservedOccupancyGrid3D::kChunkSize);
      const int local_y =
          static_cast<int>((bit_index / ObservedOccupancyGrid3D::kChunkSize) %
                           ObservedOccupancyGrid3D::kChunkSize);
      const int local_z = static_cast<int>(
          bit_index / static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize *
                                               ObservedOccupancyGrid3D::kChunkSize));
      const GridIndex3D cell{
          chunk_index.x * ObservedOccupancyGrid3D::kChunkSize + local_x,
          chunk_index.y * ObservedOccupancyGrid3D::kChunkSize + local_y,
          chunk_index.z * ObservedOccupancyGrid3D::kChunkSize + local_z};
      if (!occupancy.contains(cell) ||
          static_cast<std::size_t>(cell.x) % cell_stride != 0U ||
          static_cast<std::size_t>(cell.y) % cell_stride != 0U ||
          static_cast<std::size_t>(cell.z) % cell_stride != 0U) {
        continue;
      }
      const Point3 center = occupancy.cellCenter(cell);
      if (region.has_value() &&
          distance3D(center, region->center) > region->maximum_distance_m) {
        continue;
      }
      ++sampled_free_voxels;
      if (std::optional<BoundaryCandidate> candidate = makeBoundaryCandidate(
              occupancy, cell, map_revision, config, validation_policy,
              region.has_value() ? region->mission_goal : std::nullopt)) {
        boundary_candidates.push_back(*candidate);
      }
    }
  }
  return evaluateBoundaryCandidates(occupancy, map_revision, config, validation_policy,
                                    std::move(boundary_candidates), sampled_free_voxels,
                                    maximum_evaluations);
}

ObservationFrontierDiscovery discoverObservationFrontiersAtCells(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t map_revision,
    const SensorObservabilityConfig& config,
    const ObservedSpaceValidationPolicy validation_policy,
    const std::span<const GridIndex3D> candidate_cells,
    const std::size_t maximum_evaluations, const std::optional<Point3> mission_goal) {
  if (maximum_evaluations == 0U || !sensorObservabilityConfigIsValid(config) ||
      (mission_goal.has_value() && !finitePoint(*mission_goal))) {
    return {};
  }
  std::unordered_set<GridIndex3D, GridIndex3DHash> unique_cells;
  unique_cells.reserve(candidate_cells.size());
  std::vector<BoundaryCandidate> boundary_candidates;
  boundary_candidates.reserve(candidate_cells.size());
  std::size_t sampled_free_voxels = 0U;
  for (const GridIndex3D cell : candidate_cells) {
    if (!unique_cells.insert(cell).second || !occupancy.contains(cell) ||
        !occupancy.isKnownFree(cell)) {
      continue;
    }
    ++sampled_free_voxels;
    if (std::optional<BoundaryCandidate> candidate = makeViewpointCandidate(
            occupancy, cell, map_revision, config, validation_policy, mission_goal)) {
      boundary_candidates.push_back(*candidate);
    }
  }
  return evaluateBoundaryCandidates(occupancy, map_revision, config, validation_policy,
                                    std::move(boundary_candidates), sampled_free_voxels,
                                    maximum_evaluations);
}

const char*
observationFrontierStatusName(const ObservationFrontierStatus status) noexcept {
  switch (status) {
    case ObservationFrontierStatus::kAccepted:
      return "accepted";
    case ObservationFrontierStatus::kOutsideMap:
      return "outside_map";
    case ObservationFrontierStatus::kFootprintNotObserved:
      return "footprint_not_observed";
    case ObservationFrontierStatus::kRawCollision:
      return "raw_collision";
    case ObservationFrontierStatus::kNoUnknownBoundary:
      return "no_unknown_boundary";
    case ObservationFrontierStatus::kInsufficientRaySupport:
      return "insufficient_ray_support";
    case ObservationFrontierStatus::kInsufficientInformationGain:
      return "insufficient_information_gain";
  }
  return "unknown";
}

} // namespace drone_city_nav

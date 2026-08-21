#include "drone_city_nav/observation_frontier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

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

[[nodiscard]] Vec3 normalized(const Vec3& direction) noexcept {
  const double norm = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                                direction.z * direction.z);
  if (!(norm > 1.0e-9) || !std::isfinite(norm)) {
    return {};
  }
  return Vec3{direction.x / norm, direction.y / norm, direction.z / norm};
}

[[nodiscard]] const std::array<Vec3, 26>& observationDirections() {
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

void hashInteger(std::uint64_t& hash, const int value) noexcept {
  const std::uint64_t bits =
      static_cast<std::uint64_t>(static_cast<std::int64_t>(value));
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (bits >> shift) & 0xFFU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] ObservationFrontierId
makeFrontierId(const ObservedOccupancyGrid3D& occupancy, const Point3& pose,
               const Vec3& direction) noexcept {
  const std::optional<GridIndex3D> cell = occupancy.worldToCell(pose);
  if (!cell.has_value()) {
    return {};
  }
  std::uint64_t hash = kFnvOffsetBasis;
  hashInteger(hash, cell->x);
  hashInteger(hash, cell->y);
  hashInteger(hash, cell->z);
  hashInteger(hash, static_cast<int>(std::llround(direction.x * 1000.0)));
  hashInteger(hash, static_cast<int>(std::llround(direction.y * 1000.0)));
  hashInteger(hash, static_cast<int>(std::llround(direction.z * 1000.0)));
  return ObservationFrontierId{hash};
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

} // namespace

ObservationFrontierEvaluation evaluateObservationFrontier(
    const ObservedOccupancyGrid3D& occupancy, const Point3& observation_pose,
    const std::uint64_t map_revision, const SensorObservabilityConfig& config) {
  ObservationFrontierEvaluation result;
  const SweptFootprintResult footprint = validateRawFootprintAt(
      occupancy, observation_pose, FootprintBodyAxis{}, config.footprint);
  result.evidence.status = footprintStatus(footprint.status);
  result.evidence.footprint_observed_free = footprint.accepted();
  if (!footprint.accepted() || !(config.maximum_observation_range_m > 0.0) ||
      !(config.minimum_known_free_ray_m >= 0.0) ||
      config.minimum_supporting_rays == 0U ||
      config.minimum_information_gain_voxels == 0U) {
    return result;
  }

  const double step_m = occupancy.bounds().resolution_m;
  if (!(step_m > 0.0)) {
    result.evidence.status = ObservationFrontierStatus::kOutsideMap;
    return result;
  }
  std::unordered_set<GridIndex3D, GridIndex3DHash> information_gain_cells;
  Vec3 weighted_direction{};
  double minimum_support_m = std::numeric_limits<double>::infinity();
  const std::size_t ray_sample_count =
      static_cast<std::size_t>(std::floor(config.maximum_observation_range_m / step_m));
  for (const Vec3& direction : observationDirections()) {
    ++result.evidence.tested_rays;
    bool found_unknown = false;
    double known_free_ray_m = 0.0;
    std::size_t ray_information_gain = 0U;
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
      if (state == ObservedVoxelState::kFree && !found_unknown) {
        known_free_ray_m = distance_m;
        continue;
      }
      if (state == ObservedVoxelState::kUnknown) {
        if (known_free_ray_m + 1.0e-9 < config.minimum_known_free_ray_m) {
          break;
        }
        found_unknown = true;
        if (information_gain_cells.insert(*cell).second) {
          ++ray_information_gain;
        }
      }
    }
    if (!found_unknown || ray_information_gain == 0U) {
      continue;
    }
    ++result.evidence.supporting_rays;
    minimum_support_m = std::min(minimum_support_m, known_free_ray_m);
    weighted_direction.x += direction.x * static_cast<double>(ray_information_gain);
    weighted_direction.y += direction.y * static_cast<double>(ray_information_gain);
    weighted_direction.z += direction.z * static_cast<double>(ray_information_gain);
  }

  result.evidence.information_gain_voxels = information_gain_cells.size();
  result.evidence.minimum_known_free_ray_m =
      std::isfinite(minimum_support_m) ? minimum_support_m : 0.0;
  result.evidence.observation_direction = normalized(weighted_direction);
  if (result.evidence.information_gain_voxels == 0U) {
    result.evidence.status = ObservationFrontierStatus::kNoUnknownBoundary;
    return result;
  }
  if (result.evidence.supporting_rays < config.minimum_supporting_rays ||
      result.evidence.information_gain_voxels <
          config.minimum_information_gain_voxels) {
    result.evidence.status = ObservationFrontierStatus::kInsufficientRaySupport;
    return result;
  }
  result.evidence.status = ObservationFrontierStatus::kAccepted;
  result.frontier = ObservationFrontier{
      .id = makeFrontierId(occupancy, observation_pose,
                           result.evidence.observation_direction),
      .observation_pose = observation_pose,
      .observation_direction = result.evidence.observation_direction,
      .supporting_map_revision = map_revision,
      .supporting_rays = result.evidence.supporting_rays,
      .information_gain_voxels = result.evidence.information_gain_voxels,
      .minimum_known_free_ray_m = result.evidence.minimum_known_free_ray_m,
  };
  return result;
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
  }
  return "unknown";
}

} // namespace drone_city_nav

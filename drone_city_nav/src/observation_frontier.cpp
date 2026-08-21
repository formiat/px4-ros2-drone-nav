#include "drone_city_nav/observation_frontier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
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
makeFrontierId(const ObservedOccupancyGrid3D& occupancy, const Point3& pose) noexcept {
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

[[nodiscard]] bool
hasSupportedUnknownBoundary(const ObservedOccupancyGrid3D& occupancy,
                            const GridIndex3D origin,
                            const SensorObservabilityConfig& config) noexcept {
  const double resolution_m = occupancy.bounds().resolution_m;
  if (!(resolution_m > 0.0)) {
    return false;
  }
  const int known_free_steps = std::max(
      1, static_cast<int>(std::ceil(config.minimum_known_free_ray_m / resolution_m)));
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
    std::size_t ray_unknown_samples = 0U;
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
        ++ray_unknown_samples;
        static_cast<void>(information_gain_cells.insert(*cell));
      }
    }
    if (!found_unknown || ray_unknown_samples == 0U) {
      continue;
    }
    ++result.evidence.supporting_rays;
    minimum_support_m = std::min(minimum_support_m, known_free_ray_m);
    weighted_direction.x += direction.x * static_cast<double>(ray_unknown_samples);
    weighted_direction.y += direction.y * static_cast<double>(ray_unknown_samples);
    weighted_direction.z += direction.z * static_cast<double>(ray_unknown_samples);
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
      .id = makeFrontierId(occupancy, observation_pose),
      .observation_pose = observation_pose,
      .observation_direction = result.evidence.observation_direction,
      .supporting_map_revision = map_revision,
      .supporting_rays = result.evidence.supporting_rays,
      .information_gain_voxels = result.evidence.information_gain_voxels,
      .minimum_known_free_ray_m = result.evidence.minimum_known_free_ray_m,
  };
  return result;
}

ObservationFrontierDiscovery discoverObservationFrontiers(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t map_revision,
    const SensorObservabilityConfig& config, const std::size_t cell_stride,
    const std::size_t maximum_evaluations) {
  ObservationFrontierDiscovery result;
  if (cell_stride == 0U || maximum_evaluations == 0U) {
    return result;
  }

  using ChunkEntry = std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>;
  std::vector<ChunkEntry> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, chunk] : occupancy.chunks()) {
    chunks.emplace_back(index, &chunk);
  }
  std::ranges::sort(chunks, {}, [](const ChunkEntry& entry) {
    return std::tuple{entry.first.z, entry.first.y, entry.first.x};
  });

  struct BoundaryCandidate {
    GridIndex3D cell{};
    ObservationFrontierId stable_id{};
  };

  std::vector<BoundaryCandidate> boundary_candidates;

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
      ++result.sampled_free_voxels;
      if (!hasSupportedUnknownBoundary(occupancy, cell, config)) {
        continue;
      }
      boundary_candidates.push_back(BoundaryCandidate{
          .cell = cell,
          .stable_id = makeFrontierId(occupancy, occupancy.cellCenter(cell)),
      });
    }
  }

  result.boundary_candidates = boundary_candidates.size();
  result.evaluation_budget_exhausted = boundary_candidates.size() > maximum_evaluations;
  std::ranges::sort(boundary_candidates, [](const BoundaryCandidate& lhs,
                                            const BoundaryCandidate& rhs) {
    return std::tuple{lhs.stable_id.value, lhs.cell.z, lhs.cell.y, lhs.cell.x} <
           std::tuple{rhs.stable_id.value, rhs.cell.z, rhs.cell.y, rhs.cell.x};
  });
  const std::size_t evaluation_count =
      std::min(boundary_candidates.size(), maximum_evaluations);
  for (std::size_t index = 0U; index < evaluation_count; ++index) {
    ++result.evaluated_candidates;
    const BoundaryCandidate& candidate = boundary_candidates[index];
    ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
        occupancy, occupancy.cellCenter(candidate.cell), map_revision, config);
    if (evaluation.accepted()) {
      result.frontiers.push_back(evaluation.frontier);
    }
  }
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

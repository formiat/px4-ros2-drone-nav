#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

[[nodiscard]] bool validVolume(const DynamicAgentLidarVolume& volume) noexcept {
  return finitePoint(volume.position) && std::isfinite(volume.radius_m) &&
         volume.radius_m > 0.0 && std::isfinite(volume.lower_extent_m) &&
         volume.lower_extent_m >= 0.0 && std::isfinite(volume.upper_extent_m) &&
         volume.upper_extent_m >= 0.0;
}

[[nodiscard]] bool
cellIntersectsVolume(const GridBounds3D& bounds, const GridIndex3D cell,
                     const DynamicAgentLidarVolume& volume) noexcept {
  const Point3 cell_minimum{
      bounds.origin_x + static_cast<double>(cell.x) * bounds.resolution_m,
      bounds.origin_y + static_cast<double>(cell.y) * bounds.resolution_m,
      bounds.origin_z + static_cast<double>(cell.z) * bounds.resolution_m,
  };
  const Point3 cell_maximum{cell_minimum.x + bounds.resolution_m,
                            cell_minimum.y + bounds.resolution_m,
                            cell_minimum.z + bounds.resolution_m};
  const double volume_minimum_z = volume.position.z - volume.lower_extent_m;
  const double volume_maximum_z = volume.position.z + volume.upper_extent_m;
  if (cell_maximum.z < volume_minimum_z || cell_minimum.z > volume_maximum_z) {
    return false;
  }
  const double delta_x = std::max(
      {cell_minimum.x - volume.position.x, 0.0, volume.position.x - cell_maximum.x});
  const double delta_y = std::max(
      {cell_minimum.y - volume.position.y, 0.0, volume.position.y - cell_maximum.y});
  return delta_x * delta_x + delta_y * delta_y <= volume.radius_m * volume.radius_m;
}

[[nodiscard]] bool validConfig(const ObstacleMemory3DConfig& config) noexcept {
  return std::isfinite(config.maximum_range_m) && config.maximum_range_m > 0.0 &&
         std::isfinite(config.minimum_range_m) && config.minimum_range_m >= 0.0 &&
         config.minimum_range_m < config.maximum_range_m && config.scan_stride > 0 &&
         config.hit_weight > 0 && config.miss_weight > 0 &&
         config.minimum_score <= config.free_score && config.free_score < 0 &&
         config.occupied_score > 0 && config.occupied_score <= config.maximum_score &&
         config.free_score < config.occupied_score;
}

} // namespace

ObstacleMemory3D::ObstacleMemory3D(const GridBounds3D& bounds,
                                   ObstacleMemory3DConfig config)
    : config_{config},
      grid_{bounds} {
  if (!validConfig(config_)) {
    throw std::invalid_argument{"invalid ObstacleMemory3D config"};
  }
}

ObstacleMemory3DStats ObstacleMemory3D::integrateScan(const LidarScan3DView& scan) {
  ObstacleMemory3DStats stats{.source_beams = scan.beams.size()};
  if (!finitePoint(scan.origin_map)) {
    stats.invalid_beams = scan.beams.size();
    return stats;
  }
  const std::size_t stride = static_cast<std::size_t>(config_.scan_stride);
  for (std::size_t index = 0U; index < scan.beams.size(); index += stride) {
    const LidarBeam3D& beam = scan.beams[index];
    const double direction_norm = vectorNorm(beam.direction_map);
    if (!beam.valid || !std::isfinite(beam.range_m) ||
        beam.range_m < config_.minimum_range_m || !(direction_norm > 1.0e-9) ||
        !std::isfinite(direction_norm)) {
      ++stats.invalid_beams;
      continue;
    }
    ++stats.processed_beams;
    stats.hit_beams += beam.hit ? 1U : 0U;
    stats.miss_beams += beam.hit ? 0U : 1U;
    integrateRay(scan.origin_map, beam, stats);
  }
  if (stats.state_transitions > 0U) {
    ++revision_;
  }
  return stats;
}

std::size_t ObstacleMemory3D::forgetDynamicVolumes(
    const std::span<const DynamicAgentLidarVolume> volumes) {
  const GridBounds3D& bounds = grid_.bounds();
  std::size_t forgotten_voxels{0U};
  for (const DynamicAgentLidarVolume& volume : volumes) {
    if (!validVolume(volume)) {
      continue;
    }
    const auto cell_index = [&](const double coordinate, const double origin) noexcept {
      return static_cast<int>(std::floor((coordinate - origin) / bounds.resolution_m));
    };
    const int minimum_x =
        std::max(0, cell_index(volume.position.x - volume.radius_m, bounds.origin_x));
    const int maximum_x =
        std::min(bounds.width_cells - 1,
                 cell_index(volume.position.x + volume.radius_m, bounds.origin_x));
    const int minimum_y =
        std::max(0, cell_index(volume.position.y - volume.radius_m, bounds.origin_y));
    const int maximum_y =
        std::min(bounds.height_cells - 1,
                 cell_index(volume.position.y + volume.radius_m, bounds.origin_y));
    const int minimum_z = std::max(
        0, cell_index(volume.position.z - volume.lower_extent_m, bounds.origin_z));
    const int maximum_z = std::min(
        bounds.depth_cells - 1,
        cell_index(volume.position.z + volume.upper_extent_m, bounds.origin_z));
    if (minimum_x > maximum_x || minimum_y > maximum_y || minimum_z > maximum_z) {
      continue;
    }
    for (int z = minimum_z; z <= maximum_z; ++z) {
      for (int y = minimum_y; y <= maximum_y; ++y) {
        for (int x = minimum_x; x <= maximum_x; ++x) {
          const GridIndex3D cell{x, y, z};
          if (!cellIntersectsVolume(bounds, cell, volume)) {
            continue;
          }
          const OccupancyChunkIndex3D chunk_index =
              ObservedOccupancyGrid3D::chunkIndex(cell);
          const auto evidence = evidence_.find(chunk_index);
          if (evidence != evidence_.end()) {
            evidence->second.scores.at(ObservedOccupancyGrid3D::localBitIndex(cell)) =
                0;
          }
          if (grid_.state(cell) == ObservedVoxelState::kUnknown) {
            continue;
          }
          static_cast<void>(grid_.setState(cell, ObservedVoxelState::kUnknown));
          dirty_chunks_[chunk_index] = true;
          ++forgotten_voxels;
        }
      }
    }
  }
  if (forgotten_voxels > 0U) {
    ++revision_;
  }
  return forgotten_voxels;
}

void ObstacleMemory3D::reset() {
  grid_.clear();
  evidence_.clear();
  dirty_chunks_.clear();
  ++revision_;
  full_reset_pending_ = true;
}

const ObservedOccupancyGrid3D& ObstacleMemory3D::grid() const noexcept {
  return grid_;
}

std::uint64_t ObstacleMemory3D::revision() const noexcept {
  return revision_;
}

ObstacleMemory3DChanges ObstacleMemory3D::takeChanges() {
  ObstacleMemory3DChanges changes;
  changes.revision = revision_;
  changes.full_reset = std::exchange(full_reset_pending_, false);
  changes.dirty_chunks.reserve(dirty_chunks_.size());
  for (const auto& [index, unused] : dirty_chunks_) {
    static_cast<void>(unused);
    changes.dirty_chunks.push_back(index);
  }
  std::ranges::sort(changes.dirty_chunks, [](const OccupancyChunkIndex3D& first,
                                             const OccupancyChunkIndex3D& second) {
    if (first.z != second.z) {
      return first.z < second.z;
    }
    if (first.y != second.y) {
      return first.y < second.y;
    }
    return first.x < second.x;
  });
  dirty_chunks_.clear();
  return changes;
}

bool ObstacleMemory3D::applyEvidence(const GridIndex3D index, const int delta,
                                     ObstacleMemory3DStats& stats) {
  if (!grid_.contains(index)) {
    return false;
  }
  const OccupancyChunkIndex3D chunk_index = ObservedOccupancyGrid3D::chunkIndex(index);
  EvidenceChunk& evidence = evidence_[chunk_index];
  const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(index);
  const int before_score = evidence.scores.at(bit_index);
  const int after_score =
      std::clamp(before_score + delta, config_.minimum_score, config_.maximum_score);
  evidence.scores.at(bit_index) = static_cast<std::int16_t>(after_score);
  const ObservedVoxelState before = grid_.state(index);
  ObservedVoxelState after = ObservedVoxelState::kUnknown;
  if (after_score >= config_.occupied_score) {
    after = ObservedVoxelState::kOccupied;
  } else if (after_score <= config_.free_score) {
    after = ObservedVoxelState::kFree;
  }
  if (before == after) {
    return false;
  }
  static_cast<void>(grid_.setState(index, after));
  dirty_chunks_[chunk_index] = true;
  ++stats.state_transitions;
  return true;
}

void ObstacleMemory3D::integrateRay(const Point3& origin, const LidarBeam3D& beam,
                                    ObstacleMemory3DStats& stats) {
  const double norm = vectorNorm(beam.direction_map);
  const Vec3 direction{beam.direction_map.x / norm, beam.direction_map.y / norm,
                       beam.direction_map.z / norm};
  const double used_range = std::min(beam.range_m, config_.maximum_range_m);
  const Point3 endpoint{origin.x + used_range * direction.x,
                        origin.y + used_range * direction.y,
                        origin.z + used_range * direction.z};
  const std::optional<GridIndex3D> hit_cell =
      beam.hit ? grid_.worldToCell(endpoint) : std::nullopt;
  if (beam.hit && !hit_cell.has_value()) {
    ++stats.outside_endpoints;
  }

  const double sample_step_m = 0.5 * grid_.bounds().resolution_m;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::max(1.0, std::ceil(used_range / sample_step_m)));
  std::optional<GridIndex3D> previous;
  for (std::size_t sample = 0U; sample <= sample_count; ++sample) {
    const double distance_m =
        std::min(used_range, static_cast<double>(sample) * used_range /
                                 static_cast<double>(sample_count));
    const Point3 point{origin.x + distance_m * direction.x,
                       origin.y + distance_m * direction.y,
                       origin.z + distance_m * direction.z};
    const std::optional<GridIndex3D> cell = grid_.worldToCell(point);
    if (!cell.has_value() || (previous.has_value() && *previous == *cell)) {
      continue;
    }
    previous = cell;
    if (hit_cell.has_value() && *cell == *hit_cell) {
      continue;
    }
    static_cast<void>(applyEvidence(*cell, -config_.miss_weight, stats));
    ++stats.free_voxel_updates;
  }
  if (hit_cell.has_value()) {
    static_cast<void>(applyEvidence(*hit_cell, config_.hit_weight, stats));
    ++stats.occupied_voxel_updates;
  }
}

} // namespace drone_city_nav

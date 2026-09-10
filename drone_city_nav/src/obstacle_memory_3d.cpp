#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
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

struct ClippedGridSegment3D {
  Point3 first{};
  Point3 second{};
};

[[nodiscard]] std::optional<ClippedGridSegment3D>
clipSegmentToGrid(const GridBounds3D& bounds, const Point3& first,
                  const Point3& second) noexcept {
  const Vec3 delta{second.x - first.x, second.y - first.y, second.z - first.z};
  double entry = 0.0;
  double exit = 1.0;
  const auto clip_axis = [&entry, &exit](const double origin, const double direction,
                                         const double minimum,
                                         const double maximum) noexcept {
    constexpr double kDirectionEpsilon{1.0e-12};
    if (std::abs(direction) <= kDirectionEpsilon) {
      return origin >= minimum && origin < maximum;
    }
    double near = (minimum - origin) / direction;
    double far = (maximum - origin) / direction;
    if (near > far) {
      std::swap(near, far);
    }
    entry = std::max(entry, near);
    exit = std::min(exit, far);
    return entry <= exit;
  };
  const double maximum_x = bounds.origin_x + bounds.resolution_m * bounds.width_cells;
  const double maximum_y = bounds.origin_y + bounds.resolution_m * bounds.height_cells;
  const double maximum_z = bounds.origin_z + bounds.resolution_m * bounds.depth_cells;
  if (!clip_axis(first.x, delta.x, bounds.origin_x, maximum_x) ||
      !clip_axis(first.y, delta.y, bounds.origin_y, maximum_y) ||
      !clip_axis(first.z, delta.z, bounds.origin_z, maximum_z)) {
    return std::nullopt;
  }

  const auto clamp_inside = [](const double value, const double minimum,
                               const double maximum) noexcept {
    return std::clamp(value, minimum, std::nextafter(maximum, minimum));
  };
  const auto point_at = [&first, &delta](const double ratio) noexcept {
    return Point3{first.x + ratio * delta.x, first.y + ratio * delta.y,
                  first.z + ratio * delta.z};
  };
  Point3 clipped_first = point_at(entry);
  Point3 clipped_second = point_at(exit);
  clipped_first.x = clamp_inside(clipped_first.x, bounds.origin_x, maximum_x);
  clipped_first.y = clamp_inside(clipped_first.y, bounds.origin_y, maximum_y);
  clipped_first.z = clamp_inside(clipped_first.z, bounds.origin_z, maximum_z);
  clipped_second.x = clamp_inside(clipped_second.x, bounds.origin_x, maximum_x);
  clipped_second.y = clamp_inside(clipped_second.y, bounds.origin_y, maximum_y);
  clipped_second.z = clamp_inside(clipped_second.z, bounds.origin_z, maximum_z);
  return ClippedGridSegment3D{.first = clipped_first, .second = clipped_second};
}

template<typename Visitor>
void visitIntersectedGridCells(const ObservedOccupancyGrid3D& grid, const Point3& first,
                               const Point3& second, Visitor visitor) {
  const std::optional<ClippedGridSegment3D> clipped =
      clipSegmentToGrid(grid.bounds(), first, second);
  if (!clipped.has_value()) {
    return;
  }
  const std::optional<GridIndex3D> first_cell = grid.worldToCell(clipped->first);
  const std::optional<GridIndex3D> last_cell = grid.worldToCell(clipped->second);
  if (!first_cell.has_value() || !last_cell.has_value()) {
    return;
  }

  GridIndex3D current = *first_cell;
  visitor(current);
  if (current == *last_cell) {
    return;
  }

  const GridBounds3D& bounds = grid.bounds();
  const Vec3 delta{clipped->second.x - clipped->first.x,
                   clipped->second.y - clipped->first.y,
                   clipped->second.z - clipped->first.z};
  const auto step_for = [](const double value) noexcept {
    if (value > 0.0) {
      return 1;
    }
    if (value < 0.0) {
      return -1;
    }
    return 0;
  };
  const int step_x = step_for(delta.x);
  const int step_y = step_for(delta.y);
  const int step_z = step_for(delta.z);
  const double infinity = std::numeric_limits<double>::infinity();
  const auto parameter_delta = [resolution = bounds.resolution_m,
                                infinity](const double value) noexcept {
    return value == 0.0 ? infinity : resolution / std::abs(value);
  };
  const auto next_boundary_parameter = [resolution = bounds.resolution_m, infinity](
                                           const double coordinate, const double origin,
                                           const int cell, const int step,
                                           const double direction) noexcept {
    if (step == 0) {
      return infinity;
    }
    const double boundary =
        origin + static_cast<double>(cell + (step > 0 ? 1 : 0)) * resolution;
    return std::max(0.0, (boundary - coordinate) / direction);
  };
  double next_x = next_boundary_parameter(clipped->first.x, bounds.origin_x, current.x,
                                          step_x, delta.x);
  double next_y = next_boundary_parameter(clipped->first.y, bounds.origin_y, current.y,
                                          step_y, delta.y);
  double next_z = next_boundary_parameter(clipped->first.z, bounds.origin_z, current.z,
                                          step_z, delta.z);
  const double delta_x = parameter_delta(delta.x);
  const double delta_y = parameter_delta(delta.y);
  const double delta_z = parameter_delta(delta.z);
  const std::size_t maximum_cells = static_cast<std::size_t>(bounds.width_cells) +
                                    static_cast<std::size_t>(bounds.height_cells) +
                                    static_cast<std::size_t>(bounds.depth_cells) + 3U;
  constexpr double kTieTolerance{1.0e-12};
  for (std::size_t visited = 1U; visited < maximum_cells && current != *last_cell;
       ++visited) {
    const double next = std::min({next_x, next_y, next_z});
    if (!std::isfinite(next)) {
      return;
    }
    if (next_x <= next + kTieTolerance) {
      current.x += step_x;
      next_x += delta_x;
    }
    if (next_y <= next + kTieTolerance) {
      current.y += step_y;
      next_y += delta_y;
    }
    if (next_z <= next + kTieTolerance) {
      current.z += step_z;
      next_z += delta_z;
    }
    if (!grid.contains(current)) {
      return;
    }
    visitor(current);
  }
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
         config.near_hit_range_m > 0.0 && std::isfinite(config.minimum_range_m) &&
         config.minimum_range_m >= 0.0 &&
         config.minimum_range_m < config.maximum_range_m && config.scan_stride > 0 &&
         config.hit_weight > 0 && config.miss_weight > 0 &&
         config.minimum_score <= config.free_score && config.free_score < 0 &&
         config.occupied_score > 0 && config.occupied_score <= config.maximum_score &&
         config.free_score < config.occupied_score &&
         std::isfinite(config.nominal_evidence_interval_s) &&
         config.nominal_evidence_interval_s > 0.0 &&
         std::isfinite(config.maximum_evidence_interval_s) &&
         config.maximum_evidence_interval_s >= config.nominal_evidence_interval_s;
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
  static_cast<void>(evidenceIntervalSeconds(scan, stats));
  if (stats.stale_acquisition) {
    return stats;
  }
  const std::size_t stride = static_cast<std::size_t>(config_.scan_stride);
  ScanEvidence scan_evidence;
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
    stats.surface_beams += beam.surface_only ? 1U : 0U;
    integrateRay(scan.origin_map, beam, scan_evidence, stats);
  }
  for (const auto& [chunk_index, chunk_evidence] : scan_evidence) {
    applyChunkEvidence(chunk_index, chunk_evidence, stats);
  }
  if (stats.state_transitions > 0U) {
    ++revision_;
  }
  return stats;
}

double ObstacleMemory3D::evidenceIntervalSeconds(const LidarScan3DView& scan,
                                                 ObstacleMemory3DStats& stats) {
  if (scan.acquisition_stamp_ns <= 0) {
    stats.evidence_interval_s = config_.nominal_evidence_interval_s;
    return stats.evidence_interval_s;
  }
  if (last_evidence_stamp_ns_ > 0 &&
      scan.acquisition_stamp_ns <= last_evidence_stamp_ns_) {
    stats.stale_acquisition = true;
    return 0.0;
  }
  const double interval_s =
      last_evidence_stamp_ns_ <= 0
          ? config_.nominal_evidence_interval_s
          : 1.0e-9 * static_cast<double>(scan.acquisition_stamp_ns -
                                         last_evidence_stamp_ns_);
  last_evidence_stamp_ns_ = scan.acquisition_stamp_ns;
  stats.evidence_interval_s = std::min(interval_s, config_.maximum_evidence_interval_s);
  return stats.evidence_interval_s;
}

GridIndex3D ObstacleMemory3D::cellFromChunkBit(const OccupancyChunkIndex3D chunk,
                                               const std::size_t bit_index) noexcept {
  constexpr std::size_t kChunkSize =
      static_cast<std::size_t>(OccupancyGrid3D::kChunkSize);
  const std::size_t local_z = bit_index / (kChunkSize * kChunkSize);
  const std::size_t local_y = (bit_index / kChunkSize) % kChunkSize;
  const std::size_t local_x = bit_index % kChunkSize;
  return GridIndex3D{
      .x = chunk.x * OccupancyGrid3D::kChunkSize + static_cast<int>(local_x),
      .y = chunk.y * OccupancyGrid3D::kChunkSize + static_cast<int>(local_y),
      .z = chunk.z * OccupancyGrid3D::kChunkSize + static_cast<int>(local_z),
  };
}

void ObstacleMemory3D::recordScanEvidence(const GridIndex3D index, const bool occupied,
                                          const bool far, ScanEvidence& scan_evidence,
                                          ScanEvidenceCursor& cursor) const {
  const OccupancyChunkIndex3D chunk_index = ObservedOccupancyGrid3D::chunkIndex(index);
  if (cursor.chunk == nullptr || !(cursor.chunk_index == chunk_index)) {
    // Node-based map: the element address stays valid across later inserts.
    cursor.chunk = std::addressof(scan_evidence[chunk_index]);
    cursor.chunk_index = chunk_index;
  }
  ScanEvidenceChunk& chunk = *cursor.chunk;
  const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(index);
  const std::size_t word_index = bit_index / 64U;
  const std::uint64_t bit = std::uint64_t{1U} << (bit_index % 64U);
  chunk.observed[word_index] |= bit;
  if (occupied) {
    chunk.occupied[word_index] |= bit;
    if (far) {
      chunk.far[word_index] |= bit;
    }
  }
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
  last_evidence_stamp_ns_ = 0;
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

void ObstacleMemory3D::applyChunkEvidence(const OccupancyChunkIndex3D chunk_index,
                                          const ScanEvidenceChunk& chunk_evidence,
                                          ObstacleMemory3DStats& stats) {
  const double hit_delta = static_cast<double>(config_.hit_weight);
  const double miss_delta = -static_cast<double>(config_.miss_weight);
  const auto minimum_score = static_cast<double>(config_.minimum_score);
  const auto maximum_score = static_cast<double>(config_.maximum_score);
  EvidenceChunk* scores{nullptr};
  const ObservedOccupancyGrid3D::Chunk* grid_chunk = grid_.findChunk(chunk_index);
  bool chunk_dirty{false};
  for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
       ++word_index) {
    std::uint64_t remaining = chunk_evidence.observed[word_index];
    const std::uint64_t occupied_word = chunk_evidence.occupied[word_index];
    const std::uint64_t far_word = chunk_evidence.far[word_index];
    while (remaining != 0U) {
      const std::size_t bit_offset =
          static_cast<std::size_t>(std::countr_zero(remaining));
      remaining &= remaining - 1U;
      const std::size_t bit_index = word_index * 64U + bit_offset;
      const bool occupied = (occupied_word & (std::uint64_t{1U} << bit_offset)) != 0U;
      stats.occupied_voxel_updates += occupied ? 1U : 0U;
      stats.free_voxel_updates += occupied ? 0U : 1U;
      const GridIndex3D cell = cellFromChunkBit(chunk_index, bit_index);
      if (!grid_.contains(cell)) {
        continue;
      }
      if (scores == nullptr) {
        scores = std::addressof(evidence_[chunk_index]);
      }
      // A far hit raises the score no higher than the occupied threshold and
      // never lowers it: the voxel is occupied on that evidence, and the first
      // near look through it frees it in as many misses as the threshold is
      // above the free one, where a near-saturated voxel takes the full climb.
      const bool far = (far_word & (std::uint64_t{1U} << bit_offset)) != 0U;
      const double before_score = scores->scores[bit_index];
      const double after_score =
          occupied && far
              ? std::max(before_score,
                         std::min(before_score + hit_delta,
                                  static_cast<double>(config_.occupied_score)))
              : std::clamp(before_score + (occupied ? hit_delta : miss_delta),
                           minimum_score, maximum_score);
      scores->scores[bit_index] = after_score;
      const ObservedVoxelState before =
          grid_chunk != nullptr
              ? ObservedOccupancyGrid3D::chunkState(*grid_chunk, bit_index)
              : ObservedVoxelState::kUnknown;
      // Schmitt-trigger classification: a voxel enters the occupied or free
      // state when its score crosses that state's threshold and keeps its state
      // while the score stays between the thresholds. Without the hysteresis a
      // wall surface voxel that collects one hit and a few grazing misses per
      // scan flips its state every scan, and every consumer of occupied
      // evidence re-validates the same geometry at the scan rate.
      ObservedVoxelState after = before;
      if (after_score >= config_.occupied_score) {
        after = ObservedVoxelState::kOccupied;
      } else if (after_score <= config_.free_score) {
        after = ObservedVoxelState::kFree;
      }
      if (before == after) {
        continue;
      }
      static_cast<void>(grid_.setState(cell, after));
      // The first observed voxel of a chunk materializes it.
      grid_chunk = grid_.findChunk(chunk_index);
      chunk_dirty = true;
      ++stats.state_transitions;
    }
  }
  if (chunk_dirty) {
    dirty_chunks_[chunk_index] = true;
  }
}

void ObstacleMemory3D::integrateRay(const Point3& origin, const LidarBeam3D& beam,
                                    ScanEvidence& scan_evidence,
                                    ObstacleMemory3DStats& stats) const {
  const double norm = vectorNorm(beam.direction_map);
  const Vec3 direction{beam.direction_map.x / norm, beam.direction_map.y / norm,
                       beam.direction_map.z / norm};
  const double used_range = std::min(beam.range_m, config_.maximum_range_m);
  const Point3 endpoint{origin.x + used_range * direction.x,
                        origin.y + used_range * direction.y,
                        origin.z + used_range * direction.z};
  const bool hit_within_range = beam.hit && beam.range_m <= config_.maximum_range_m;
  const bool far_hit = hit_within_range && beam.range_m > config_.near_hit_range_m;
  const std::optional<GridIndex3D> hit_cell =
      hit_within_range ? grid_.worldToCell(endpoint) : std::nullopt;
  if (hit_within_range && !hit_cell.has_value()) {
    ++stats.outside_endpoints;
  }
  ScanEvidenceCursor cursor;
  if (beam.surface_only) {
    if (hit_cell.has_value()) {
      recordScanEvidence(*hit_cell, true, far_hit, scan_evidence, cursor);
    }
    return;
  }
  visitIntersectedGridCells(grid_, origin, endpoint, [&](const GridIndex3D cell) {
    if (!hit_cell.has_value() || cell != *hit_cell) {
      recordScanEvidence(cell, false, false, scan_evidence, cursor);
    }
  });
  if (hit_cell.has_value()) {
    recordScanEvidence(*hit_cell, true, far_hit, scan_evidence, cursor);
  }
}

} // namespace drone_city_nav

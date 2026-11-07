#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>

// The obstacle memory's moving layer (roadmap item 21, specification K65):
// what the layer holds, what joins it and what leaves it.

namespace drone_city_nav {

bool ObstacleMemory3D::heldMoving(const GridIndex3D cell) const noexcept {
  const auto chunk = moving_.find(ObservedOccupancyGrid3D::chunkIndex(cell));
  if (chunk == moving_.end()) {
    return false;
  }
  const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(cell);
  return (chunk->second.held.at(bit_index / 64U) &
          (std::uint64_t{1U} << (bit_index % 64U))) != 0U;
}

std::uint16_t ObstacleMemory3D::confirmationsAt(const GridIndex3D cell) const noexcept {
  const auto scores = evidence_.find(ObservedOccupancyGrid3D::chunkIndex(cell));
  return scores == evidence_.end() ? std::uint16_t{0U}
                                   : scores->second.confirmations.at(
                                         ObservedOccupancyGrid3D::localBitIndex(cell));
}

void ObstacleMemory3D::joinMoving(const GridIndex3D cell, const std::int64_t now_ms,
                                  ObstacleMemory3DStats& stats) {
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const GridIndex3D near{cell.x + dx, cell.y + dy, cell.z + dz};
        if ((dx == 0 && dy == 0 && dz == 0) || !grid_.contains(near) ||
            !grid_.isOccupied(near) || grid_.isUnobservable(near) || heldMoving(near) ||
            confirmationsAt(near) > kYoungConfirmations) {
          continue;
        }
        holdMoving(near, now_ms, stats);
      }
    }
  }
}

bool ObstacleMemory3D::movingEvidenceAt(const GridIndex3D cell) const noexcept {
  if (heldMoving(cell)) {
    return true;
  }
  const bool occupied = grid_.isOccupied(cell);
  if (occupied &&
      (grid_.isUnobservable(cell) || confirmationsAt(cell) > kYoungConfirmations)) {
    // A surface seen for long, or the dark: never a body.
    return false;
  }
  // A voxel beside one the layer holds joins it, free, unknown or a young
  // occupancy: the cluster that shifts. Away from the layer, a hit where free
  // space had been observed is a body's appearance, a hit in unknown space or
  // on a young occupancy is nothing yet, and a surface voxel's neighbour
  // takes a return now and then from the range's noise: ordinary, as the
  // surface is, unless the body walks there (r3013: a body turning a quarter
  // metre from a wall left its wall-side cells).
  bool beside_ordinary{false};
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const GridIndex3D near{cell.x + dx, cell.y + dy, cell.z + dz};
        if ((dx == 0 && dy == 0 && dz == 0) || !grid_.contains(near) ||
            !grid_.isOccupied(near)) {
          continue;
        }
        if (heldMoving(near)) {
          return true;
        }
        beside_ordinary = true;
      }
    }
  }
  return !occupied && grid_.state(cell) == ObservedVoxelState::kFree &&
         !beside_ordinary;
}

void ObstacleMemory3D::liftSweep() {
  for (const auto& [cell, before] : swept_) {
    // Still the sweep's: a scan's evidence never reaches a swept voxel, as the
    // sweep is lifted before the evidence applies.
    static_cast<void>(grid_.setUnobservable(cell, false));
    static_cast<void>(grid_.setState(cell, before));
    dirty_chunks_[ObservedOccupancyGrid3D::chunkIndex(cell)] = true;
  }
  swept_.clear();
}

void ObstacleMemory3D::holdMoving(const GridIndex3D cell, const std::int64_t now_ms,
                                  ObstacleMemory3DStats& stats) {
  const OccupancyChunkIndex3D chunk_index = ObservedOccupancyGrid3D::chunkIndex(cell);
  const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(cell);
  MovingChunk& held = moving_[chunk_index];
  held.held.at(bit_index / 64U) |= std::uint64_t{1U} << (bit_index % 64U);
  ++held.count;
  const auto stamp_ms = static_cast<std::int32_t>(
      std::clamp<std::int64_t>(now_ms, 0, std::numeric_limits<std::int32_t>::max()));
  held.since_ms.at(bit_index) = stamp_ms;
  held.last_ms.at(bit_index) = stamp_ms;
  const auto scores = evidence_.find(chunk_index);
  if (scores != evidence_.end()) {
    scores->second.scores.at(bit_index) = static_cast<double>(config_.occupied_score);
    scores->second.confirmations.at(bit_index) = 0U;
  }
  ++stats.moving_joined;
}

void ObstacleMemory3D::followMoving(const ScanEvidence& scan_evidence,
                                    const Point3& origin, const std::int64_t now_ms,
                                    ObstacleMemory3DStats& stats) {
  // The layer's voxels, and the young occupancies this scan hit: a body first
  // seen where nothing had been observed is told by its step.
  std::vector<GridIndex3D> cells;
  for (const auto& [chunk_index, chunk] : moving_) {
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      std::uint64_t remaining = chunk.held.at(word_index);
      while (remaining != 0U) {
        const auto bit_offset = static_cast<std::size_t>(std::countr_zero(remaining));
        remaining &= remaining - 1U;
        cells.push_back(cellFromChunkBit(chunk_index, word_index * 64U + bit_offset));
      }
    }
  }
  for (const auto& [chunk_index, chunk_evidence] : scan_evidence) {
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      std::uint64_t remaining = chunk_evidence.occupied.at(word_index);
      while (remaining != 0U) {
        const auto bit_offset = static_cast<std::size_t>(std::countr_zero(remaining));
        remaining &= remaining - 1U;
        const GridIndex3D cell =
            cellFromChunkBit(chunk_index, word_index * 64U + bit_offset);
        if (grid_.isOccupied(cell) && !grid_.isUnobservable(cell) &&
            !heldMoving(cell) && confirmationsAt(cell) <= kYoungConfirmations) {
          cells.push_back(cell);
        }
      }
    }
  }
  tracker_.update(cells, grid_.bounds(), now_ms);
  followed_bodies_.clear();
  for (std::size_t index = 0U; index < tracker_.bodies().size(); ++index) {
    const MovingBody3D& body = tracker_.bodies()[index];
    bool held = false;
    for (const GridIndex3D cell : tracker_.cells(index)) {
      held = held || heldMoving(cell);
    }
    if (body.moving()) {
      // A cluster that stepped is a body: its voxels, and the ones it had the
      // scan before, are the layer's from now.
      for (const std::span<const GridIndex3D> group :
           {tracker_.cells(index), tracker_.previousCells(index)}) {
        for (const GridIndex3D cell : group) {
          if (grid_.isOccupied(cell) && !grid_.isUnobservable(cell) &&
              !heldMoving(cell) && confirmationsAt(cell) <= kYoungConfirmations) {
            holdMoving(cell, now_ms, stats);
          }
        }
      }
      held = true;
    }
    if (held) {
      followed_bodies_.push_back(body);
    }
  }
  stats.followed_bodies = followed_bodies_.size();
  if (!(config_.moving_sweep_s > 0.0)) {
    return;
  }
  // The vehicle's own place is never swept: a body walking up to a hovering
  // vehicle leaves it a way out (the exit the law owes, K66).
  constexpr double kKeepsClearOfVehicleM{1.0};
  for (const MovingBody3D& body : followed_bodies_) {
    for (const GridIndex3D cell :
         sweptCells3D(body, config_.moving_sweep_s, grid_.bounds())) {
      if (grid_.isOccupied(cell) ||
          distance3D(grid_.cellCenter(cell), origin) <= kKeepsClearOfVehicleM) {
        continue;
      }
      const ObservedVoxelState before = grid_.state(cell);
      static_cast<void>(grid_.setState(cell, ObservedVoxelState::kOccupied));
      static_cast<void>(grid_.setUnobservable(cell, true));
      dirty_chunks_[ObservedOccupancyGrid3D::chunkIndex(cell)] = true;
      swept_.emplace_back(cell, before);
      ++stats.swept_voxels;
    }
  }
}

void ObstacleMemory3D::settleMoving(const std::int64_t now_ms,
                                    ObstacleMemory3DStats& stats) {
  const auto life_ms = static_cast<std::int64_t>(1000.0 * config_.moving_life_s);
  const auto stand_ms = static_cast<std::int64_t>(1000.0 * config_.moving_stand_s);
  for (auto chunk = moving_.begin(); chunk != moving_.end();) {
    MovingChunk& held = chunk->second;
    bool chunk_dirty{false};
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      std::uint64_t remaining = held.held.at(word_index);
      while (remaining != 0U) {
        const std::size_t bit_offset =
            static_cast<std::size_t>(std::countr_zero(remaining));
        remaining &= remaining - 1U;
        const std::size_t bit_index = word_index * 64U + bit_offset;
        const std::int64_t since_last = now_ms - held.last_ms.at(bit_index);
        const bool expired = since_last > life_ms;
        const bool stood =
            since_last == 0 && now_ms - held.since_ms.at(bit_index) >= stand_ms;
        if (!expired && !stood) {
          continue;
        }
        held.held.at(word_index) &= ~(std::uint64_t{1U} << bit_offset);
        --held.count;
        const GridIndex3D cell = cellFromChunkBit(chunk->first, bit_index);
        const auto scores = evidence_.find(chunk->first);
        if (expired) {
          if (scores != evidence_.end()) {
            scores->second.scores.at(bit_index) = 0.0;
            scores->second.confirmations.at(bit_index) = 0U;
          }
          if (grid_.contains(cell) && grid_.isOccupied(cell)) {
            static_cast<void>(grid_.setState(cell, ObservedVoxelState::kUnknown));
            chunk_dirty = true;
            ++stats.state_transitions;
          }
          ++stats.moving_expired;
        } else {
          // Stood for the stand's time under the sensor's eye: a wall from now,
          // with the score of one seen as long.
          if (scores != evidence_.end()) {
            scores->second.scores.at(bit_index) =
                static_cast<double>(config_.maximum_score);
          }
          ++stats.moving_promoted;
        }
      }
    }
    if (chunk_dirty) {
      dirty_chunks_[chunk->first] = true;
    }
    stats.moving_voxels += held.count;
    chunk = held.count == 0U ? moving_.erase(chunk) : std::next(chunk);
  }
}

} // namespace drone_city_nav

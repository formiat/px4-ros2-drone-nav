#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <iterator>
#include <limits>

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
        const OccupancyChunkIndex3D chunk_index =
            ObservedOccupancyGrid3D::chunkIndex(near);
        const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(near);
        MovingChunk& held = moving_[chunk_index];
        held.held.at(bit_index / 64U) |= std::uint64_t{1U} << (bit_index % 64U);
        ++held.count;
        const auto stamp_ms = static_cast<std::int32_t>(std::clamp<std::int64_t>(
            now_ms, 0, std::numeric_limits<std::int32_t>::max()));
        held.since_ms.at(bit_index) = stamp_ms;
        held.last_ms.at(bit_index) = stamp_ms;
        const auto scores = evidence_.find(chunk_index);
        if (scores != evidence_.end()) {
          scores->second.scores.at(bit_index) =
              static_cast<double>(config_.occupied_score);
          scores->second.confirmations.at(bit_index) = 0U;
        }
        ++stats.moving_joined;
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

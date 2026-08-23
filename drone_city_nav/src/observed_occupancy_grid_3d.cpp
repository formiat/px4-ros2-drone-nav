#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validBounds(const GridBounds3D& bounds) noexcept {
  return std::isfinite(bounds.origin_x) && std::isfinite(bounds.origin_y) &&
         std::isfinite(bounds.origin_z) && std::isfinite(bounds.resolution_m) &&
         bounds.resolution_m > 0.0 && bounds.width_cells > 0 &&
         bounds.height_cells > 0 && bounds.depth_cells > 0;
}

[[nodiscard]] bool chunkEmpty(const ObservedOccupancyChunk3D& chunk) noexcept {
  return std::ranges::all_of(chunk.observed,
                             [](const std::uint64_t word) { return word == 0U; });
}

[[nodiscard]] std::size_t popcount(const OccupancyGrid3D::Chunk& words) noexcept {
  std::size_t count{0U};
  for (const std::uint64_t word : words) {
    count += static_cast<std::size_t>(std::popcount(word));
  }
  return count;
}

} // namespace

ObservedOccupancyChunkStorage3D::ObservedOccupancyChunkStorage3D(
    ObservedOccupancyChunk3D chunk)
    : chunk_{std::make_shared<ObservedOccupancyChunk3D>(chunk)} {
}

const ObservedOccupancyChunk3D& ObservedOccupancyChunkStorage3D::get() const noexcept {
  return *chunk_;
}

const ObservedOccupancyChunk3D*
ObservedOccupancyChunkStorage3D::operator->() const noexcept {
  return chunk_.get();
}

ObservedOccupancyChunk3D& ObservedOccupancyChunkStorage3D::mutableChunk() {
  if (!chunk_.unique()) {
    chunk_ = std::make_shared<ObservedOccupancyChunk3D>(*chunk_);
  }
  return *chunk_;
}

ObservedOccupancyGrid3D::ObservedOccupancyGrid3D(const GridBounds3D& bounds)
    : bounds_{bounds} {
  if (!validBounds(bounds_)) {
    throw std::invalid_argument{"invalid ObservedOccupancyGrid3D bounds"};
  }
}

const GridBounds3D& ObservedOccupancyGrid3D::bounds() const noexcept {
  return bounds_;
}

bool ObservedOccupancyGrid3D::contains(const GridIndex3D index) const noexcept {
  return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
         index.x < bounds_.width_cells && index.y < bounds_.height_cells &&
         index.z < bounds_.depth_cells;
}

std::optional<GridIndex3D>
ObservedOccupancyGrid3D::worldToCell(const Point3& point) const noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return std::nullopt;
  }
  const GridIndex3D index{
      static_cast<int>(std::floor((point.x - bounds_.origin_x) / bounds_.resolution_m)),
      static_cast<int>(std::floor((point.y - bounds_.origin_y) / bounds_.resolution_m)),
      static_cast<int>(
          std::floor((point.z - bounds_.origin_z) / bounds_.resolution_m))};
  return contains(index) ? std::optional<GridIndex3D>{index} : std::nullopt;
}

Point3 ObservedOccupancyGrid3D::cellCenter(const GridIndex3D index) const noexcept {
  return Point3{
      bounds_.origin_x + (static_cast<double>(index.x) + 0.5) * bounds_.resolution_m,
      bounds_.origin_y + (static_cast<double>(index.y) + 0.5) * bounds_.resolution_m,
      bounds_.origin_z + (static_cast<double>(index.z) + 0.5) * bounds_.resolution_m};
}

ObservedVoxelState
ObservedOccupancyGrid3D::state(const GridIndex3D index) const noexcept {
  if (!contains(index)) {
    return ObservedVoxelState::kUnknown;
  }
  const auto found = chunks_.find(chunkIndex(index));
  if (found == chunks_.end()) {
    return ObservedVoxelState::kUnknown;
  }
  const std::size_t bit_index = localBitIndex(index);
  if (!bit(found->second->observed, bit_index)) {
    return ObservedVoxelState::kUnknown;
  }
  return bit(found->second->occupied, bit_index) ? ObservedVoxelState::kOccupied
                                                 : ObservedVoxelState::kFree;
}

bool ObservedOccupancyGrid3D::isKnown(const GridIndex3D index) const noexcept {
  return state(index) != ObservedVoxelState::kUnknown;
}

bool ObservedOccupancyGrid3D::isKnownFree(const GridIndex3D index) const noexcept {
  return state(index) == ObservedVoxelState::kFree;
}

bool ObservedOccupancyGrid3D::isOccupied(const GridIndex3D index) const noexcept {
  return state(index) == ObservedVoxelState::kOccupied;
}

std::size_t ObservedOccupancyGrid3D::knownVoxelCount() const noexcept {
  return known_voxels_;
}

std::size_t ObservedOccupancyGrid3D::freeVoxelCount() const noexcept {
  return free_voxels_;
}

std::size_t ObservedOccupancyGrid3D::occupiedVoxelCount() const noexcept {
  return occupied_voxels_;
}

const ObservedOccupancyGrid3D::ChunkMap&
ObservedOccupancyGrid3D::chunks() const noexcept {
  return chunks_;
}

bool ObservedOccupancyGrid3D::setState(const GridIndex3D index,
                                       const ObservedVoxelState state_value) {
  if (!contains(index)) {
    return false;
  }
  const ObservedVoxelState before = state(index);
  if (before == state_value) {
    return false;
  }
  const OccupancyChunkIndex3D chunk_index = chunkIndex(index);
  const std::size_t bit_index = localBitIndex(index);
  if (state_value == ObservedVoxelState::kUnknown) {
    auto found = chunks_.find(chunk_index);
    if (found == chunks_.end()) {
      return false;
    }
    Chunk& chunk = found->second.mutableChunk();
    setBit(chunk.observed, bit_index, false);
    setBit(chunk.occupied, bit_index, false);
    if (chunkEmpty(chunk)) {
      chunks_.erase(found);
    }
  } else {
    Chunk& chunk = mutableChunk(chunk_index);
    setBit(chunk.observed, bit_index, true);
    setBit(chunk.occupied, bit_index, state_value == ObservedVoxelState::kOccupied);
  }
  updateCounts(before, state_value);
  return true;
}

bool ObservedOccupancyGrid3D::replaceChunk(const OccupancyChunkIndex3D index,
                                           const Chunk& chunk) {
  const auto found = chunks_.find(index);
  if (found != chunks_.end() && found->second->observed == chunk.observed &&
      found->second->occupied == chunk.occupied) {
    return false;
  }
  if (found != chunks_.end()) {
    const std::size_t old_known = popcount(found->second->observed);
    const std::size_t old_occupied = popcount(found->second->occupied);
    known_voxels_ -= old_known;
    occupied_voxels_ -= old_occupied;
    free_voxels_ -= old_known - old_occupied;
  }
  if (chunkEmpty(chunk)) {
    chunks_.erase(index);
    return true;
  }
  Chunk normalized = chunk;
  std::ranges::transform(normalized.occupied, normalized.observed,
                         normalized.occupied.begin(), std::bit_and<>{});
  const std::size_t new_known = popcount(normalized.observed);
  const std::size_t new_occupied = popcount(normalized.occupied);
  known_voxels_ += new_known;
  occupied_voxels_ += new_occupied;
  free_voxels_ += new_known - new_occupied;
  chunks_.insert_or_assign(index, ChunkStorage{normalized});
  return true;
}

ObservedOccupancyGrid3D::Chunk&
ObservedOccupancyGrid3D::mutableChunk(const OccupancyChunkIndex3D index) {
  const auto [found, inserted] = chunks_.try_emplace(index, ChunkStorage{Chunk{}});
  static_cast<void>(inserted);
  return found->second.mutableChunk();
}

void ObservedOccupancyGrid3D::clear() {
  chunks_.clear();
  known_voxels_ = 0U;
  free_voxels_ = 0U;
  occupied_voxels_ = 0U;
}

OccupancyGrid3D ObservedOccupancyGrid3D::occupiedSnapshot() const {
  OccupancyGrid3D snapshot{bounds_};
  for (const auto& [chunk_index, storage] : chunks_) {
    const Chunk& chunk = storage.get();
    for (std::size_t bit_index = 0U; bit_index < OccupancyGrid3D::kVoxelsPerChunk;
         ++bit_index) {
      if (!bit(chunk.occupied, bit_index)) {
        continue;
      }
      const int local_x = static_cast<int>(bit_index % kChunkSize);
      const int local_y = static_cast<int>((bit_index / kChunkSize) % kChunkSize);
      const int local_z = static_cast<int>(
          bit_index / static_cast<std::size_t>(kChunkSize * kChunkSize));
      const GridIndex3D index{chunk_index.x * kChunkSize + local_x,
                              chunk_index.y * kChunkSize + local_y,
                              chunk_index.z * kChunkSize + local_z};
      if (contains(index)) {
        snapshot.setOccupied(index);
      }
    }
  }
  return snapshot;
}

ObservedOccupancyGrid3D
ObservedOccupancyGrid3D::crop(const GridBounds3D& crop_bounds) const {
  ObservedOccupancyGrid3D result{crop_bounds};
  for (int z = 0; z < crop_bounds.depth_cells; ++z) {
    for (int y = 0; y < crop_bounds.height_cells; ++y) {
      for (int x = 0; x < crop_bounds.width_cells; ++x) {
        const GridIndex3D target{x, y, z};
        const Point3 center = result.cellCenter(target);
        const std::optional<GridIndex3D> source = worldToCell(center);
        if (source.has_value()) {
          static_cast<void>(result.setState(target, state(*source)));
        }
      }
    }
  }
  return result;
}

OccupancyChunkIndex3D
ObservedOccupancyGrid3D::chunkIndex(const GridIndex3D index) noexcept {
  return OccupancyChunkIndex3D{index.x / kChunkSize, index.y / kChunkSize,
                               index.z / kChunkSize};
}

std::size_t ObservedOccupancyGrid3D::localBitIndex(const GridIndex3D index) noexcept {
  const int local_x = index.x % kChunkSize;
  const int local_y = index.y % kChunkSize;
  const int local_z = index.z % kChunkSize;
  return (static_cast<std::size_t>(local_z) * kChunkSize +
          static_cast<std::size_t>(local_y)) *
             kChunkSize +
         static_cast<std::size_t>(local_x);
}

bool ObservedOccupancyGrid3D::bit(const OccupancyGrid3D::Chunk& words,
                                  const std::size_t index) noexcept {
  const std::size_t word = index / 64U;
  const std::size_t offset = index % 64U;
  return (words.at(word) & (std::uint64_t{1U} << offset)) != 0U;
}

void ObservedOccupancyGrid3D::setBit(OccupancyGrid3D::Chunk& words,
                                     const std::size_t index,
                                     const bool value) noexcept {
  const std::size_t word = index / 64U;
  const std::size_t offset = index % 64U;
  const std::uint64_t mask = std::uint64_t{1U} << offset;
  if (value) {
    words.at(word) |= mask;
  } else {
    words.at(word) &= ~mask;
  }
}

void ObservedOccupancyGrid3D::updateCounts(const ObservedVoxelState before,
                                           const ObservedVoxelState after) noexcept {
  const auto subtract = [this](const ObservedVoxelState state_value) {
    if (state_value != ObservedVoxelState::kUnknown) {
      --known_voxels_;
    }
    if (state_value == ObservedVoxelState::kFree) {
      --free_voxels_;
    } else if (state_value == ObservedVoxelState::kOccupied) {
      --occupied_voxels_;
    }
  };
  const auto add = [this](const ObservedVoxelState state_value) {
    if (state_value != ObservedVoxelState::kUnknown) {
      ++known_voxels_;
    }
    if (state_value == ObservedVoxelState::kFree) {
      ++free_voxels_;
    } else if (state_value == ObservedVoxelState::kOccupied) {
      ++occupied_voxels_;
    }
  };
  subtract(before);
  add(after);
}

} // namespace drone_city_nav

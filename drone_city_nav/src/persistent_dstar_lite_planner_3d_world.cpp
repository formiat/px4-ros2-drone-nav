#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {
namespace {

[[nodiscard]] GridIndex3D rawCellFromChunkBit(const OccupancyChunkIndex3D& chunk,
                                              const std::size_t bit_index) noexcept {
  const std::size_t layer = static_cast<std::size_t>(OccupancyGrid3D::kChunkSize) *
                            static_cast<std::size_t>(OccupancyGrid3D::kChunkSize);
  const auto local_z = static_cast<int>(bit_index / layer);
  const std::size_t within_layer = bit_index % layer;
  const auto local_y = static_cast<int>(
      within_layer / static_cast<std::size_t>(OccupancyGrid3D::kChunkSize));
  const auto local_x = static_cast<int>(
      within_layer % static_cast<std::size_t>(OccupancyGrid3D::kChunkSize));
  return GridIndex3D{
      chunk.x * OccupancyGrid3D::kChunkSize + local_x,
      chunk.y * OccupancyGrid3D::kChunkSize + local_y,
      chunk.z * OccupancyGrid3D::kChunkSize + local_z,
  };
}

} // namespace

void PersistentDStarLitePlanner3DImpl::installWorld(
    const PersistentPlannerWorld3D& world) {
  world_ = world;
  lattice_.installWorld(world_);
}

void PersistentDStarLitePlanner3DImpl::installDepartureEvidence(
    const PersistentPlannerWorld3D& world) {
  world_.proprioceptive_free_space_seed = world.proprioceptive_free_space_seed;
  world_.launch_support_contact = world.launch_support_contact;
  lattice_.installDepartureEvidence(world_);
}

double PersistentDStarLitePlanner3DImpl::anchorDepartureEvidence(const Point3& start) {
  if (!world_.proprioceptive_free_space_seed.has_value()) {
    return -1.0;
  }
  ProprioceptiveFreeSpaceSeed3D& seed = *world_.proprioceptive_free_space_seed;
  const double distance_m = distance3D(seed.position, start);
  if (distance_m <= 0.0) {
    return distance_m;
  }
  seed.position = start;
  lattice_.installDepartureEvidence(world_);
  return distance_m;
}

PersistentPlannerWorldUpdate3D
PersistentDStarLitePlanner3DImpl::updateWorld(const PersistentPlannerWorld3D& world) {
  PersistentPlannerWorldUpdate3D update;
  if (!world.valid() || world.bounds() == nullptr) {
    return update;
  }
  if (!initialized_ && world_.producer_instance_id == 0U) {
    installWorld(world);
    lattice_.configureGridGeometry(*world.bounds());
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }
  if (world.producer_instance_id == world_.producer_instance_id &&
      lattice_.sameGridGeometry(*world.bounds()) && world.revision < world_.revision) {
    // The caller's snapshot lags behind evidence this planner has already
    // absorbed from a newer continuation. Occupied evidence is monotonic in
    // time, so the resident world stays authoritative and the request simply
    // continues on it with its own departure evidence.
    installDepartureEvidence(world);
    update.accepted = true;
    update.occupied_world_unchanged = true;
    update.resident_world_retained = true;
    return update;
  }
  if (world_.producer_instance_id == 0U ||
      world.producer_instance_id != world_.producer_instance_id ||
      !lattice_.sameGridGeometry(*world.bounds()) ||
      (world.revision == world_.revision &&
       world.occupied_fingerprint != world_.occupied_fingerprint)) {
    installWorld(world);
    lattice_.configureGridGeometry(*world.bounds());
    lattice_.resetEdgeEvidence();
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }
  if (world.revision == world_.revision) {
    update.accepted = true;
    update.occupied_world_unchanged = true;
    installWorld(world);
    return update;
  }
  if (world.occupied_fingerprint == world_.occupied_fingerprint) {
    installWorld(world);
    update.accepted = true;
    update.occupied_world_unchanged = true;
    return update;
  }
  if (world.observed_occupancy == nullptr || world_.observed_occupancy == nullptr) {
    installWorld(world);
    lattice_.resetEdgeEvidence();
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }

  // A skipped publication makes the dirty-chunk delta incomplete. The exact
  // change set is still available by comparing the two resident grids chunk
  // by chunk, which is far cheaper than discarding every search label.
  const bool dirty_chunks_complete =
      world.incremental_parent_revision == world_.revision;
  const auto diff_started = std::chrono::steady_clock::now();
  update.changed_cells = changedOccupiedCells(world_, world, dirty_chunks_complete);
  update.diff_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - diff_started)
                       .count();
  update.occupied_cells_removed =
      std::ranges::any_of(update.changed_cells, [&](const GridIndex3D cell) {
        return world_.observed_occupancy->isOccupied(cell) &&
               !world.observed_occupancy->isOccupied(cell);
      });
  if (update.changed_cells.empty() ||
      update.changed_cells.size() > config_.maximum_incremental_changed_voxels) {
    update.changed_cells.clear();
    update.requires_reset = true;
  }
  const auto install_started = std::chrono::steady_clock::now();
  installWorld(world);
  update.install_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - install_started)
                          .count();
  update.accepted = true;
  return update;
}

std::vector<GridIndex3D> PersistentDStarLitePlanner3DImpl::changedOccupiedCells(
    const PersistentPlannerWorld3D& previous, const PersistentPlannerWorld3D& current,
    const bool dirty_chunks_complete) const {
  const ObservedOccupancyGrid3D& before = *previous.observed_occupancy;
  const ObservedOccupancyGrid3D& after = *current.observed_occupancy;
  std::vector<OccupancyChunkIndex3D> chunks =
      dirty_chunks_complete ? current.dirty_chunks
                            : std::vector<OccupancyChunkIndex3D>{};
  if (chunks.empty()) {
    chunks.reserve(before.chunks().size() + after.chunks().size());
    for (const auto& [index, storage] : before.chunks()) {
      static_cast<void>(storage);
      chunks.push_back(index);
    }
    for (const auto& [index, storage] : after.chunks()) {
      static_cast<void>(storage);
      chunks.push_back(index);
    }
  }
  std::ranges::sort(chunks, {}, [](const OccupancyChunkIndex3D& index) {
    return std::tuple{index.z, index.y, index.x};
  });
  chunks.erase(std::unique(chunks.begin(), chunks.end()), chunks.end());

  std::vector<GridIndex3D> changed;
  for (const OccupancyChunkIndex3D& index : chunks) {
    const ObservedOccupancyChunk3D* const before_chunk = before.findChunk(index);
    const ObservedOccupancyChunk3D* const after_chunk = after.findChunk(index);
    if (before_chunk == after_chunk) {
      continue;
    }
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      const std::uint64_t before_word =
          before_chunk != nullptr ? before_chunk->occupied.at(word_index) : 0U;
      const std::uint64_t after_word =
          after_chunk != nullptr ? after_chunk->occupied.at(word_index) : 0U;
      std::uint64_t difference = before_word ^ after_word;
      while (difference != 0U) {
        const int bit_offset = std::countr_zero(difference);
        const std::size_t bit_index =
            word_index * 64U + static_cast<std::size_t>(bit_offset);
        const GridIndex3D cell = rawCellFromChunkBit(index, bit_index);
        if (before.contains(cell)) {
          changed.push_back(cell);
          if (changed.size() > config_.maximum_incremental_changed_voxels) {
            return changed;
          }
        }
        difference &= difference - 1U;
      }
    }
  }
  return changed;
}

} // namespace drone_city_nav::detail

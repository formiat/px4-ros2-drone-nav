#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
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

bool ObstacleMemory3D::oldOrdinary(const GridIndex3D cell) const noexcept {
  return grid_.contains(cell) && grid_.isOccupied(cell) && !heldMoving(cell) &&
         (grid_.isUnobservable(cell) || confirmationsAt(cell) > kYoungConfirmations);
}

bool ObstacleMemory3D::oldSurfaceVoxel(const GridIndex3D cell) const noexcept {
  // A surface has extent: an old occupancy with an old neighbour. A lone old
  // voxel among a body's is the body's own, confirmed before the body was
  // seen to move (r3045: two such voxels dropped the whole column from the
  // clustering for three seconds of the approach).
  if (!oldOrdinary(cell)) {
    return false;
  }
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx != 0 || dy != 0 || dz != 0) &&
            oldOrdinary({cell.x + dx, cell.y + dy, cell.z + dz})) {
          return true;
        }
      }
    }
  }
  return false;
}

bool ObstacleMemory3D::besideOldSurface(const GridIndex3D cell) const noexcept {
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx != 0 || dy != 0 || dz != 0) &&
            oldSurfaceVoxel({cell.x + dx, cell.y + dy, cell.z + dz})) {
          return true;
        }
      }
    }
  }
  return false;
}

bool ObstacleMemory3D::inAColumn(const GridIndex3D cell) const noexcept {
  // A body is a column: its voxels stand on one another. A floor's or a
  // ceiling's voxels lie beside one another, and a body's feet joined the
  // young floor of the corridor into one cluster wider than any body (r3057:
  // the body's held column went unclustered for the whole approach until the
  // floor under it had been confirmed old).
  const GridIndex3D above{cell.x, cell.y, cell.z + 1};
  const GridIndex3D below{cell.x, cell.y, cell.z - 1};
  return (grid_.contains(above) && grid_.isOccupied(above)) ||
         (grid_.contains(below) && grid_.isOccupied(below));
}

bool ObstacleMemory3D::besideSurfaceOrItsSkin(const GridIndex3D cell) const noexcept {
  // A surface's skin is the young occupancy beside it, the range's noise on
  // its near side at close range (r3053: the body turning a quarter metre
  // from the corridor's wall was one cluster with that young skin, and no
  // body until it walked clear).
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const GridIndex3D near{cell.x + dx, cell.y + dy, cell.z + dz};
        if (oldSurfaceVoxel(near)) {
          return true;
        }
        if (grid_.contains(near) && grid_.isOccupied(near) && !heldMoving(near) &&
            !oldOrdinary(near) && besideOldSurface(near)) {
          return true;
        }
      }
    }
  }
  return false;
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
  // Inside a followed body's envelope (a cell beyond its reach) a voxel beside
  // one the layer holds joins it, free, unknown or a young occupancy: the
  // cluster that shifts. Beyond every envelope only a free voxel beside a held
  // one does (r3025: a held voxel's unknown neighbours taken whatever stood
  // round them spread the layer over every fresh surface, 28 thousand voxels).
  // Away from the layer, a hit where free space had been observed is a body's
  // appearance, a hit in unknown space or on a young occupancy is nothing
  // yet, and a surface voxel's neighbour takes a return now and then from the
  // range's noise: ordinary, as the surface is, unless the body walks there
  // (r3013: a body turning a quarter metre from a wall left its wall-side
  // cells).
  const Point3 centre = grid_.cellCenter(cell);
  const double resolution_m = grid_.bounds().resolution_m;
  bool inside_envelope = false;
  for (const MovingBody3D& body : enveloped_bodies_) {
    if (std::hypot(centre.x - body.centre_map.x, centre.y - body.centre_map.y) <=
            body.radius_m + resolution_m &&
        centre.z >= body.bottom_m - resolution_m &&
        centre.z <= body.top_m + resolution_m) {
      inside_envelope = true;
      break;
    }
  }
  const bool free = !occupied && grid_.state(cell) == ObservedVoxelState::kFree;
  bool beside_ordinary{false};
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const GridIndex3D near{cell.x + dx, cell.y + dy, cell.z + dz};
        if ((dx == 0 && dy == 0 && dz == 0) || !grid_.contains(near) ||
            !grid_.isOccupied(near)) {
          continue;
        }
        if (heldMoving(near) && (free || inside_envelope)) {
          return true;
        }
        beside_ordinary = beside_ordinary || !heldMoving(near);
      }
    }
  }
  return free && !beside_ordinary;
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
  // A voxel beside an old surface is that surface's skin (the range's noise
  // on its near side) or a body's feet or wall-side cells: not a body's
  // voxel to cluster, or a body a quarter metre from a wall is one cluster
  // with the wall's skin, hugging it, and goes unread (r3041: the crossing
  // body at the corridor's wall was first followed after the vehicle had
  // passed it). What remains of the body, its column clear of the wall, is
  // enough.
  std::vector<GridIndex3D> cells;
  std::vector<GridIndex3D> warm_cells;
  for (const auto& [chunk_index, chunk] : moving_) {
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      std::uint64_t remaining = chunk.held.at(word_index);
      while (remaining != 0U) {
        const auto bit_offset = static_cast<std::size_t>(std::countr_zero(remaining));
        remaining &= remaining - 1U;
        const GridIndex3D cell =
            cellFromChunkBit(chunk_index, word_index * 64U + bit_offset);
        if (!besideSurfaceOrItsSkin(cell) && inAColumn(cell)) {
          cells.push_back(cell);
        }
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
        if ((chunk_evidence.warm.at(word_index) & (std::uint64_t{1U} << bit_offset)) !=
            0U) {
          warm_cells.push_back(cell);
        }
        if (grid_.isOccupied(cell) && !grid_.isUnobservable(cell) &&
            !heldMoving(cell) && confirmationsAt(cell) <= kYoungConfirmations &&
            !besideSurfaceOrItsSkin(cell) && inAColumn(cell)) {
          cells.push_back(cell);
        }
      }
    }
  }
  tracker_.update(cells, warm_cells, grid_.bounds(), now_ms);
  // The bodies the planner read on the scan before, and how far each may
  // have walked from where it was last seen.
  const std::vector<MovingBody3D> read_before = std::move(followed_bodies_);
  const double elapsed_s =
      followed_ms_ > 0 ? static_cast<double>(now_ms - followed_ms_) * 1.0e-3 : 0.0;
  followed_ms_ = now_ms;
  const auto reachM = [&](const MovingBody3D& body) {
    return body.radius_m +
           std::max(kWarmBodyWalkMps,
                    std::hypot(body.velocity_map_mps.x, body.velocity_map_mps.y)) *
               (body.unseen_s + elapsed_s) +
           config_.moving_match_m;
  };
  const auto within = [](const MovingBody3D& body, const Point3& place,
                         const double reach_m) {
    return std::hypot(body.centre_map.x - place.x, body.centre_map.y - place.y) <=
           reach_m;
  };
  followed_bodies_.clear();
  enveloped_bodies_.clear();
  for (std::size_t index = 0U; index < tracker_.bodies().size(); ++index) {
    const MovingBody3D& body = tracker_.bodies()[index];
    std::size_t held_cells = 0U;
    std::size_t hugging_cells = 0U;
    for (const GridIndex3D cell : tracker_.cells(index)) {
      held_cells += heldMoving(cell) ? 1U : 0U;
      hugging_cells += besideOldSurface(cell) ? 1U : 0U;
    }
    // A cluster that hugs an old surface along more than half its voxels is
    // that surface's skin, the range's noise on its near side, which walks
    // along the wall with the vehicle (r3035: walls 14 to 18 m off read as
    // bodies at 0.5 to 3 m/s); a body stands clear of surfaces but for its
    // feet.
    if (!body.warm && 2U * hugging_cells > tracker_.cells(index).size()) {
      continue;
    }
    if (body.moving()) {
      // A cluster seen to move is a body: its voxels, the ones it had the
      // scan before and every one it has had since it was first seen are the
      // layer's from now.
      for (const std::span<const GridIndex3D> group :
           {tracker_.cells(index), tracker_.previousCells(index),
            tracker_.cellsSinceFirst(index)}) {
        for (const GridIndex3D cell : group) {
          if (grid_.isOccupied(cell) && !grid_.isUnobservable(cell) &&
              !heldMoving(cell) && confirmationsAt(cell) <= kYoungConfirmations) {
            holdMoving(cell, now_ms, stats);
          }
          // A voxel beside the body's that is no surface's is the body's own:
          // a lone old one confirmed before the body was seen to move, or a
          // young one left out of the cluster for standing beside a wall's
          // skin (the body's wall-side column).
          for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dx = -1; dx <= 1; ++dx) {
                const GridIndex3D near{cell.x + dx, cell.y + dy, cell.z + dz};
                if ((dx != 0 || dy != 0 || dz != 0) && grid_.contains(near) &&
                    grid_.isOccupied(near) && !grid_.isUnobservable(near) &&
                    !heldMoving(near) && !oldSurfaceVoxel(near)) {
                  holdMoving(near, now_ms, stats);
                }
              }
            }
          }
        }
      }
      held_cells = tracker_.cells(index).size();
    }
    // The planner reads a warm body's cluster (K64), at once and at any range
    // it clusters at: an oncoming body the vehicle closed on at 5 to 7 m/s
    // was first read by its step after the vehicle had passed over it (r3465,
    // r3466), the core having sighted it from 28 m and more. A cluster where a
    // body read before may have walked to is that body, whatever the core
    // says of it this scan (under the vehicle it leaves the core's field). A
    // cluster without the class is no body to the planner: read by its step,
    // a wall's young skin was handed over 7 to 20 times an ordinary flight
    // (r3441 to r3450), and with a metre walked from its first place 6 to 9
    // times still (r3472 to r3477). Its voxels are the layer's all the same.
    const bool read =
        body.warm || std::ranges::any_of(read_before, [&](const MovingBody3D& before) {
          return within(before, body.centre_map, reachM(before));
        });
    if (read) {
      followed_bodies_.push_back(body);
      followed_bodies_.back().warm = true;
    }
    // The layer grows in the envelope of a body the planner reads, and of a
    // cluster seen to move that it holds by half at least within 12 m: a
    // young wall patch with held voxels in it is a wall (r3031, r3033), and
    // beyond 12 m a body is a few scattered voxels, as a far wall's skin is
    // (r3037).
    constexpr double kEnvelopesWithinM{12.0};
    if (read || (body.moving() && 2U * held_cells >= tracker_.cells(index).size() &&
                 std::hypot(body.centre_map.x - origin.x,
                            body.centre_map.y - origin.y) <= kEnvelopesWithinM)) {
      enveloped_bodies_.push_back(body);
    }
  }
  // A body read before whose cluster this scan does not show is read on while
  // the layer holds a voxel where it may be, for the layer's life at most: at
  // the moments that counted the cluster was not there (r3475, r3476: the
  // body 0.6 to 3 m from the vehicle, the law released, the vehicle flown
  // over it at 2.6 m/s).
  const std::size_t seen = followed_bodies_.size();
  for (const MovingBody3D& read : read_before) {
    MovingBody3D unseen = read;
    unseen.unseen_s += elapsed_s;
    const double reach_m = reachM(read) - config_.moving_match_m;
    if (unseen.unseen_s > config_.moving_life_s ||
        std::ranges::any_of(std::span{followed_bodies_}.first(seen),
                            [&](const MovingBody3D& body) {
                              return within(read, body.centre_map, reachM(read));
                            }) ||
        std::ranges::none_of(cells, [&](const GridIndex3D cell) {
          return heldMoving(cell) && within(read, grid_.cellCenter(cell), reach_m);
        })) {
      continue;
    }
    followed_bodies_.push_back(unseen);
  }
  stats.followed_bodies = followed_bodies_.size();
  if (!(config_.moving_sweep_s > 0.0)) {
    return;
  }
  // The vehicle's own place is never swept: a body walking up to a hovering
  // vehicle leaves it a way out (the exit the law owes, K66).
  constexpr double kKeepsClearOfVehicleM{1.0};
  for (const MovingBody3D& body : followed_bodies_) {
    if (body.unseen_s > 0.0) {
      continue;
    }
    for (const GridIndex3D cell : sweptCells3D(
             body, config_.moving_sweep_s, config_.moving_reach_m, grid_.bounds())) {
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

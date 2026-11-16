#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
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
  // A column's foot: a young occupancy under a voxel the layer holds, over an
  // old floor. On a body's first scan in a place its feet stand beside the
  // floor's voxels and are ordinary; never taken, the row a quarter metre
  // over the floor stayed along the whole walk of a body seen from over 12 m
  // (r3502: 56 cell-snapshots up to 11.9 s old, every one at that height).
  const GridIndex3D over{cell.x, cell.y, cell.z + 1};
  if (occupied && grid_.contains(over) && heldMoving(over) &&
      (oldSurfaceVoxel(GridIndex3D{cell.x, cell.y, cell.z - 1}) ||
       oldSurfaceVoxel(GridIndex3D{cell.x, cell.y, cell.z - 2}))) {
    return true;
  }
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
  // Of the layer's voxels those a return hit within half a second are a
  // body's own surface now (a scan comes every 0.1 to 0.3 s): an oncoming
  // body hides its trail behind itself, the trail's voxels live out the
  // layer's 3 s unseen, and clustered with them the body was a streak 3.6 m
  // long, wider than a body and dropped as a surface, while the bodies read
  // stood 2.4 to 5.4 m from the true one (r3481).
  constexpr std::int64_t kFreshMs{500};
  std::vector<GridIndex3D> cells;
  std::vector<GridIndex3D> warm_cells;
  for (const auto& [chunk_index, chunk] : moving_) {
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      std::uint64_t remaining = chunk.held.at(word_index);
      while (remaining != 0U) {
        const auto bit_offset = static_cast<std::size_t>(std::countr_zero(remaining));
        remaining &= remaining - 1U;
        const std::size_t bit_index = word_index * 64U + bit_offset;
        const GridIndex3D cell = cellFromChunkBit(chunk_index, bit_index);
        if (now_ms - chunk.last_ms.at(bit_index) > kFreshMs) {
          continue;
        }
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
            !heldMoving(cell) && confirmationsAt(cell) <= kYoungConfirmations) {
          // A body a quarter metre from a wall is young occupancies the layer
          // does not take (they stand beside the wall's): what a body read
          // before is read on by, as by the layer's voxels (r3496: unread at
          // the wall, 2.1 m from the vehicle, and passed at 2.7 m/s).
          if (!besideSurfaceOrItsSkin(cell) && inAColumn(cell)) {
            cells.push_back(cell);
          }
        }
      }
    }
  }
  tracker_.update(cells, warm_cells, grid_.bounds(), now_ms);
  stats.warm_voxels = warm_cells.size();
  stats.body_clusters = tracker_.bodies().size();
  // The bodies the planner read on the scan before, and how far each may
  // have walked from where it was last seen.
  const std::vector<MovingBody3D> read_before = std::move(followed_bodies_);
  const std::vector<std::int64_t> warm_before = std::move(followed_warm_ms_);
  const std::vector<Point3> marked_before = std::move(followed_marked_at_);
  // A body the core has not marked for the stand's time is an occupancy like
  // any (K65). The layer's life was too short a hold: a body pacing 2 to 4 m
  // before a waiting vehicle walks out of the core's field at either end of
  // its walk for seconds (r3496).
  const auto stand_ms = static_cast<std::int64_t>(1000.0 * config_.moving_stand_s);
  const std::int64_t followed_ms_before = followed_ms_;
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
  // A body's place is its own height too: by the plan alone a patch of the
  // ceiling 3 m over a body's head took its class and its place, the body
  // went unread and the vehicle came 0.19 m over its head (r3547).
  const auto within = [&](const MovingBody3D& body, const Point3& place,
                          const double reach_m) {
    return std::hypot(body.centre_map.x - place.x, body.centre_map.y - place.y) <=
               reach_m &&
           place.z >= body.bottom_m - grid_.bounds().resolution_m &&
           place.z <= body.top_m + grid_.bounds().resolution_m;
  };
  // A body that walks is one the core need not mark again: half a metre from
  // where it stood at its last mark is a mark of its own. Flown along with
  // and over, a body stays out of the core's field for as long as it walks
  // (r3503: unread 6 s under the vehicle, which sank to 0.33 m over its
  // head); a patch of wall that took the class by its place goes nowhere.
  // Its walking: a step from where it stood the scan before no longer than
  // a walk makes, to a place half a metre from its mark. A body read before is
  // one cluster's to continue, the nearest: with every cluster within reach
  // taking the class and every one a metre from its parent's mark "walking",
  // the class ran along the corridor's walls, 23 to 26 bodies read at once
  // and the vehicle held 101 and 128 s (r3511, r3512: 1.60 and 1.34 m/s). A
  // longer step is another thing taking the body's place: it keeps the mark's
  // age and stands from there.
  constexpr double kWalkedM{0.5};
  const auto apartM = [](const Point3& from, const Point3& to) {
    return std::hypot(to.x - from.x, to.y - from.y);
  };
  const auto marks = [&](const MovingBody3D& before, const Point3& place,
                         std::int64_t& warm_ms, Point3& marked_at) {
    if (apartM(before.centre_map, place) >
        1.5 * kWarmBodyWalkMps * (before.unseen_s + elapsed_s) +
            2.0 * grid_.bounds().resolution_m) {
      marked_at = place;
    } else if (apartM(marked_at, place) >= kWalkedM) {
      warm_ms = now_ms;
      marked_at = place;
    }
  };

  // Where the core's marks of this scan stand within a read body's reach is
  // the body, whatever the clusters make of it: walking at the stereo set it
  // trails the layer's voxels of the last half second and is no cluster of a
  // body's size, the nearest cluster in its reach was four voxels of the
  // floor's young returns, the body was held there and the true one walked on
  // unread under the vehicle, 19 to 26 marked voxels in every scan (r3563).
  // The marks are the body's height too, to a body's tallest: first marked
  // by its legs alone, a body was 0.75 m tall to the law for the rest of the
  // approach, and the vehicle 0.5 m over its head was 1.4 m over it (r3580).
  struct Marked {
    Point3 place{};
    double bottom_m{0.0};
    double top_m{0.0};
  };

  const double half_cell_m = 0.5 * grid_.bounds().resolution_m;
  std::vector<std::optional<Marked>> sighted(read_before.size());
  for (std::size_t before = 0U; before < read_before.size(); ++before) {
    const MovingBody3D& read = read_before[before];
    const double reach_m = reachM(read);
    Point3 sum{};
    std::size_t count = 0U;
    double bottom_m = read.bottom_m;
    double top_m = read.top_m;
    for (const GridIndex3D cell : warm_cells) {
      const Point3 centre = grid_.cellCenter(cell);
      if (apartM(read.centre_map, centre) > reach_m ||
          centre.z + half_cell_m > read.bottom_m + kTallestBodyM ||
          centre.z - half_cell_m < read.top_m - kTallestBodyM) {
        continue;
      }
      sum.x += centre.x;
      sum.y += centre.y;
      ++count;
      bottom_m = std::min(bottom_m, centre.z - half_cell_m);
      top_m = std::max(top_m, centre.z + half_cell_m);
    }
    if (count > 0U) {
      sighted[before] =
          Marked{.place = Point3{sum.x / static_cast<double>(count),
                                 sum.y / static_cast<double>(count), read.centre_map.z},
                 .bottom_m = bottom_m,
                 .top_m = top_m};
    }
  }
  std::vector<std::size_t> continues(tracker_.bodies().size(), read_before.size());
  for (std::size_t before = 0U; before < read_before.size(); ++before) {
    if (sighted[before].has_value()) {
      continue;
    }
    std::size_t nearest = continues.size();
    // A cluster carries no class: it is the body only where the body may have
    // walked to since it was shown. With the tracker's match added, a cold
    // cluster of ten voxels by the floor 1.9 m beyond a body 0.4 s unseen
    // took its place and its height (1.6 m for 3.0), the body walked on
    // unread, and the vehicle came back over it 0.69 m over its head (r3665).
    double nearest_m = reachM(read_before[before]) - config_.moving_match_m;
    for (std::size_t index = 0U; index < continues.size(); ++index) {
      const double apart_m =
          apartM(read_before[before].centre_map, tracker_.bodies()[index].centre_map);
      if (continues[index] == read_before.size() && apart_m <= nearest_m &&
          within(read_before[before], tracker_.bodies()[index].centre_map, apart_m)) {
        nearest_m = apart_m;
        nearest = index;
      }
    }
    if (nearest < continues.size()) {
      continues[nearest] = before;
    }
  }
  followed_bodies_.clear();
  followed_warm_ms_.clear();
  followed_marked_at_.clear();
  enveloped_bodies_.clear();
  stepping_clusters_.clear();
  constexpr double kEnvelopesWithinM{12.0};
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
    // The planner reads a warm body's cluster (K64), at once and at any range
    // it clusters at: an oncoming body the vehicle closed on at 5 to 7 m/s
    // was first read by its step after the vehicle had passed over it (r3465,
    // r3466), the core having sighted it from 28 m and more. A cluster where a
    // body read before may have walked to is that body while the core's last
    // mark on it is no older than the layer's life (under the vehicle a body
    // leaves the core's field; a patch of wall beside its walk, which took
    // the class by its place, kept it 16 s, r3479). A cluster without the
    // class is no body to the planner: read by its step, a wall's young skin
    // was handed over 7 to 20 times an ordinary flight (r3441 to r3450), and
    // with a metre walked from its first place 6 to 9 times still (r3472 to
    // r3477). Its voxels are the layer's all the same.
    std::int64_t warm_ms = body.warm ? now_ms : 0;
    Point3 marked_at = body.centre_map;
    if (!body.warm && continues[index] < read_before.size()) {
      warm_ms = warm_before[continues[index]];
      marked_at = marked_before[continues[index]];
      marks(read_before[continues[index]], body.centre_map, warm_ms, marked_at);
    }
    const bool read = warm_ms > 0 && now_ms - warm_ms <= stand_ms;
    // Seen to move once, it is a body for as long as it is tracked: while its
    // speed read nought at the turn of its walk a quarter metre from the
    // corridor's wall, the hits on its wall-side column were ordinary ones
    // beside the wall, and the column stood there for as long as the vehicle
    // looked elsewhere, 12 and 107 s (r3646).
    if (body.moved || read) {
      // A cluster seen to move, or read as a body, is a body: its voxels, the
      // ones it had the scan before and every one it has had since it was
      // first seen are the layer's from now (a body read by its class at the
      // end of its walk a quarter metre from a wall left its wall-side cells
      // an ordinary occupancy there, 3.6 s old, r3511).
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
    if (read) {
      followed_bodies_.push_back(body);
      followed_bodies_.back().warm = true;
      followed_warm_ms_.push_back(warm_ms);
      followed_marked_at_.push_back(marked_at);
    } else if (body.moving()) {
      stepping_clusters_.push_back(body);
    }
    // The layer grows in the envelope of a body the planner reads, and of a
    // cluster seen to move that it holds by half at least within 12 m: a
    // young wall patch with held voxels in it is a wall (r3031, r3033), and
    // beyond 12 m a body is a few scattered voxels, as a far wall's skin is
    // (r3037).
    if (read || (body.moved && 2U * held_cells >= tracker_.cells(index).size() &&
                 std::hypot(body.centre_map.x - origin.x,
                            body.centre_map.y - origin.y) <= kEnvelopesWithinM)) {
      enveloped_bodies_.push_back(body);
    }
  }
  // A body read before whose cluster this scan does not show is read on while
  // the layer holds a voxel, or the scan hit a young occupancy, where it may
  // be, the core's last mark on it no older than the stand's time: at the
  // moments that counted the cluster was
  // not there (r3475, r3476: the body 0.6 to 3 m from the vehicle, the law
  // released, the vehicle flown over it at 2.6 m/s).
  // Seen from over its head a body is a few voxels a cell or two tall, no
  // cluster of a body's height: it is read where the voxels a scan showed
  // within its reach stand (the layer's of the last half second and the
  // scan's young hits), and only with none of them is it unseen.
  // A body no scan shows is taken off by a look at where it stood, not by a
  // clock: while no scan has observed a cell at every level of the column it
  // was last seen in, the vehicle within the envelopes' range of it, it may
  // stand there still. Hovering 4 m up and 3.3 m before a pacing body, the
  // stereo set sees neither its legs nor its class (the pair's and the
  // core's fields pass over it); the stand's time ran out, the body was no
  // body, and the vehicle flew down at it at 2.1 m/s, to sight it 2.7 m off
  // (r3582, -0.19 m).
  const auto blind = [&](const MovingBody3D& body) {
    // Only while the vehicle stands within a walk of where it stood, where
    // the sensors look over it: farther off they can look, and a thing the
    // core's marks put 5 to 10 m off where no body was stood read for 200 s,
    // the vehicle held before it (r3649).
    if (std::hypot(body.centre_map.x - origin.x, body.centre_map.y - origin.y) >
        body.radius_m + config_.moving_reach_m +
            kWarmBodyWalkMps * config_.moving_sweep_s + 1.0) {
      return false;
    }
    MovingBody3D column = body;
    column.velocity_map_mps = Vec3{};
    column.radius_m = std::max(column.radius_m, grid_.bounds().resolution_m);
    const std::vector<GridIndex3D> stood =
        sweptCells3D(column, 1.0, 0.0, grid_.bounds());
    if (stood.empty()) {
      return false;
    }
    const auto [low, high] = std::ranges::minmax(stood, {}, &GridIndex3D::z);
    std::vector<bool> looked(static_cast<std::size_t>(high.z - low.z + 1), false);
    for (const GridIndex3D cell : stood) {
      const auto chunk = scan_evidence.find(ObservedOccupancyGrid3D::chunkIndex(cell));
      if (chunk == scan_evidence.end()) {
        continue;
      }
      const std::size_t bit = ObservedOccupancyGrid3D::localBitIndex(cell);
      if ((chunk->second.observed.at(bit / 64U) & (std::uint64_t{1U} << (bit % 64U))) !=
          0U) {
        looked[static_cast<std::size_t>(cell.z - low.z)] = true;
      }
    }
    return !std::ranges::all_of(looked, [](const bool level) { return level; });
  };
  // A body read by the core's marks alone, cluster or none, takes what a
  // cluster's body does: the young occupancies in its column and a cell
  // round it are the layer's, and the layer grows in its envelope. Without
  // it the column a body turned in a quarter metre from the corridor's wall
  // stood on as an ordinary occupancy, its hits there being beside the wall's
  // (r3599: both ends of the crossing body's walk, six levels of two cells
  // each, for as long as the vehicle looked elsewhere).
  const auto takesIn = [&](const MovingBody3D& body) {
    MovingBody3D column = body;
    column.velocity_map_mps = Vec3{};
    column.radius_m += grid_.bounds().resolution_m;
    for (const GridIndex3D cell : sweptCells3D(column, 1.0, 0.0, grid_.bounds())) {
      if (grid_.isOccupied(cell) && !grid_.isUnobservable(cell) && !heldMoving(cell) &&
          confirmationsAt(cell) <= kYoungConfirmations) {
        holdMoving(cell, now_ms, stats);
      }
    }
    enveloped_bodies_.push_back(body);
  };
  const std::size_t seen = followed_bodies_.size();
  for (std::size_t before = 0U; before < read_before.size(); ++before) {
    const MovingBody3D& read = read_before[before];
    if (now_ms - warm_before[before] > stand_ms ||
        std::ranges::any_of(std::span{followed_bodies_}.first(seen),
                            [&](const MovingBody3D& body) {
                              return within(read, body.centre_map, reachM(read));
                            })) {
      continue;
    }
    MovingBody3D unshown = read;
    std::int64_t warm_ms = warm_before[before];
    Point3 marked_at = marked_before[before];
    if (sighted[before].has_value()) {
      // Marked by the core this scan: there it stands, its walk not measured.
      unshown.centre_map.x = sighted[before]->place.x;
      unshown.centre_map.y = sighted[before]->place.y;
      unshown.bottom_m = sighted[before]->bottom_m;
      unshown.top_m = sighted[before]->top_m;
      unshown.velocity_map_mps = Vec3{};
      unshown.unseen_s = 0.0;
      warm_ms = now_ms;
      marked_at = unshown.centre_map;
      takesIn(unshown);
    } else {
      // Unseen, wherever it has walked to: for the stand's time from its last
      // mark, whatever the memory holds where it stood. Dropped with the last
      // of its voxels, a body the vehicle came over (4.1 m up, 1.2 to 2 m
      // aside: under the pair's field and between the ring's) was no body
      // 1.7 s later, and the vehicle came down to 0.8 m over its head at
      // 1.8 m/s (r3573). Placed where a scan showed voxels within its reach,
      // it stood by the floor's young returns while the body walked on: 3 m
      // from the true one when that passed the vehicle at 0.74 m (r3580; so
      // r3553 and r3563).
      unshown.unseen_s = std::min(unshown.unseen_s + elapsed_s, config_.moving_stand_s);
      if (blind(read)) {
        warm_ms += now_ms - (followed_ms_before > 0 ? followed_ms_before : now_ms);
      }
    }
    followed_bodies_.push_back(unshown);
    followed_warm_ms_.push_back(warm_ms);
    followed_marked_at_.push_back(marked_at);
  }

  // The core's marks that stand by no body read are a body of their own, at
  // once, cluster or none: at the pair's range a body's returns scatter along
  // the ray and with the layer's voxels about them make no cluster of a
  // body's size (r3568: 17 marked voxels a scan from 5.3 m, the body unread
  // down to 1.5 m and passed at 1.0 m at 1.4 m/s). A mark beside an old
  // surface is the surface behind the body.
  struct Sighting {
    double sum_x{0.0};
    double sum_y{0.0};
    double low_x{0.0};
    double high_x{0.0};
    double low_y{0.0};
    double high_y{0.0};
    double low_z{0.0};
    double high_z{0.0};
    std::size_t count{0U};
  };

  std::vector<Sighting> sightings;
  for (const GridIndex3D cell : warm_cells) {
    const Point3 centre = grid_.cellCenter(cell);
    if (besideSurfaceOrItsSkin(cell) ||
        std::ranges::any_of(followed_bodies_, [&](const MovingBody3D& body) {
          return apartM(body.centre_map, centre) <=
                 body.radius_m + config_.moving_match_m;
        })) {
      continue;
    }
    const auto group = std::ranges::find_if(sightings, [&](const Sighting& sighting) {
      const auto count = static_cast<double>(sighting.count);
      return std::hypot(sighting.sum_x / count - centre.x,
                        sighting.sum_y / count - centre.y) <= config_.moving_match_m;
    });
    if (group == sightings.end()) {
      sightings.push_back(Sighting{.sum_x = centre.x,
                                   .sum_y = centre.y,
                                   .low_x = centre.x,
                                   .high_x = centre.x,
                                   .low_y = centre.y,
                                   .high_y = centre.y,
                                   .low_z = centre.z,
                                   .high_z = centre.z,
                                   .count = 1U});
      continue;
    }
    group->sum_x += centre.x;
    group->sum_y += centre.y;
    group->low_x = std::min(group->low_x, centre.x);
    group->high_x = std::max(group->high_x, centre.x);
    group->low_y = std::min(group->low_y, centre.y);
    group->high_y = std::max(group->high_y, centre.y);
    group->low_z = std::min(group->low_z, centre.z);
    group->high_z = std::max(group->high_z, centre.z);
    ++group->count;
  }
  for (const Sighting& sighting : sightings) {
    if (sighting.count < config_.moving_least_voxels) {
      continue;
    }
    const auto count = static_cast<double>(sighting.count);
    MovingBody3D body;
    body.centre_map = Point3{sighting.sum_x / count, sighting.sum_y / count,
                             0.5 * (sighting.low_z + sighting.high_z)};
    body.radius_m = 0.5 * std::hypot(sighting.high_x - sighting.low_x,
                                     sighting.high_y - sighting.low_y) +
                    half_cell_m;
    body.bottom_m = sighting.low_z - half_cell_m;
    body.top_m = sighting.high_z + half_cell_m;
    body.voxels = sighting.count;
    body.warm = true;
    takesIn(body);
    followed_bodies_.push_back(body);
    followed_warm_ms_.push_back(now_ms);
    followed_marked_at_.push_back(body.centre_map);
  }
  stats.followed_bodies = followed_bodies_.size();
  if (!(config_.moving_sweep_s > 0.0)) {
    return;
  }
  // The vehicle's own place is never swept: a body walking up to a hovering
  // vehicle leaves it a way out (the exit the law owes, K66).
  constexpr double kKeepsClearOfVehicleM{1.0};
  // The way out of a vehicle at the place's levels: what lies behind a plane
  // this far before the vehicle as the body sees it. It is the leaning hull
  // (0.65 m), half a voxel, the camera's 0.3 m before the vehicle's centre and
  // the tracking tube's error (0.49 m). With the plane through the vehicle,
  // and none for a measured walk's way, the place of a body walking at the
  // vehicle stood 0.7 m from its centre: the flown way back "collided", every
  // candidate back failed its first segment, and the vehicle held at rest
  // until the body walked into it (r3663: contact; r3664: 0.33 m).
  constexpr double kWayOutM{1.5};
  const auto sweep = [&](const MovingBody3D& body) {
    const double away_x = origin.x - body.centre_map.x;
    const double away_y = origin.y - body.centre_map.y;
    const double range_m = std::hypot(away_x, away_y);
    const bool owed_way_out =
        range_m > kWayOutM && origin.z <= sweptTopM(body, config_.moving_reach_m,
                                                    grid_.bounds().resolution_m);
    for (const GridIndex3D cell : sweptCells3D(
             body, config_.moving_sweep_s, config_.moving_reach_m, grid_.bounds())) {
      const Point3 centre = grid_.cellCenter(cell);
      if (grid_.isOccupied(cell) ||
          distance3D(centre, origin) <= kKeepsClearOfVehicleM ||
          (owed_way_out &&
           (centre.x - origin.x) * away_x + (centre.y - origin.y) * away_y >
               -kWayOutM * range_m)) {
        continue;
      }
      const ObservedVoxelState before = grid_.state(cell);
      static_cast<void>(grid_.setState(cell, ObservedVoxelState::kOccupied));
      static_cast<void>(grid_.setUnobservable(cell, true));
      dirty_chunks_[ObservedOccupancyGrid3D::chunkIndex(cell)] = true;
      swept_.emplace_back(cell, before);
      ++stats.swept_voxels;
    }
  };
  const double walk_m = kWarmBodyWalkMps * config_.moving_sweep_s;
  for (const MovingBody3D& body : followed_bodies_) {
    // Unseen, a body keeps its place a walk wide for as long as it is read:
    // there or a walk from there. Given up once a scan had looked at where
    // it stood, the place was gone while the body, walked on and back, was
    // under the vehicle, out of every sensor's field, and the route came
    // down to 0.49 m over its head (r3645).
    if (body.unseen_s <= 0.0 && body.moving()) {
      sweep(body);
      continue;
    }
    // A walk not measured may go any way (the law charges it a person's):
    // the place is kept a person's walk in the sweep's time wider on every
    // side. Kept as the voxels alone, a body read 5 m off with no step yet
    // seen had every route laid 0.4 to 0.8 m over its head or before it, and
    // walked up to the vehicle held in its way, 0.33 m (r3564).
    // The place does not grow with the time unseen: a walk wider for every
    // second of it, the place of a body gone into a side passage stood ten
    // metres either way down the corridor for as long as the body was read,
    // 35 to 62 thousand voxels, the vehicle turned back from it five times
    // and the flight did not end (r3675; so r3671).
    // Its reach is a person's walk in the time the vehicle needs to rise over
    // the place: 1.2 m of climb at the 0.4 m/s it climbed before a body
    // (r3672, r3673) and a replan. A walk in the sweep's 1.5 s wide, the way
    // over an oncoming body climbed in the last 5 m before the place's edge;
    // the place came onto the climb every second, the route was released
    // each time, the search's answer was the way round the block, and in
    // twelve approaches the vehicle did not pass (r3679). So wide, the edge
    // stands at the vehicle's way out from 8.5 m and the climb is at once.
    constexpr double kRiseOverS{4.0};
    MovingBody3D wide = body;
    wide.radius_m += kWarmBodyWalkMps * kRiseOverS;
    sweep(wide);
  }
  // Motion without the class (roadmap item 21: a body out of the core's
  // field, seen by the pair or the ring alone): a cluster seen to step is
  // swept along its measured velocity while a person's walk in the sweep's
  // time would bring it to the vehicle's own place. The planner is handed no
  // body for it: such clusters are a wall's young skin 8 to 74 times a
  // flight (r3562 to r3566), a body once (r3561: unread from the side, passed
  // 0.63 m before it at 1.5 m/s).
  for (const MovingBody3D& body : stepping_clusters_) {
    if (std::hypot(body.centre_map.x - origin.x, body.centre_map.y - origin.y) <=
        body.radius_m + config_.moving_reach_m + walk_m + kKeepsClearOfVehicleM) {
      sweep(body);
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
          left_.push_back(cell);
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
  // A column's foot leaves with the column: the young occupancies under a
  // voxel of the layer that a scan saw gone or that lived out its life are
  // the same thing's lower end, ordinary only for standing beside the floor.
  // Seen from over 12 m a body's rings fall 0.35 m apart, no hit stood over
  // its feet in the scan that showed them, and the row a quarter metre over
  // the floor stayed along its walk for as long as nothing looked there
  // (r3502, r3532: 56 and 59 cell-snapshots up to 11.9 s old).
  for (const GridIndex3D gone : left_) {
    for (int below = 1; below <= 3; ++below) {
      const GridIndex3D cell{gone.x, gone.y, gone.z - below};
      if (!grid_.contains(cell)) {
        break;
      }
      if (!grid_.isOccupied(cell)) {
        continue;
      }
      if (grid_.isUnobservable(cell) || heldMoving(cell) ||
          confirmationsAt(cell) > kYoungConfirmations) {
        break;
      }
      const OccupancyChunkIndex3D chunk_index =
          ObservedOccupancyGrid3D::chunkIndex(cell);
      const auto scores = evidence_.find(chunk_index);
      if (scores != evidence_.end()) {
        const std::size_t bit_index = ObservedOccupancyGrid3D::localBitIndex(cell);
        scores->second.scores.at(bit_index) = 0.0;
        scores->second.confirmations.at(bit_index) = 0U;
      }
      static_cast<void>(grid_.setState(cell, ObservedVoxelState::kUnknown));
      dirty_chunks_[chunk_index] = true;
      ++stats.state_transitions;
    }
  }
  left_.clear();
}

} // namespace drone_city_nav

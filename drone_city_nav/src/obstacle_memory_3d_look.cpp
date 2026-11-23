#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

// A body no scan shows is taken off by a look at where it stood, not by a
// clock: the stand's time runs only over the scans that could have shown it
// there, and a scan can show it in two ways.
//
// By its rays: a cell observed at every level of the column the body stood
// in. Hovering 4 m up and 3.3 m before a pacing body, the stereo set sees
// neither its legs nor its class (the pair's and the core's fields pass over
// it); the stand's time ran out, the body was no body, and the vehicle flew
// down at it at 2.1 m/s, to sight it 2.7 m off (r3582, -0.19 m).
//
// By the frame of the sensor that reads bodies (the thermal core): the upper
// half of the column, its middle and its top, inside the frame, within the
// range a body is read at, with no occupancy the memory holds on the way to
// it. The core reads a body by a warm region cut by its frame's edge too
// (K64), so a body standing there would have been read by its upper half; the
// foot is not asked for: looking straight at a place 6 m off from 2.7 m over
// its floor, the heading turned to it by the gaze, the vehicle stood 12 s
// with the body gone, the column's foot 23 degrees under the core's axis, its
// middle 16 and its top 8 (r4064). Sighted across a room from 8 m,
// a body walked up its passage behind the wall, was no body 5 s later, and
// the vehicle came down into the passage and met it stepping back (r3944,
// -0.33 m); read from 5.6 m, a body walked aside out of the core's 57 degrees
// while the vehicle stood, the pair not reaching it, was no body 5 s later,
// and came back unread as the vehicle crossed its line (r4017, -0.47 m).
// Over the 24 meetings kept from 2a2541c7 the clock alone took 29 bodies off
// within the 10 m a place is kept in, the place aside of the core's frame for
// 170 of the 176 s they stood unseen (tools/unlooked_bodies.py).
bool ObstacleMemory3D::scanLooksAt(const MovingBody3D& body,
                                   const ScanEvidence& scan_evidence,
                                   const BodyReadingView3D& view) const {
  MovingBody3D column = body;
  column.velocity_map_mps = Vec3{};
  column.radius_m = std::max(column.radius_m, grid_.bounds().resolution_m);
  const std::vector<GridIndex3D> stood = sweptCells3D(column, 1.0, 0.0, grid_.bounds());
  if (stood.empty()) {
    return true;
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
  if (std::ranges::all_of(looked, [](const bool level) { return level; })) {
    return true;
  }
  if (!(view.range_m > 0.0)) {
    return false;
  }
  const double step_m = 0.5 * grid_.bounds().resolution_m;
  // The column's middle and top, at the centres of its cells.
  const double foot_m = grid_.cellCenter(low).z;
  const double top_m = grid_.cellCenter(high).z;
  for (const double level_m : {0.5 * (foot_m + top_m), top_m}) {
    const Vec3 to{body.centre_map.x - view.origin_map.x,
                  body.centre_map.y - view.origin_map.y, level_m - view.origin_map.z};
    const double length_m = std::hypot(to.x, to.y, to.z);
    const double ahead_m = to.x * view.forward_map.x + to.y * view.forward_map.y +
                           to.z * view.forward_map.z;
    const double across_m =
        to.x * view.across_map.x + to.y * view.across_map.y + to.z * view.across_map.z;
    const double up_m =
        to.x * view.up_map.x + to.y * view.up_map.y + to.z * view.up_map.z;
    if (length_m > view.range_m || !(ahead_m > 0.0) ||
        std::abs(std::atan2(across_m, ahead_m)) > view.half_across_rad ||
        std::abs(std::atan2(up_m, ahead_m)) > view.half_updown_rad) {
      return false;
    }
    // Up to the body's own place: its voxels and its reading's error are no
    // surface before it.
    const auto steps =
        static_cast<int>((length_m - body.radius_m - config_.moving_match_m) / step_m);
    for (int step = 1; step <= steps; ++step) {
      const double share = step * step_m / length_m;
      const std::optional<GridIndex3D> cell = grid_.worldToCell(
          {view.origin_map.x + share * to.x, view.origin_map.y + share * to.y,
           view.origin_map.z + share * to.z});
      if (cell.has_value() && grid_.isOccupied(*cell)) {
        return false;
      }
    }
  }
  return true;
}

} // namespace drone_city_nav

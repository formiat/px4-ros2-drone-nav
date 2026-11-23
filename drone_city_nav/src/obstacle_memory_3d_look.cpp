#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

// A body no scan shows is taken off by a look at where it stood, not by a
// clock: its stand's time stands still while a look the vehicle can still
// have is outstanding, within the distance a place is kept in (farther off it
// is not kept: a thing the core's marks put 5 to 10 m off where no body was
// stood read for 200 s, the vehicle held before it, r3649). Three looks can be
// outstanding.
//
// The rays' of a vehicle beside the place: until a scan has observed a cell
// at every level of the column the body stood in. Hovering 4 m up and 3.3 m
// before a pacing body, the stereo set sees neither its legs nor its class
// (the pair's and the core's fields pass over it); the stand's time ran out,
// the body was no body, and the vehicle flew down at it at 2.1 m/s, to sight
// it 2.7 m off (r3582, -0.19 m).
//
// The look round a surface: while an occupancy the memory holds stands
// between the vehicle and the place. Sighted across a room from 8 m, a body
// walked up its passage behind the wall, was no body 5 s later, and the
// vehicle came down into the passage and met it stepping back (r3944,
// -0.33 m).
//
// The turn of the heading: while the place stands aside of the frame of the
// sensor that reads bodies (the thermal core) and a turn of the heading
// would bring the body's upper half into it, within the frame's range and its
// field up and down. Read from 5.6 m, a body walked aside out of the core's
// 57 degrees while the vehicle stood, the pair not reaching it, was no body
// 5 s later, and came back unread as the vehicle crossed its line (r4017,
// -0.47 m); over the 24 meetings kept from 2a2541c7 the clock alone took 39
// bodies off within 10 m while a turn would have shown their place, aside of
// the core's frame for 187 of the 200 s they stood unseen
// (tools/unlooked_bodies.py). The gaze makes that
// turn (K87). The core reads a body by a warm region its frame cuts (K64),
// so the upper half is asked for, not the foot (r4064: 12 s stood with the
// heading on the place, the foot 23 degrees under the core's axis).
//
// Where no turn of the heading would bring the place into the frame (it lies
// under or over the frame's field from where the vehicle is, or beyond its
// range) no look is outstanding and the time runs: held there for as long as
// the place went unlooked, a vehicle that rose over a place under a ceiling
// 6 m up stood 17 s against the ceiling, 25 degrees over the place's middle,
// and drifted into it (r4066, a contact).
bool ObstacleMemory3D::lookOutstanding(const MovingBody3D& body,
                                       const ScanEvidence& scan_evidence,
                                       const LidarScan3DView& scan) const {
  const Point3& origin = scan.origin_map;
  const double apart_m =
      std::hypot(body.centre_map.x - origin.x, body.centre_map.y - origin.y);
  if (apart_m > kBodyPlaceKeptWithinM) {
    return false;
  }
  MovingBody3D column = body;
  column.velocity_map_mps = Vec3{};
  column.radius_m = std::max(column.radius_m, grid_.bounds().resolution_m);
  const std::vector<GridIndex3D> stood = sweptCells3D(column, 1.0, 0.0, grid_.bounds());
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
  if (std::ranges::all_of(looked, [](const bool level) { return level; })) {
    return false;
  }
  // Beside the place, within a walk and a metre of it, the sensors look over
  // it: the rays' look is outstanding.
  if (apart_m <= body.radius_m + config_.moving_reach_m +
                     kWarmBodyWalkMps * config_.moving_sweep_s + 1.0) {
    return true;
  }
  const double middle_m = 0.5 * (body.bottom_m + body.top_m);
  const double step_m = 0.5 * grid_.bounds().resolution_m;
  const Vec3 aim{body.centre_map.x - origin.x, body.centre_map.y - origin.y,
                 middle_m - origin.z};
  const double aim_m = std::hypot(aim.x, aim.y, aim.z);
  // Up to the body's own place: its voxels and its reading's error are no
  // surface before it.
  const auto steps =
      static_cast<int>((aim_m - body.radius_m - config_.moving_match_m) / step_m);
  for (int step = 1; step <= steps; ++step) {
    const double share = step * step_m / aim_m;
    const std::optional<GridIndex3D> cell = grid_.worldToCell(
        {origin.x + share * aim.x, origin.y + share * aim.y, origin.z + share * aim.z});
    if (cell.has_value() && grid_.isOccupied(*cell)) {
      return true;
    }
  }
  const BodyReadingView3D& view = scan.body_view;
  if (!(view.range_m > 0.0)) {
    return false;
  }
  bool aside = false;
  for (const double level_m : {middle_m, grid_.cellCenter(high).z}) {
    const Vec3 to{body.centre_map.x - view.origin_map.x,
                  body.centre_map.y - view.origin_map.y, level_m - view.origin_map.z};
    const double ahead_m = to.x * view.forward_map.x + to.y * view.forward_map.y +
                           to.z * view.forward_map.z;
    const double across_m =
        to.x * view.across_map.x + to.y * view.across_map.y + to.z * view.across_map.z;
    const double up_m =
        to.x * view.up_map.x + to.y * view.up_map.y + to.z * view.up_map.z;
    // What a turn about the frame's own upright cannot change: the range and
    // the height over or under the frame's axis.
    if (std::hypot(to.x, to.y, to.z) > view.range_m ||
        std::abs(std::atan2(up_m, std::hypot(ahead_m, across_m))) >
            view.half_updown_rad) {
      return false;
    }
    aside = aside || !(ahead_m > 0.0) ||
            std::abs(std::atan2(across_m, ahead_m)) > view.half_across_rad;
  }
  return aside;
}

} // namespace drone_city_nav

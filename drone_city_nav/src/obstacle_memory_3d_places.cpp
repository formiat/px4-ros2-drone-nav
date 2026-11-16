#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

// The places and the ways the followed bodies keep between scans (roadmap
// item 21, specification K66): occupied by no measurement, lifted before the
// next scan's evidence applies.

namespace drone_city_nav {

void ObstacleMemory3D::layPlaces(const Point3& origin, ObstacleMemory3DStats& stats) {
  const auto apartM = [](const Point3& from, const Point3& to) {
    return std::hypot(to.x - from.x, to.y - from.y);
  };
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
  constexpr double kPlaceHoldsM{0.75};
  // The rise before a body: how far off a body's place first reaches the
  // vehicle. Laid round the body alone, the place of an oncoming body came
  // into the vehicle's way with the pair's first returns, 4.7 m and 3.3 s
  // before the meeting; the first route was a climb where the vehicle stood,
  // it rose 1.26 m through two more releases, 0.38 m/s, and met the body
  // 0.80 m over its head (r3685). From the level of a body's chest it has
  // 1.9 m to rise to be 1.2 m over a head: 5 s, and at the closing of a
  // walker and a vehicle the body's law has slowed, 1.9 m/s, 10 m.
  constexpr double kRiseWithinM{10.0};
  const std::vector<MovingBody3D> places_before = std::move(held_places_);
  held_places_.clear();
  const auto stepped = [step_m = grid_.bounds().resolution_m](const double held,
                                                              const double read) {
    return std::abs(read - held) <= step_m ? held
                                           : held + std::copysign(step_m, read - held);
  };
  for (const MovingBody3D& body : followed_bodies_) {
    // Unseen, a body keeps its place a walk wide for as long as it is read:
    // there or a walk from there. Given up once a scan had looked at where
    // it stood, the place was gone while the body, walked on and back, was
    // under the vehicle, out of every sensor's field, and the route came
    // down to 0.49 m over its head (r3645).
    // A body laid by its size alone keeps no place: it is the law's, which
    // slows the vehicle for it from afar. With the place of a body 10 to 19 m
    // off in the planner's world, the ways laid went through what the vehicle
    // had not seen (the side passages, the rooms over the corridor's walls),
    // the vehicle turned back from the body again and again, and four
    // flights of five did not end (r3676, r3679 to r3682; r3678 passed).
    // Within the rise's distance a body's place reaches the vehicle's way
    // out (below), a body laid by its size as any.
    const bool near = apartM(origin, body.centre_map) <= kRiseWithinM;
    if (body.sized && !near) {
      continue;
    }
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
    // A walk in 4 s wide (e96a0b19: the edge at the vehicle's way out from
    // 8.5 m, the climb at once) the place shut the side passages too; the
    // planner laid its ways through what it had not seen, and the vehicle did
    // not climb over once (r3680).
    // The place stands still while the body's reading stays within it: its
    // centre while the reading is within three quarters of a metre, its top,
    // bottom and radius a cell a scan towards the reading. Laid anew on every
    // scan's reading, it changed a thousand to sixteen hundred voxels of the
    // planner's world a scan; the search was cancelled by the next scan before
    // it converged, and its first feasible answer, a way round the block 81 s
    // longer, was taken by the vehicle the body's law had stopped (r3679:
    // twelve approaches, no pass, the flight not ended).
    MovingBody3D place = body;
    const auto before =
        std::ranges::find_if(places_before, [&](const MovingBody3D& held) {
          return apartM(held.centre_map, body.centre_map) <= kPlaceHoldsM;
        });
    if (before != places_before.end()) {
      place.centre_map = before->centre_map;
      place.radius_m = stepped(before->radius_m, body.radius_m);
      place.bottom_m = stepped(before->bottom_m, body.bottom_m);
      place.top_m = stepped(before->top_m, body.top_m);
    }
    held_places_.push_back(place);
    MovingBody3D wide = place;
    wide.radius_m += walk_m;
    if (near) {
      // The stretch between the body and the vehicle is the body's too: it
      // may be coming, and the vehicle is to be over the place when it does.
      // The swept cells are a walk from the body's place to the vehicle's.
      wide.velocity_map_mps =
          Vec3{(origin.x - place.centre_map.x) / config_.moving_sweep_s,
               (origin.y - place.centre_map.y) / config_.moving_sweep_s, 0.0};
    }
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

} // namespace drone_city_nav

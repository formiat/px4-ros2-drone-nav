#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {

namespace {

// Horizontal metres a climb or a descent takes per metre of height, steepest
// last: a slant keeps the leg's speed, and a vertical line is flown from rest.
// A gentler one than two to one spends its last hundred metres under the
// crowns' tops, where the trees not yet seen stand (r3261). Half a metre a
// metre before the vertical line gained nothing: the first leg's climb along
// it rose among the crowns at the clearance law's 3 to 5 m/s, 15.9 and 16.2 s
// against the vertical line's 15.0 (r3330, r3331).
constexpr std::array<double, 3U> kSlantRunsPerMetre{2.0, 1.0, 0.0};

// The vertical speed a vehicle in flight climbs a step of the leg's level at:
// its climb is as long as its speed over this, so the step is flown at the
// leg's speed. Two to one, a step of 5 m seen at 14 m/s asked 7 m/s of climb,
// and the vehicle slowed to 6.8 m/s for it and lost 1.6 s (r3330).
constexpr double kStepClimbRateMps{2.5};

// A descent is as long as the leg's speed over the vehicle's speed of descent
// asks (the time model's two, 15 and 5 m/s on the open sky: three to one), no
// gentler than four to one, so the leg's speed is carried into it. Two to one
// asks 6.75 m/s of descent at 13.5 m/s, and the vehicle slowed to 8 to 9 m/s
// at the top of every descent (r3338, r3342). Read from the vehicle's speed
// when the leg is taken, the slant was two to one on every leg taken at rest.
// A vehicle already coming down inside the gentlest cone goes straight to the
// descent's foot.
constexpr double kGentlestDescentRun{4.0};

// A slanted climb is also tried from this far straight up: the ground a
// vehicle took off from closes every slant from where it stands, the first
// leg was taken before the takeoff, climbed its whole 26 m on the spot and
// gathered its speed at the top, 14.8 s to 100 m out against the 11.5 to
// 13.6 s of the legs climbed along a slant (r3364, r3366).
constexpr double kClimbStepM{3.0};
constexpr double kDescendingMps{0.5};

// The level metres flown into a goal at its height: room for the stop in the
// plane from the corner (6 m/s stops in 3.6 m at the open sky's braking of
// 5 m/s^2, specification V3). Ten were worse: where neither slant cleared the
// descent was the vertical one 10 m short of the goal, the vehicle ran past
// its top and came down near the goal itself, 4 m under its height (r3295) or
// onto the ground (r3294).
constexpr double kLevelApproachM{5.0};

// The height over the goal's the descent ends at, the approach's 5 m coming
// down the rest: the descents ran 0.4 to 1.1 m under their feet (r3307 to
// r3310), and a foot at the goal's own height put the vehicle under its
// route's lowest point, where the speed law holds a descent's reference at
// nought (specification K78), and brought it to the goal from below, climbing
// at 0.5 to 2.5 m/s and at 0.4 to 1.7 m/s in the plane (r3305 to r3310).
constexpr double kApproachDropM{1.0};

// The horizontal distance from a point to the leg's ground track.
[[nodiscard]] double trackDistanceM(const double x, const double y, const Point3& start,
                                    const Point3& goal) noexcept {
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  const double along =
      length_squared > 0.0
          ? std::clamp(((x - start.x) * dx + (y - start.y) * dy) / length_squared, 0.0,
                       1.0)
          : 0.0;
  return std::hypot(x - start.x - along * dx, y - start.y - along * dy);
}

// The top of the highest occupied voxel the memory holds within `corridor_m`
// of the leg's ground track.
[[nodiscard]] std::optional<double>
highestObservedObstacleZM(const ObservedOccupancyGrid3D& occupancy, const Point3& start,
                          const Point3& goal, const double corridor_m) noexcept {
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const GridBounds3D& bounds = occupancy.bounds();
  const double chunk_m = static_cast<double>(kChunkSize) * bounds.resolution_m;
  // A chunk's centre lies within half its diagonal of each of its cells.
  const double chunk_reach_m = corridor_m + chunk_m * std::numbers::sqrt2 / 2.0;
  std::optional<int> top_cell;
  for (const auto& [index, storage] : occupancy.chunks()) {
    if (top_cell.has_value() && (index.z + 1) * kChunkSize - 1 <= *top_cell) {
      continue;
    }
    if (trackDistanceM(bounds.origin_x + (static_cast<double>(index.x) + 0.5) * chunk_m,
                       bounds.origin_y + (static_cast<double>(index.y) + 0.5) * chunk_m,
                       start, goal) > chunk_reach_m) {
      continue;
    }
    const OccupancyGrid3D::Chunk& occupied = storage->occupied;
    for (int layer = kChunkSize - 1; layer >= 0; --layer) {
      const int cell_z = index.z * kChunkSize + layer;
      if (top_cell.has_value() && cell_z <= *top_cell) {
        break;
      }
      bool layer_occupied{false};
      for (int row = 0; row < kChunkSize && !layer_occupied; ++row) {
        for (int column = 0; column < kChunkSize && !layer_occupied; ++column) {
          const GridIndex3D cell{index.x * kChunkSize + column,
                                 index.y * kChunkSize + row, cell_z};
          const std::size_t bit = ObservedOccupancyGrid3D::localBitIndex(cell);
          layer_occupied =
              (occupied.at(bit / 64U) & (std::uint64_t{1U} << (bit % 64U))) != 0U &&
              trackDistanceM(bounds.origin_x + (static_cast<double>(cell.x) + 0.5) *
                                                   bounds.resolution_m,
                             bounds.origin_y + (static_cast<double>(cell.y) + 0.5) *
                                                   bounds.resolution_m,
                             start, goal) <= corridor_m;
        }
      }
      if (layer_occupied) {
        top_cell = cell_z;
        break;
      }
    }
  }
  if (!top_cell.has_value()) {
    return std::nullopt;
  }
  return bounds.origin_z + static_cast<double>(*top_cell + 1) * bounds.resolution_m;
}

} // namespace

// The open sky's leg (specification K76): over everything the memory holds
// along it, within the planner's clearance reach of its ground track (a crown
// 40 m aside of the leg raised every leg of r3273 to 49 m, and its descents to
// the goals took 20 s each). The level is the highest obstacle seen there plus
// the margin, under the envelope's
// ceiling and no lower than either end; the vehicle climbs to it along the leg
// and comes down along it at the gentlest slant the raw world clears, to a
// metre over the goal's height 5 m short of it, and comes down that metre over
// the rest. Unknown space on the way is
// free (I1): what stands in it is seen from the sensor's range, and the route is taken
// again from where the vehicle then is. Nothing is searched: with the leg held the
// lattice searches idle.
std::optional<std::vector<Point3>> PersistentDStarLitePlanner3DImpl::overflightPath(
    const Point3& start, const Point3& goal, const Vec3& velocity) const {
  if (!(config_.overflight_margin_m > 0.0) || world_.observed_occupancy == nullptr) {
    return std::nullopt;
  }
  const double horizontal_m = std::hypot(goal.x - start.x, goal.y - start.y);
  if (!(horizontal_m > config_.minimum_horizontal_step_m)) {
    return std::nullopt;
  }
  const double ceiling_m =
      config_.flight_envelope.maximum_target_z_m - config_.minimum_vertical_step_m;
  const std::optional<double> top_m = highestObservedObstacleZM(
      *world_.observed_occupancy, start, goal, config_.clearance_ranking_distance_m);
  const double level_m = std::min(
      ceiling_m,
      std::max({start.z, goal.z,
                top_m.has_value() ? *top_m + config_.overflight_margin_m : start.z}));
  const double direction_x = (goal.x - start.x) / horizontal_m;
  const double direction_y = (goal.y - start.y) / horizontal_m;
  // The last metres into the goal are flown a metre down from the descent's
  // foot where they fit and clear, so the descent all but stops at their
  // beginning in flight. Stopping both at the goal, the vehicle braked in the
  // plane first and its descent ran on under it, 0.5 to 1.8 m past the route's
  // end, whose recovery cost 5 to 16 s at a goal (r3274, r3287, r3289).
  for (const double approach_m : {kLevelApproachM, 0.0}) {
    const double run_m = horizontal_m - approach_m;
    if (!(run_m > config_.minimum_horizontal_step_m)) {
      continue;
    }
    const Point3 foot{
        goal.x - direction_x * approach_m, goal.y - direction_y * approach_m,
        approach_m > 0.0 ? std::min(level_m, goal.z + kApproachDropM) : goal.z};
    const auto clears =
        [&](std::vector<Point3> path) -> std::optional<std::vector<Point3>> {
      if (approach_m > 0.0) {
        path.push_back(goal);
      }
      if (path.size() >= 2U && lattice_.pathTraversable(path)) {
        return path;
      }
      return std::nullopt;
    };
    // On the way down already (inside the gentlest descent's cone over the
    // goal): the rest of the descent is taken from where the vehicle is, level
    // at its height and then down, before the leg is climbed again. A descent a
    // crown had closed sent the vehicle 25 m back up to the level and cost it
    // 20 s (r3261).
    const double remaining_descent_m = start.z - foot.z;
    // The descent's slants: the vehicle's own first, where it is gentler.
    const auto slants_after = [](const double first) {
      std::vector<double> runs{kSlantRunsPerMetre.begin(), kSlantRunsPerMetre.end()};
      if (first > runs.front()) {
        runs.insert(runs.begin(), first);
      }
      return runs;
    };
    const double flown_descent_run = std::min(
        kGentlestDescentRun, config_.time_model.maximum_horizontal_speed_mps /
                                 config_.time_model.maximum_vertical_speed_mps);
    const std::vector<double> descent_runs = slants_after(flown_descent_run);
    if (remaining_descent_m > 0.0 &&
        run_m <= kGentlestDescentRun * remaining_descent_m +
                     config_.minimum_horizontal_step_m) {
      for (const double descent_run :
           slants_after(-velocity.z > kDescendingMps ? kGentlestDescentRun
                                                     : flown_descent_run)) {
        const double descent_run_m = std::min(run_m, descent_run * remaining_descent_m);
        std::vector<Point3> path{start};
        if (run_m - descent_run_m > 1.0e-6) {
          path.push_back(Point3{foot.x - direction_x * descent_run_m,
                                foot.y - direction_y * descent_run_m, start.z});
        }
        path.push_back(foot);
        if (std::optional<std::vector<Point3>> cleared = clears(std::move(path))) {
          return cleared;
        }
      }
    }
    const double climb_m = std::max(0.0, level_m - start.z);
    const double descent_m = std::max(0.0, level_m - foot.z);
    // The climb's slants: the vehicle's own first, where it is gentler.
    std::vector<double> climb_runs{kSlantRunsPerMetre.begin(),
                                   kSlantRunsPerMetre.end()};
    if (const double flown_run = std::hypot(velocity.x, velocity.y) / kStepClimbRateMps;
        flown_run > climb_runs.front()) {
      climb_runs.insert(climb_runs.begin(), flown_run);
    }
    for (const double descent_run : descent_runs) {
      // Of the climbs that clear, the one the time model flies soonest: a
      // slant beside a crown is as slow as its clearance makes it. The first
      // that cleared, two to one, passed 0.2 m off the crowns by a goal in
      // three flights of nine, and its climb took 10 to 13 s against 5
      // (r3346, r3350, r3352).
      std::optional<std::vector<Point3>> soonest;
      double soonest_s{std::numeric_limits<double>::infinity()};
      for (const double climb_run : climb_runs) {
        for (const double step_m : {0.0, kClimbStepM}) {
          const double climb_run_m = climb_run * (climb_m - step_m);
          const double descent_run_m = descent_run * descent_m;
          // A height that is not there has one slant, not three, and a climb
          // on the spot no step.
          if (climb_run_m + descent_run_m > run_m ||
              (!(climb_m > 0.0) && climb_run != climb_runs.front()) ||
              (!(descent_m > 0.0) && descent_run != descent_runs.front()) ||
              (step_m > 0.0 && !(climb_run_m > 0.0))) {
            continue;
          }
          std::vector<Point3> path{start};
          const auto append = [&path](const Point3& point) {
            if (distance3D(path.back(), point) > 1.0e-6) {
              path.push_back(point);
            }
          };
          append(Point3{start.x, start.y, start.z + step_m});
          append(Point3{start.x + direction_x * climb_run_m,
                        start.y + direction_y * climb_run_m, level_m});
          append(Point3{foot.x - direction_x * descent_run_m,
                        foot.y - direction_y * descent_run_m, level_m});
          append(foot);
          if (std::optional<std::vector<Point3>> cleared = clears(std::move(path))) {
            const FlightPathTimeProfile3D profile = pathTimeProfile(*cleared, velocity);
            if (const double ranked_s = lattice_.rankedPathTimeS(*cleared, profile);
                profile.valid && ranked_s < soonest_s) {
              soonest_s = ranked_s;
              soonest = std::move(cleared);
            }
          }
        }
      }
      if (soonest.has_value()) {
        return soonest;
      }
    }
  }
  return std::nullopt;
}

bool PersistentDStarLitePlanner3DImpl::holdOverflight(
    const PersistentPlannerRequest3D& request, PlannerUpdate3D& update) {
  // A held leg is taken again from where the vehicle is at every update, at no
  // lower a level than the vehicle's own: so it stays the leg over everything
  // the memory holds as the memory grows, and its start is the vehicle, not a
  // join. Held as it was first taken, a crown seen since within the leg's
  // reach and under its margin, which the bare body still cleared, left it the
  // incumbent: a consumer that released it as blocked was handed the same leg
  // three times and threaded the crowns at 0.6 to 1 m/s for 12 s (r3297).
  if (const SpatialRouteCandidate3D* const held = coordinator_.incumbent();
      held != nullptr && held->source == SpatialRouteCandidateSource3D::kOverflight) {
    if (std::optional<std::vector<Point3>> path =
            overflightPath(request.start, exact_goal_, request.velocity)) {
      if (std::optional<SpatialRouteCandidate3D> again = makeCandidate(
              std::move(*path), SpatialRouteCandidateSource3D::kOverflight,
              request.velocity)) {
        coordinator_.retain(std::move(*again));
      }
    }
  }
  if (coordinator_.incumbent() == nullptr) {
    if (std::optional<std::vector<Point3>> path =
            overflightPath(request.start, exact_goal_, request.velocity)) {
      if (std::optional<SpatialRouteCandidate3D> candidate = makeCandidate(
              std::move(*path), SpatialRouteCandidateSource3D::kOverflight,
              request.velocity)) {
        update.improved_incumbent = coordinator_.consider(std::move(*candidate));
      }
    }
  }
  return coordinator_.incumbent() != nullptr &&
         coordinator_.incumbent()->source == SpatialRouteCandidateSource3D::kOverflight;
}

} // namespace drone_city_nav::detail

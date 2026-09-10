#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/derived_clearance_3d.hpp"
#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

// A sample of the motion under execution whose body clearance falls below the
// constraint threshold: how far along the remaining motion it lies, and the
// clearance there.
struct ConstrainedHorizonSample3D {
  double distance_m{0.0};
  double clearance_m{0.0};
};

// Where the motion the vehicle is carrying out right now comes close to known
// occupied evidence.
//
// The speed policy needs this from the horizon under execution, not from the
// last controller candidate: a candidate that was never published moved
// nothing, and sizing the reference speed from its clearance makes the
// reference chase trajectories the vehicle never flew. It needs every
// constrained sample rather than the minimum over the whole horizon: a
// single grazing sample fifteen metres ahead is a reason to arrive there
// slowly, not a reason to crawl now, and reading it as an immediate cap makes
// the reference oscillate as the horizon shortens and lengthens around it.
// A mild constraint close by does not hide a tight one behind it either, so
// the samples are kept in path order for the policy to fold.
struct ExecutedHorizonClearance3D {
  bool available{false};
  // Every sample below the constraint threshold, nearest first.
  std::vector<ConstrainedHorizonSample3D> constrained_samples;
  // Smallest body clearance anywhere on the remaining motion, for diagnostics.
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  // Path length along the remaining motion to the first sample whose body
  // envelope reaches space the evidence has not observed. Unknown space is not
  // a clearance constraint — it stays traversable and carries no penalty — but
  // it is where the sensor's guarantee ends along this motion: nothing beyond
  // it has been seen, so the sensor-braking law reads this as the range the
  // evidence actually covers, in place of the range the sensor guarantees in
  // the open. Absent when the whole remaining motion runs through observed
  // space.
  std::optional<double> unobserved_distance_m;

  [[nodiscard]] bool constrained() const noexcept {
    return available && !constrained_samples.empty();
  }

  // Path length along the remaining motion to the first constrained sample.
  [[nodiscard]] double distanceToConstraintM() const noexcept {
    return constrained() ? constrained_samples.front().distance_m
                         : std::numeric_limits<double>::infinity();
  }

  // Body clearance at the first constrained sample.
  [[nodiscard]] double constrainedClearanceM() const noexcept {
    return constrained() ? constrained_samples.front().clearance_m
                         : std::numeric_limits<double>::infinity();
  }

  // The remaining motion enters space the evidence has not observed.
  [[nodiscard]] bool unobserved() const noexcept {
    return available && unobserved_distance_m.has_value();
  }

  // Path length along the remaining motion to the first unobserved sample.
  [[nodiscard]] double distanceToUnobservedM() const noexcept {
    return unobserved() ? *unobserved_distance_m
                        : std::numeric_limits<double>::infinity();
  }
};

// Measures the remaining part of `horizon` from `first_remaining_state_index`
// against the current evidence. Samples whose clearance is unknown or off the
// grid are not constraints: unknown space stays traversable and carries no
// penalty, and the body validation remains the only hard authority. The first
// such sample is recorded as the end of the observed range along the motion.
[[nodiscard]] ExecutedHorizonClearance3D measureExecutedHorizonClearance3D(
    const FiniteMotionHorizon3D& horizon, std::size_t first_remaining_state_index,
    const EsdfGrid3D& grid, std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint, double constraint_clearance_m);

// Route length from `from_station_m` to the start of the first route segment
// whose body envelope reaches space the evidence has not observed, probing no
// farther than `lookahead_m`. The executed horizon ends where the vehicle can
// come to rest, so the frontier it can find is never beyond the stopping path
// and it finds none while the vehicle can still stop before it; the route is
// where the vehicle goes next, and the evidence along it is probed as far as
// the latest scan is. Absent when the route stays observed through the
// lookahead, or ends inside it.
[[nodiscard]] std::optional<double>
measureRouteObservedRange3D(std::span<const RouteSample3D> route, double from_station_m,
                            double lookahead_m, const EsdfGrid3D& grid,
                            std::span<const float> esdf_m,
                            const SweptFootprintConfig& footprint);

// Where the route ahead of `from_station_m` comes close to known occupied
// evidence, probing no farther than `lookahead_m`, in the same form the
// executed horizon reports.
//
// The executed horizon ends where the vehicle can come to rest, so how far it
// reaches is decided by the speed the reference already asked for: at rest it
// sees only the space beside the vehicle, admits cruise, and finds the tight
// spot ahead only once the vehicle is fast enough to reach into it. The route
// is the geometry the vehicle is committed to and its clearance profile does
// not move with the speed, so reading the same tube and stopping laws off it
// bounds the speed before the horizon grows into the constraint. Unobserved
// space is not reported here: it is a constraint on the sensor-braking range,
// which the route already answers for through measureRouteObservedRange3D.
[[nodiscard]] ExecutedHorizonClearance3D measureRouteClearance3D(
    std::span<const RouteSample3D> route, double from_station_m, double lookahead_m,
    const EsdfGrid3D& grid, std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint, double constraint_clearance_m);

} // namespace drone_city_nav

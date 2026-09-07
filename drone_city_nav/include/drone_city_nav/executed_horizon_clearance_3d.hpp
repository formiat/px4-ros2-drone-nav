#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/derived_clearance_3d.hpp"
#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <limits>
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
};

// Measures the remaining part of `horizon` from `first_remaining_state_index`
// against the current evidence. Samples whose clearance is unknown or off the
// grid are not constraints: unknown space stays traversable and carries no
// penalty, and the body validation remains the only hard authority.
[[nodiscard]] ExecutedHorizonClearance3D measureExecutedHorizonClearance3D(
    const FiniteMotionHorizon3D& horizon, std::size_t first_remaining_state_index,
    const EsdfGrid3D& grid, std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint, double constraint_clearance_m);

} // namespace drone_city_nav

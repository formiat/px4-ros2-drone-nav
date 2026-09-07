#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/derived_clearance_3d.hpp"
#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

// Where the motion the vehicle is carrying out right now first comes close to
// known occupied evidence.
//
// The speed policy needs this from the horizon under execution, not from the
// last controller candidate: a candidate that was never published moved
// nothing, and sizing the reference speed from its clearance makes the
// reference chase trajectories the vehicle never flew. It also needs the first
// constrained point rather than the minimum over the whole horizon: a single
// grazing sample fifteen metres ahead is a reason to arrive there slowly, not
// a reason to crawl now, and reading it as an immediate cap makes the
// reference oscillate as the horizon shortens and lengthens around it.
struct ExecutedHorizonClearance3D {
  bool available{false};
  // Path length along the remaining motion to the first sample whose body
  // clearance falls below the constraint threshold.
  double distance_to_constraint_m{std::numeric_limits<double>::infinity()};
  // Body clearance at that sample.
  double constrained_clearance_m{std::numeric_limits<double>::infinity()};
  // Smallest body clearance anywhere on the remaining motion, for diagnostics.
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};

  [[nodiscard]] bool constrained() const noexcept {
    return available && std::isfinite(distance_to_constraint_m);
  }
};

// Measures the remaining part of `horizon` from `first_remaining_state_index`
// against the current evidence. Samples whose clearance is unknown or off the
// grid are not constraints: unknown space stays traversable and carries no
// penalty, and the body validation remains the only hard authority.
[[nodiscard]] ExecutedHorizonClearance3D measureExecutedHorizonClearance3D(
    const FiniteMotionHorizon3D& horizon, std::size_t first_remaining_state_index,
    const EsdfGrid3D& grid, std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint, double constraint_clearance_m) noexcept;

} // namespace drone_city_nav

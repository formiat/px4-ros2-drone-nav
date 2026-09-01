#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>

namespace drone_city_nav {

// Why a mission route stops being owned, and where the vehicle sits along it.
// Both are route contracts rather than execution state: planning, trajectory
// extension, execution, and diagnostics all speak them, so they live below the
// execution layer that acts on them.
enum class RouteReleaseReason3D : std::uint8_t {
  kNone,
  kNoActiveRoute,
  kBlocked,
  kExhausted,
  kStalled,
  kNoEligibleRollouts,
  kDiverged,
  kObjectiveChanged,
};

struct RouteProgressProjection3D {
  bool valid{false};
  double station_m{0.0};
  double total_length_m{0.0};
  double remaining_m{0.0};
  double cross_track_m{0.0};
  Point2 point{};
  Point2 tangent{};
};

} // namespace drone_city_nav

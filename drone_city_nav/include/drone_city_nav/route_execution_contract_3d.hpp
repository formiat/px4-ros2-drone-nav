#pragma once

#include "drone_city_nav/route_planning_3d.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace drone_city_nav {

// The opt-in route-adherence policy uses this centerline corridor. Independent
// physical validation remains mandatory when the policy is disabled.
inline constexpr double kFiniteExecutionRouteCrossTrackToleranceM3D{2.0};

[[nodiscard]] std::optional<float> routeCrossTrackTolerance3D(bool enabled) noexcept;

enum class RouteEndpointSemantics3D : std::uint8_t {
  kContinuation,
  kLocalStop,
  kMissionStop,
  kEmergencyBrakeTail,
};

struct RouteContinuityLineage3D {
  std::uint64_t mission_epoch{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
};

// `endpoint_is_local_stop` marks a route that ends at a stop of its own
// rather than continuing: the closest-approach route an exhausted frontier
// publishes stops where the search ran out, and the vehicle rests there while
// the extension plans on from it.
[[nodiscard]] RouteEndpointSemantics3D
routeEndpointSemantics3D(bool reaches_mission_goal, bool mission_endpoint_is_terminal,
                         bool endpoint_is_local_stop = false) noexcept;

// Only true endpoint semantics may shape the nominal route speed to zero.
// Unknown enum values fail closed as terminal stops.
[[nodiscard]] bool
routeEndpointHasTerminalStop3D(RouteEndpointSemantics3D semantics) noexcept;

// A local execution boundary keeps every non-mission finite fallback inside
// certified route geometry. Continuations still use this boundary even though
// their nominal speed profile remains nonzero.
[[nodiscard]] bool
routeEndpointUsesLocalBoundary3D(RouteEndpointSemantics3D semantics) noexcept;

[[nodiscard]] std::uint64_t
routeContinuityId3D(const RouteIntent3D& intent,
                    const RouteContinuityLineage3D& lineage = {}) noexcept;

[[nodiscard]] std::string_view
routeEndpointSemantics3DName(RouteEndpointSemantics3D semantics) noexcept;

} // namespace drone_city_nav

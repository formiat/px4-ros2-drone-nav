#pragma once

#include "drone_city_nav/route_planning_3d.hpp"

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

enum class RouteEndpointSemantics3D : std::uint8_t {
  kContinuation,
  kObservationStop,
  kMissionStop,
  kEmergencyBrakeTail,
};

struct RouteContinuityLineage3D {
  std::uint64_t mission_epoch{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
};

[[nodiscard]] RouteEndpointSemantics3D
routeEndpointSemantics3D(const RouteIntent3D& intent, bool reaches_intent_target,
                         bool reaches_mission_goal) noexcept;

[[nodiscard]] std::uint64_t
routeContinuityId3D(const RouteIntent3D& intent,
                    const RouteContinuityLineage3D& lineage = {}) noexcept;

[[nodiscard]] std::string_view
routeEndpointSemantics3DName(RouteEndpointSemantics3D semantics) noexcept;

} // namespace drone_city_nav

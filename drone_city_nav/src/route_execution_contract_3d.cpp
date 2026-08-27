#include "drone_city_nav/route_execution_contract_3d.hpp"

#include <cstddef>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t byte_index = 0U; byte_index < sizeof(value); ++byte_index) {
    hash ^= (value >> (byte_index * 8U)) & 0xffU;
    hash *= kFnvPrime;
  }
}

} // namespace

std::optional<float> routeCrossTrackTolerance3D(const bool enabled) noexcept {
  return enabled ? std::optional<float>{static_cast<float>(
                       kFiniteExecutionRouteCrossTrackToleranceM3D)}
                 : std::nullopt;
}

RouteEndpointSemantics3D
routeEndpointSemantics3D(const RouteIntent3D& intent, const bool reaches_intent_target,
                         const bool reaches_mission_goal,
                         const bool mission_endpoint_is_terminal) noexcept {
  if (reaches_mission_goal) {
    return mission_endpoint_is_terminal ? RouteEndpointSemantics3D::kMissionStop
                                        : RouteEndpointSemantics3D::kContinuation;
  }
  if (intent.purpose == RouteIntentPurpose3D::kObservationFrontier &&
      intent.observation_stop_required && reaches_intent_target) {
    return RouteEndpointSemantics3D::kObservationStop;
  }
  return RouteEndpointSemantics3D::kContinuation;
}

bool routeEndpointHasTerminalStop3D(const RouteEndpointSemantics3D semantics) noexcept {
  switch (semantics) {
    case RouteEndpointSemantics3D::kContinuation:
      return false;
    case RouteEndpointSemantics3D::kObservationStop:
    case RouteEndpointSemantics3D::kMissionStop:
    case RouteEndpointSemantics3D::kEmergencyBrakeTail:
      return true;
  }
  return true;
}

bool routeEndpointUsesLocalBoundary3D(
    const RouteEndpointSemantics3D semantics) noexcept {
  switch (semantics) {
    case RouteEndpointSemantics3D::kMissionStop:
      return false;
    case RouteEndpointSemantics3D::kContinuation:
    case RouteEndpointSemantics3D::kObservationStop:
    case RouteEndpointSemantics3D::kEmergencyBrakeTail:
      return true;
  }
  return true;
}

std::uint64_t routeContinuityId3D(const RouteIntent3D& intent,
                                  const RouteContinuityLineage3D& lineage) noexcept {
  if (!intent.valid) {
    return 0U;
  }
  const std::uint64_t route_lineage =
      intent.strategic_plan_id != 0U
          ? intent.strategic_plan_id
          : makeRouteIntentId3D(intent.source, intent.purpose, intent.mission_target,
                                intent.mission_target, intent.target_identity,
                                intent.observation_stop_required);
  if (lineage.mission_epoch == 0U && lineage.assignment_generation == 0U &&
      lineage.target_detection_id == 0U && lineage.target_track_id == 0U) {
    return route_lineage;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, route_lineage);
  hashValue(hash, lineage.mission_epoch);
  hashValue(hash, lineage.assignment_generation);
  hashValue(hash, lineage.target_detection_id);
  hashValue(hash, lineage.target_track_id);
  return hash == 0U ? 1U : hash;
}

std::string_view
routeEndpointSemantics3DName(const RouteEndpointSemantics3D semantics) noexcept {
  switch (semantics) {
    case RouteEndpointSemantics3D::kContinuation:
      return "continuation";
    case RouteEndpointSemantics3D::kObservationStop:
      return "observation_stop";
    case RouteEndpointSemantics3D::kMissionStop:
      return "mission_stop";
    case RouteEndpointSemantics3D::kEmergencyBrakeTail:
      return "emergency_brake_tail";
  }
  return "invalid";
}

} // namespace drone_city_nav

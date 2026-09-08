#include <string_view>

#include "route_lifecycle_coordinator_3d.hpp"

namespace drone_city_nav {

const char* routeCandidateDisposition3DName(
    const RouteCandidateDisposition3D disposition) noexcept {
  switch (disposition) {
    case RouteCandidateDisposition3D::kActivated:
      return "activated";
    case RouteCandidateDisposition3D::kRetrySameCandidate:
      return "retry_same_candidate";
    case RouteCandidateDisposition3D::kRetireSearchAndReplan:
      return "retire_search_and_replan";
    case RouteCandidateDisposition3D::kContinueForImprovement:
      return "continue_for_improvement";
    case RouteCandidateDisposition3D::kTerminalReject:
      return "terminal_reject";
  }
  return "continue_for_improvement";
}

std::string_view routeLifecycleExtensionStatus3DName(
    const RouteLifecycleExtensionStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleExtensionStatus3D::kNotRequired:
      return "not_required";
    case RouteLifecycleExtensionStatus3D::kInvalidRequest:
      return "invalid_request";
    case RouteLifecycleExtensionStatus3D::kInvalidTransaction:
      return "invalid_transaction";
    case RouteLifecycleExtensionStatus3D::kRouteQueueBusy:
      return "route_queue_busy";
    case RouteLifecycleExtensionStatus3D::kWorldRefreshUnavailable:
      return "world_refresh_unavailable";
    case RouteLifecycleExtensionStatus3D::kSearchQueued:
      return "search_queued";
    case RouteLifecycleExtensionStatus3D::kWorldRefreshQueued:
      return "world_refresh_queued";
  }
  return "unknown";
}

std::string_view
routeLifecycleReplanStatus3DName(const RouteLifecycleReplanStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleReplanStatus3D::kObjectiveUnavailable:
      return "objective_unavailable";
    case RouteLifecycleReplanStatus3D::kDeferredDuringExtension:
      return "deferred_during_extension";
    case RouteLifecycleReplanStatus3D::kDeferredReplanInFlight:
      return "deferred_replan_in_flight";
    case RouteLifecycleReplanStatus3D::kInvalidResidentWorld:
      return "invalid_resident_world";
    case RouteLifecycleReplanStatus3D::kWaitingInitialSearch:
      return "waiting_initial_search";
    case RouteLifecycleReplanStatus3D::kGenerationMismatch:
      return "generation_mismatch";
    case RouteLifecycleReplanStatus3D::kWaitingRawSnapshot:
      return "waiting_raw_snapshot";
    case RouteLifecycleReplanStatus3D::kInvalidStart:
      return "invalid_start";
    case RouteLifecycleReplanStatus3D::kSuppressedFailedSearch:
      return "suppressed_failed_search";
    case RouteLifecycleReplanStatus3D::kInvalidTransaction:
      return "invalid_transaction";
    case RouteLifecycleReplanStatus3D::kRouteQueueBusy:
      return "route_queue_busy";
    case RouteLifecycleReplanStatus3D::kQueued:
      return "queued";
  }
  return "unknown";
}

std::string_view routeLifecycleTrackingRefreshStatus3DName(
    const RouteLifecycleTrackingRefreshStatus3D status) noexcept {
  switch (status) {
    case RouteLifecycleTrackingRefreshStatus3D::kNotRequired:
      return "not_required";
    case RouteLifecycleTrackingRefreshStatus3D::kInvalidRequest:
      return "invalid_request";
    case RouteLifecycleTrackingRefreshStatus3D::kLifecycleBusy:
      return "lifecycle_busy";
    case RouteLifecycleTrackingRefreshStatus3D::kWorldRefreshUnavailable:
      return "world_refresh_unavailable";
    case RouteLifecycleTrackingRefreshStatus3D::kQueued:
      return "queued";
  }
  return "unknown";
}

} // namespace drone_city_nav

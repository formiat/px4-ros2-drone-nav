#pragma once

#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <memory>
#include <optional>

namespace drone_city_nav {

struct ProductionRouteExecutionSelection3D {
  std::shared_ptr<const CertifiedRouteSuffix3D> route;
  std::shared_ptr<const CommittedExecutionAuthority3D> source_authority;
  // The resident snapshot remains the single CAS predecessor. A successful
  // progress assessment is retained only as an immutable certification base
  // and cannot become controller-visible until a complete execution plan is
  // composed with it.
  std::shared_ptr<const ExecutionPlan3D> source_snapshot;
  std::shared_ptr<const ExecutionPlan3D> certification_snapshot;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> progress_preparation;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_route;
  std::shared_ptr<const VersionedObservedRawWorld3D> lifecycle_observed_raw_world;
  RouteProgressProjection3D projection{};
  TrackingErrorTubeExecutionAssessment3D tracking_error_tube{};
  TrackingErrorTubeHandoffAssessment3D tracking_error_tube_handoff{};
  RouteExecutionStatus3D status{RouteExecutionStatus3D::kNoActiveRoute};
  std::optional<RouteLifecycleEvent3D> lifecycle_event;
  Point3 hold_position{};
  double station_m{0.0};
  bool route_usable{false};
  bool tracking_error_tube_handoff_active{false};
  bool execution_owner_available{false};
  // The execution owner is a certified stationary hold: the vehicle holds a
  // position but executes no route, so it still needs a successor route.
  bool stationary_hold_owner{false};
  bool pending_activation{false};
  bool physical_trajectory_invalidated{false};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
};

} // namespace drone_city_nav

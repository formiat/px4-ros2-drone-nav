#pragma once

#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"
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
  // The execution owner executes no route of its own: a certified stationary
  // hold pins a position, a stop brakes to one. Either owns the wire while the
  // vehicle needs a successor route, so neither may suppress the search for
  // it.
  bool routeless_execution_owner{false};
  // Why the sealed successor is or is not the route the plan may take next.
  PendingRouteEligibility3D pending_eligibility{
      PendingRouteEligibility3D::kInvalidPending};
  bool pending_activation{false};
  // Why an eligible sealed successor was not taken this tick. A successor the
  // plan may take and does not take is the state a vehicle waits in with a
  // route already certified, and it left no trace at all: the hold that
  // followed named the eligibility, which said the successor was fine.
  bool pending_refresh_available{false};
  // Why the refresh against the current world produced no route, when it
  // produced none. The recertification already decides this; discarding it
  // left the wait unexplained.
  RouteCertificationStatus3D pending_refresh_status{
      RouteCertificationStatus3D::kNotAttempted};
  RouteSpliceReadinessStatus3D pending_splice_readiness{
      RouteSpliceReadinessStatus3D::kReady};
  bool physical_trajectory_invalidated{false};
  // Station of the first route sample the persistent raw world blocks while
  // the route is still followed and its replacement is being searched: the
  // speed policy brakes toward it as toward a route end.
  std::optional<double> raw_blocked_station_m;
  // Station of the first route sample within the lookahead that the latest
  // lidar scan touches. The scan is evidence the persistent memory reaches
  // only after its own integration; the vehicle brakes toward it now, as it
  // brakes toward a persistent block, instead of meeting it at the end of its
  // horizon and stopping hard.
  std::optional<double> latest_lidar_blocked_station_m;
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
};

} // namespace drone_city_nav

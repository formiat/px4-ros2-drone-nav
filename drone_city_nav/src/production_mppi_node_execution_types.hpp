#pragma once

#include "drone_city_nav/cooperative_mppi_adapter.hpp"
#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/noncooperative_collision_avoidance.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_planner_3d.hpp"

namespace drone_city_nav {

struct ProductionMppiStability {
  double first_control_delta{0.0};
  double position_rms_m{0.0};
  double position_max_m{0.0};
  double terminal_shift_m{0.0};
  bool valid{false};
};

struct ProductionMppiPredictionError {
  double position_m{0.0};
  double velocity_mps{0.0};
  double yaw_rad{0.0};
  bool valid{false};
};

struct ProductionMppiCooperativeCommand {
  CooperativeManeuverCommandData data;
  std::int64_t receive_stamp_ns{0};
};

struct ProductionMppiCooperativeUpdate {
  CooperativeMppiAdapterResult mppi{};
  CooperativePassageUse passage{};
  CooperativePassageYieldDecision yield{};
  std::uint64_t command_generation{0U};
  double command_age_ms{-1.0};
};

struct ProductionMppiNonCooperativeTracks {
  std::vector<NonCooperativeAircraftTrack> tracks;
  std::uint64_t source_scan_sequence{0U};
  std::int64_t receive_stamp_ns{0};
};

struct ProductionMppiNonCooperativeUpdate {
  NonCooperativeAvoidanceUpdate avoidance{};
  std::uint64_t source_scan_sequence{0U};
  double transport_age_ms{-1.0};
  bool enabled{false};
};

struct ProductionRoutePlanningWork3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  std::shared_ptr<const RoutePlannerSession3D> continuation_session;

  [[nodiscard]] bool valid() const noexcept {
    return transaction != nullptr && transaction->valid();
  }
};

struct ProductionRouteExecutionSelection3D {
  std::shared_ptr<const CertifiedRouteSuffix3D> route;
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
  bool pending_activation{false};
  bool physical_trajectory_invalidated{false};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
};

} // namespace drone_city_nav

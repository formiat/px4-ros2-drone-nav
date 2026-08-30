#pragma once

// Included inside namespace drone_city_nav after the route pipeline contracts.

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

struct ProductionRouteSearchCandidate3D {
  Point3 search_start{};
  Vec3 search_velocity{};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;
  RouteIntent3D intent{};
  SegmentEvidence3D evidence{};
  SpatialRouteCandidate3D spatial_route{};
  PlannerInputStatus3D planner_input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D planner_progress{SearchProgress3D::kInvalidated};
  PlannerTelemetry3D planner_telemetry{};
  std::vector<RouteSample3D> route;
};

struct ProductionPlannerSession3D {
  PersistentPlannerRequest3D request{};
  Point3 mission_goal{};
  Point3 search_start{};
  Vec3 search_velocity{};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;
  RouteIntent3D intent{};
};

struct ProductionPlannerUpdate3D {
  std::optional<ProductionRouteSearchCandidate3D> improved_incumbent;
  std::shared_ptr<const ProductionPlannerSession3D> planner_session;
  PlannerDispatch3D dispatch{};
  PlannerInputStatus3D planner_input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D planner_progress{SearchProgress3D::kInvalidated};
  PlannerTelemetry3D planner_telemetry{};
  bool planner_invoked{false};
  double search_ms{0.0};
};

struct ProductionRoutePlanningWork3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  std::shared_ptr<const ProductionPlannerSession3D> continuation_session;

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
  std::shared_ptr<const ExecutionRouteSnapshot3D> source_snapshot;
  std::shared_ptr<const ExecutionRouteSnapshot3D> certification_snapshot;
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

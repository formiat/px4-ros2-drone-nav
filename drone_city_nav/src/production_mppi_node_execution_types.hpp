#pragma once

// Included inside namespace drone_city_nav after ProductionMppiPreparedEsdf is defined.

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

struct ProductionIncrementalTopologySearch3D {
  IncrementalTopologicalNavigationObservation3D observation{};
  IncrementalTopologicalPlan3D plan{};
  std::optional<IncrementalTopologicalLatticeDirective3D> directive;
  IncrementalTopologicalPlanCommit3D commit{};
  std::size_t graph_node_count{0U};
  std::size_t graph_edge_count{0U};
  double observation_ms{0.0};
  double planning_ms{0.0};
  double no_executable_route_age_ms{0.0};
};

enum class ProductionIncrementalTopologyRejectionReason3D : std::uint8_t {
  kSegmentEvidenceRawCollision,
  kMaterializedRouteRawCollision,
};

struct ProductionRouteSearchCandidate3D {
  Point3 search_start{};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;
  RouteIntent3D intent{};
  SegmentEvidence3D evidence{};
  RiskAwareLattice3DResult lattice{};
  std::optional<ProductionIncrementalTopologySearch3D> topology;
  Lattice3DStrategicDirective directive{};
};

struct ProductionRouteCandidateSet3D {
  std::vector<ProductionRouteSearchCandidate3D> candidates;
  Vec3 preferred_direction{};
  double search_ms{0.0};
};

struct ProductionRouteExecutionSelection3D {
  std::shared_ptr<const CertifiedRouteSuffix3D> route;
  std::shared_ptr<const ExecutionRouteSnapshot3D> source_snapshot;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_route;
  std::shared_ptr<const VersionedObservedRawWorld3D> lifecycle_observed_raw_world;
  GlobalGuideProjection projection{};
  RouteExecutionStatus3D status{RouteExecutionStatus3D::kNoActiveRoute};
  std::optional<RouteLifecycleEvent3D> lifecycle_event;
  Point3 hold_position{};
  double station_m{0.0};
  bool route_usable{false};
  bool execution_owner_available{false};
  bool pending_activation{false};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
};

#pragma once

#include "drone_city_nav/applied_control_admission.hpp"
#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/cooperative_mppi_adapter.hpp"
#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/execution_route_geometry_3d.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/intercept_guidance.hpp"
#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mission_waypoint_capture_gate.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_nominal_reseed.hpp"
#include "drone_city_nav/mppi_rollout_budget.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/msg/cooperative_maneuver_command.hpp"
#include "drone_city_nav/msg/cooperative_passage_intent.hpp"
#include "drone_city_nav/msg/latest_lidar_obstacle_scan.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/navigation_health.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/msg/radar_track_mode_command.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/msg/target_track_array.hpp"
#include "drone_city_nav/navigation_angular_derivative.hpp"
#include "drone_city_nav/navigation_health_supervisor.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/noncooperative_collision_avoidance.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/offboard_session_admission.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/raw_obstacle_3d_ros.hpp"
#include "drone_city_nav/raw_obstacle_delta.hpp"
#include "drone_city_nav/rolling_route_telemetry_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/route_planning_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/static_route_geometry.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/tracking_objective.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"

namespace drone_city_nav {

enum class ProductionPlanningSearchKind : std::uint8_t {
  kNone,
  kPersistentDStarLite3D,
};

struct ProductionPersistentPlannerTelemetry3D {
  PersistentPlannerStatus3D status{PersistentPlannerStatus3D::kInvalidInput};
  std::uint64_t mission_epoch{0U};
  std::uint64_t planned_on_revision{0U};
  std::uint64_t occupied_fingerprint{0U};
  std::uint64_t search_generation{0U};
  std::uint64_t repair_generation{0U};
  std::size_t expansions{0U};
  std::size_t changed_occupied_voxels{0U};
  std::size_t affected_lattice_states{0U};
  std::size_t repair_lattice_states_processed{0U};
  std::size_t repair_lattice_states_pending{0U};
  std::size_t feasibility_expansions{0U};
  std::size_t records{0U};
  std::size_t open_entries{0U};
  std::size_t shortcut_checks{0U};
  std::size_t shortcuts_applied{0U};
  std::size_t lattice_edge_queries{0U};
  std::size_t raw_edge_validation_checks{0U};
  std::size_t adaptive_edge_queries{0U};
  std::size_t adaptive_edges_in_extracted_path{0U};
  std::size_t maximum_queried_lattice_level{0U};
  std::size_t execution_time_search_expansions{0U};
  std::size_t execution_time_search_records{0U};
  std::size_t execution_time_search_open_entries{0U};
  double path_length_m{0.0};
  double remaining_goal_distance_m{0.0};
  double execution_time_search_objective_s{0.0};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};
  double world_update_ms{0.0};
  double search_ms{0.0};
  bool invoked{false};
  bool executable{false};
  bool search_state_reused{false};
  bool occupied_world_unchanged{false};
  bool incumbent_retained{false};
  bool repair_pending{false};
  bool feasibility_attempted{false};
  bool feasibility_route_found{false};
  bool execution_time_search_complete{false};
  bool search_complete{false};
};

struct ProductionMppiPreparedEsdf;
struct ProductionMppiPlanningTickFinalization;
struct ProductionMppiControllerTick;
struct ProductionMppiControllerTickResult;
struct ProductionRouteActivationSnapshot3D;
struct ProductionRouteActivationResult3D;
struct ProductionRouteMaterialization3D;
struct ProductionMppiExecutionCycle;
struct RouteCompilerConfig3D;
struct ProductionMppiHorizonCommit;
enum class ProductionMppiHoldOwnershipTransition3D : std::uint8_t;
enum class ProductionMppiHorizonCommitStatus : std::uint8_t;

struct ProductionMaterializedRouteProposal3D {
  MaterializedRouteProposal3D identity{};
  ProductionRouteGeometry3D geometry{};
};

struct ProductionMppiPreparedEsdf {
  LocalWorldGeneration local_world_generation{};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  // Raw observation that causally anchors this local ESDF generation.
  std::uint64_t source_raw_revision{0U};
  std::uint64_t source_occupied_fingerprint{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double build_ms{0.0};
  double esdf_x_pass_ms{0.0};
  double esdf_y_pass_ms{0.0};
  double esdf_z_pass_ms{0.0};
  double esdf_finalize_ms{0.0};
  double conversion_ms{0.0};
  double upload_ms{0.0};
  double route_search_ms{0.0};
  double continuation_validation_ms{0.0};
  double route_smoothing_ms{0.0};
  double route_shortcut_validation_ms{0.0};
  double route_corner_validation_ms{0.0};
  double passage_volume_build_ms{0.0};
  std::size_t route_shortcuts_applied{0U};
  std::size_t route_corners_smoothed{0U};
  std::size_t route_shortcut_candidates{0U};
  std::size_t route_parallel_shortcut_candidates{0U};
  std::size_t route_corner_candidates{0U};
  std::size_t route_parallel_corner_candidates{0U};
  bool passage_volume_resource_reused{false};
  double candidate_validation_ms{0.0};
  std::uint64_t route_fingerprint{0U};
  RouteInstanceId3D bound_route_instance_id{};
  mppi::EsdfGrid grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const ObservedOccupancyGrid3D> observed_occupancy;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner;
  ObservedEsdfResource3D observed_esdf_resource{};
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  bool launch_support_resolution_pending{false};
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route;
  std::shared_ptr<const std::vector<RouteSample3D>> route_3d;
  std::shared_ptr<const ProductionRouteGeometry3D> compiled_route_geometry;
  ExecutionRouteGeometryValidation3D route_compilation_validation{};
  std::size_t route_stop_turn_count{0U};
  RouteIntent3D route_intent{};
  SegmentEvidence3D route_segment_evidence{};
  std::shared_ptr<const std::vector<Point2>> route_2d_projection;
  std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
  std::shared_ptr<const std::vector<PassageTraversalEdge>> passage_traversals;
  std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      cooperative_passage_assignments;
  std::shared_ptr<const std::vector<PassageTraversalId>> selected_passage_traversal_ids;
  StaticRouteObjective search_objective{};
  StaticRouteObjective route_objective{};
  std::uint64_t route_generation{0U};
  bool route_reaches_mission_goal{false};
  RouteReleaseReason3D route_release_reason{RouteReleaseReason3D::kNoActiveRoute};
  RouteProgressProjection3D route_projection{};
  ProductionPlanningSearchKind planning_search_kind{
      ProductionPlanningSearchKind::kNone};
  ProductionPersistentPlannerTelemetry3D planner{};
  RouteInstanceId3D planning_search_base_route_instance_id{};
  std::optional<double> planning_search_base_stitch_station_m;
  // Search provenance and activation continuity are separate contracts. A
  // valid ID requires certified overlap with this exact route at activation;
  // an empty ID permits a dynamically validated atomic route handoff.
  RouteInstanceId3D required_splice_base_route_instance_id{};
  Point3 planning_search_start{};
  Point3 planning_search_goal{};
  Point3 planning_candidate_endpoint{};
  Vec3 planning_search_direction{};
  std::size_t planning_candidate_points{0U};
  std::size_t planning_candidate_samples{0U};
  CertifiedRouteReserveStatus3D certified_route_reserve_status{
      CertifiedRouteReserveStatus3D::kInvalid};
  double certified_route_reserve_available_m{0.0};
  double certified_route_reserve_required_m{0.0};
  double certified_route_reserve_shortfall_m{0.0};
  bool static_route_extension_request{false};
  std::uint64_t static_route_extension_base_generation{0U};
  bool static_route_replan_request{false};
  std::uint64_t static_route_replan_base_generation{0U};
  RouteReleaseReason3D static_route_replan_reason{RouteReleaseReason3D::kNone};
  StaticRouteCandidateStatus static_route_candidate_status{
      StaticRouteCandidateStatus::kEmpty};
  StaticRouteActivationStatus static_route_activation_status{
      StaticRouteActivationStatus::kNotAttempted};
  RoutePublicationStatus3D static_route_publication_status{
      RoutePublicationStatus3D::kNotAssessed};
  bool static_route_world_compatible{false};
  bool static_route_generation_assessed{false};
  bool static_route_generation_matches{false};
};

#include "production_mppi_node_execution_types.hpp"

struct ProductionMppiRvizSnapshot {
  std::vector<mppi::State> candidate_horizon;
  std::vector<mppi::State> previous_horizon;
  std::vector<mppi::State> execution_horizon;
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> route;
  std::shared_ptr<const std::vector<PassageTraversalEdge>> passage_traversals;
  std::shared_ptr<const std::vector<PassageTraversalId>> selected_passage_traversal_ids;
};

struct ProductionMppiDiagnosticsSnapshot {
  mppi::MppiTickInput input{};
  mppi::MppiTickResult result{};
  ProductionMppiPreparedEsdf esdf{};
  ProductionMppiStability stability{};
  ProductionMppiPredictionError prediction{};
  MppiLivenessResult liveness{};
  DirectTrackingManeuverUpdate direct_tracking_maneuver{};
  MppiSpeedPolicyResult speed_policy{};
  RouteProgressUpdate3D route_progress{};
  MppiEligibleRolloutUpdate no_eligible_recovery{};
  MissionGoalCaptureResult goal_capture{};
  ProductionMppiExecutionPublication execution{};
  ProductionMppiPlanningState planning_state{ProductionMppiPlanningState::kPlanned};
  std::optional<ProductionMppiRvizSnapshot> rviz;
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::string target_source;
  std::uint64_t tick_sequence{0U};
  std::uint64_t memory_sequence{0U};
  double pose_age_ms{0.0};
  double esdf_age_ms{0.0};
  double observation_age_ms{0.0};
  double control_feedback_age_ms{0.0};
  double route_station_m{0.0};
  double route_remaining_m{0.0};
  double snapshot_ms{0.0};
  double stability_ms{0.0};
  RollingRouteTelemetryObservation3D rolling_route{};
  bool route_projection_valid{false};
  bool local_route_stop_is_terminal{false};
  bool liveness_reseed_requested{false};
  bool pose_predicted{false};
  ProductionMppiPreviousControlSource previous_control_source{
      ProductionMppiPreviousControlSource::kEngineFallback};
  MppiRolloutBudgetDecision rollout_budget{};
  ProductionMppiCooperativeUpdate cooperative{};
  ProductionMppiNonCooperativeUpdate noncooperative{};
  mppi::RiskTier route_required_risk_tier{mppi::RiskTier::kPreferred};
};

[[nodiscard]] const char*
productionPlanningSearchKindName(ProductionPlanningSearchKind kind) noexcept;

class ProductionMppiNode final : public rclcpp::Node {
public:
  explicit ProductionMppiNode(const rclcpp::NodeOptions& options);
  ~ProductionMppiNode() override;

  ProductionMppiNode(const ProductionMppiNode&) = delete;
  ProductionMppiNode& operator=(const ProductionMppiNode&) = delete;
  ProductionMppiNode(ProductionMppiNode&&) = delete;
  ProductionMppiNode& operator=(ProductionMppiNode&&) = delete;

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& message);
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& message);
  void onVehicleLandDetected(const px4_msgs::msg::VehicleLandDetected& message);
  void onNavigationReadiness(const std_msgs::msg::Bool& message);
  void onRawObstacleSnapshot3D(msg::RawObstacleSnapshot3D::ConstSharedPtr message);
  void onRawObstacleDelta3D(msg::RawObstacleDelta3D::ConstSharedPtr message);
  void onLatestLidarObstacleScan(const msg::LatestLidarObstacleScan& message);
  void queueRawWorld3D(const RawObstacleGridUpdate3D& update, double reconstruction_ms);
  void onMemoryStatus(const msg::ObstacleMemoryStatus& message);
  void onAppliedControl(const msg::MppiControlFeedback& message);
  void invalidateAppliedControlWitnessLocked() noexcept;
  void onNavigationObjective(const msg::NavigationObjective& message);
  void onCooperativeManeuverCommand(const msg::CooperativeManeuverCommand& message);
  void publishRadarTrackModeCommand(const ProductionNavigationObjective& objective,
                                    std::uint8_t reason);
  void requestStaticEsdfWork(bool force_refresh = false);
  void completeStaticEsdfWork(bool world_ready) noexcept;
  void publishWorldReadiness(bool ready);
  [[nodiscard]] NavigationHealthAssessment updateNavigationHealth(
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      const ProductionMppiAppliedControl& applied_control,
      const ProductionMppiExecutionHorizonOwner& execution_horizon_owner,
      const std::shared_ptr<const ExecutionRouteSnapshot3D>& execution_snapshot,
      bool world_current, std::int64_t now_ns);
  void publishNavigationHealth(const NavigationHealthAssessment& assessment);
  [[nodiscard]] std::shared_ptr<const ProductionNavigationObjective>
  navigationObjective() const;
  void requestRouteRelease(RouteReleaseReason3D reason,
                           std::uint64_t route_generation = 0U);
  void requestStaticRouteReplan(RouteReleaseReason3D reason,
                                std::uint64_t route_generation);
  void configureOptionalNavigationConstraints();
  void configureStaticRouteGeometry();
  void configureStaticRouteExtension(double maximum_horizontal_acceleration_mps2);
  [[nodiscard]] RouteCompilerConfig3D routeCompilerConfig3D() const noexcept;
  [[nodiscard]] TrackingErrorTubeWorld3D
  trackingErrorTubeWorld3D(const ProductionMppiPreparedEsdf& world) const noexcept;
  static void
  bindStaticRouteRequestToExecution(ProductionMppiPreparedEsdf& request,
                                    const CertifiedRouteSuffix3D& active_route,
                                    const RouteProgressProjection3D& projection);
  void maybeRequestStaticRouteExtensionFromExecution(
      const ProductionMppiPreparedEsdf& esdf,
      const ProductionRouteExecutionSelection3D& route_execution,
      const ProductionMppiNavigation& navigation, std::int64_t now_ns);
  void
  maybeRequestStaticRouteExtension(const ProductionMppiPreparedEsdf& esdf,
                                   const CertifiedRouteSuffix3D& active_route,
                                   const ProductionMppiNavigation& navigation,
                                   const RouteProgressProjection3D& route_projection,
                                   std::int64_t now_ns);
  void
  maybeRequestStaticTrackingWorldRefresh(const ProductionMppiPreparedEsdf& esdf,
                                         const ProductionMppiNavigation& navigation,
                                         const ProductionNavigationObjective& objective,
                                         std::int64_t now_ns);
  void finishStaticRouteExtension(std::uint64_t base_generation,
                                  bool extension_activated = false);
  void finishStaticRouteReplan(std::uint64_t base_generation, bool route_activated);
  void finishStaticRouteSearch(const ProductionMppiPreparedEsdf& world,
                               bool route_activated = false);
  void esdfWorker(std::stop_token stop_token);
  [[nodiscard]] std::optional<std::chrono::steady_clock::time_point>
  processObservedEsdf3D(const ProductionMppiRawWorld3D& raw_world);
  [[nodiscard]] std::optional<ProprioceptiveFreeSpaceSeed3D>
  prepareObservedExecutionEvidence3D(
      const ProductionMppiRawWorld3D& raw_world,
      const ProductionMppiNavigation& navigation,
      const ProductionMppiAppliedControl& applied_control,
      const ProductionMppiExecutionHorizonOwner& execution_horizon_owner);
  void queueLatestObservedWorldForPose(const ProductionMppiNavigation& navigation);
  void routePlanningWorker(std::stop_token stop_token);
  void processRouteSearch3D(const ProductionMppiPreparedEsdf& world,
                            const ProductionMppiNavigation& navigation);
  [[nodiscard]] RouteSegmentCompletionAssessment3D
  assessActiveRouteCompletion3D(const ProductionMppiPreparedEsdf& world,
                                const Point3& position);
  [[nodiscard]] std::uint64_t nextRouteGeneration3D();
  [[nodiscard]] ProductionRouteActivationSnapshot3D captureRouteActivationSnapshot3D();
  [[nodiscard]] ProductionRouteActivationResult3D
  prepareRouteActivation3D(const ProductionMppiPreparedEsdf& search_world,
                           ProductionMppiPreparedEsdf prepared,
                           NavigationWorldCertificate3D planned_world_certificate,
                           StaticRouteCandidateValidation validation,
                           StaticRouteReplacementPolicy replacement_policy,
                           const Point3& mission_goal,
                           std::uint64_t candidate_generation,
                           const ProductionRouteActivationSnapshot3D& snapshot);
  void commitRouteActivation3D(const ProductionMppiPreparedEsdf& search_world,
                               const ProductionRouteActivationSnapshot3D& snapshot,
                               std::uint64_t candidate_generation,
                               ProductionRouteActivationResult3D& result);
  [[nodiscard]] ProductionRouteMaterialization3D materializeRouteCandidate3D(
      const ProductionMppiPreparedEsdf& world,
      const ProductionMppiNavigation& navigation, const Point3& mission_goal,
      const ProductionRouteSearchCandidate3D& candidate,
      std::uint64_t candidate_generation, const CertifiedRouteSuffix3D* active_route,
      const ProductionMppiRawWorld3D* activation_raw_world);
  [[nodiscard]] ProductionRouteCandidateSet3D generateRouteCandidates3D(
      const ProductionMppiPreparedEsdf& world,
      const ProductionMppiNavigation& navigation, const Point3& mission_goal,
      const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
      const CertifiedRouteSuffix3D* active_route);
  void diagnosticsWorker(std::stop_token stop_token);
  void startPlanningTimer();
  void initializeRuntimeInterfaces();
  [[nodiscard]] ProductionRouteExecutionSelection3D resolveRouteExecution3D(
      const ProductionMppiPreparedEsdf& world,
      const ProductionNavigationObjective* objective,
      const ProductionMppiNavigation& navigation,
      const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
      const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
      std::uint64_t minimum_tracking_sample_sequence,
      std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity,
      bool observed_3d_world);
  void configureCooperativeTraffic();
  void createCooperativeTrafficInterfaces(
      const rclcpp::SubscriptionOptions& subscription_options);
  [[nodiscard]] ProductionMppiCooperativeUpdate prepareCooperativeTick(
      std::span<const CooperativePassageAssignment> passage_assignments,
      const ConstrainedRouteObservation& route_observation,
      const std::optional<ProductionMppiCooperativeCommand>& command,
      std::int64_t now_ns, double planned_speed_mps);
  void configureNonCooperativeAvoidance();
  void createNonCooperativeAvoidanceInterface(
      const rclcpp::SubscriptionOptions& subscription_options);
  void onNonCooperativeTracks(const msg::TargetTrackArray& message);
  [[nodiscard]] ProductionMppiNonCooperativeUpdate
  prepareNonCooperativeTick(const mppi::State& ownship,
                            const ProductionMppiNonCooperativeTracks& tracks,
                            std::int64_t now_ns);
  [[nodiscard]] MissionWaypointUpdate updateMissionWaypoint(
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      const ProductionMppiNavigation& navigation,
      const ProductionMppiVehicleStatus& vehicle_status,
      const ProductionMppiAppliedControl& applied_control,
      const ProductionMppiExecutionHorizonOwner& execution_horizon_owner,
      std::uint64_t applied_control_discontinuity_generation,
      bool applied_control_discontinuity_generation_valid,
      bool vehicle_status_epoch_stable, bool goal_capture_latched, std::int64_t now_ns);
  void publishMissionWaypointAcknowledgement(
      const ProductionNavigationObjective& completed_objective,
      const MissionWaypointUpdate& update,
      const ProductionMppiAppliedControl& applied_control,
      const ProductionMppiExecutionHorizonOwner& execution_horizon_owner,
      std::int64_t now_ns);
  void planningTick();
  [[nodiscard]] bool
  worldGenerationAvailableForPlanning(const ProductionMppiPreparedEsdf& world,
                                      std::int64_t now_ns);
  [[nodiscard]] std::optional<mppi::MppiTickResult>
  planOnCapturedWorldGeneration(const ProductionMppiPreparedEsdf& world,
                                const mppi::MppiTickInput& input);
  void finalizePlanningTick(const ProductionMppiPlanningTickFinalization& finalization);
  [[nodiscard]] std::optional<ProductionMppiControllerTickResult>
  runPlanningController(const ProductionMppiControllerTick& tick);
  void processDiagnostics(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void logDiagnosticsEvents(const ProductionMppiDiagnosticsSnapshot& snapshot,
                            const ConstrainedRouteObservation& route_constraint);
  void publishRviz(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot snapshot);
  void recordTickStatistics(const mppi::MppiTickResult& result,
                            ProductionMppiPlanningState planning_state,
                            const ProductionMppiExecutionPublication& execution,
                            bool liveness_reseed_requested,
                            const RollingRouteTelemetryObservation3D& rolling_route);
  void publishSummary();
  [[nodiscard]] ProductionMppiExecutionPublication publishExecutionHorizon(
      const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
      const ProductionMppiPreparedEsdf& esdf,
      const ProductionRouteExecutionSelection3D& route_execution,
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
      const std::shared_ptr<const VersionedLatestLidarEvidence3D>&
          latest_lidar_evidence,
      const OffboardSessionAdmissionState& offboard_session,
      std::int64_t offboard_session_receive_stamp_ns,
      ProductionMppiPlanningState planning_state, std::int64_t now_ns);
  [[nodiscard]] msg::MppiTrajectoryHorizon
  makeExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                       std::int64_t valid_until_ns, ProductionMppiExecutionMode mode,
                       ProductionMppiExecutionReason reason);
  [[nodiscard]] ProductionMppiHorizonCommitStatus
  commitAndPublishExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                                   const msg::MppiTrajectoryHorizon& horizon,
                                   const ProductionMppiHorizonCommit& commit);
  [[nodiscard]] ProductionMppiHorizonCommitStatus commitExecutionSnapshotHorizon(
      const ProductionMppiExecutionCycle& cycle,
      const std::shared_ptr<const ExecutionRouteSnapshot3D>& expected,
      const ExecutionRouteTransitionResult3D& transition,
      const msg::MppiTrajectoryHorizon& horizon,
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      const std::shared_ptr<const ExecutionRouteSnapshot3D>& certification_snapshot,
      const std::shared_ptr<const ExecutionRouteTransitionResult3D>&
          progress_preparation);
  [[nodiscard]] std::optional<mppi::FiniteExecutionPathWorld>
  exactSnapshotValidationWorld(
      const ProductionMppiExecutionCycle& cycle, const CertifiedRouteSuffix3D& route,
      std::optional<mppi::FiniteExecutionPathTerminalBoundary> terminal_boundary,
      const std::shared_ptr<const VersionedObservedRawWorld3D>&
          observed_world_override = nullptr);
  [[nodiscard]] std::optional<mppi::FiniteExecutionPathWorld>
  exactDirectValidationWorld(const ProductionMppiExecutionCycle& cycle,
                             const DirectTrackingFiniteExecution3D& execution);
  [[nodiscard]] std::optional<ProductionMppiExecutionPublication>
  retainSnapshotFinitePath(const ProductionMppiExecutionCycle& cycle,
                           ProductionMppiExecutionReason replacement_failure_reason);
  [[nodiscard]] std::optional<ProductionMppiExecutionPublication>
  retainDirectFinitePath(const ProductionMppiExecutionCycle& cycle,
                         ProductionMppiExecutionReason replacement_failure_reason);
  [[nodiscard]] std::optional<ProductionMppiExecutionPublication>
  retainActiveFinitePath(const ProductionMppiExecutionCycle& cycle,
                         ProductionMppiExecutionReason replacement_failure_reason);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishPositionHold(const ProductionMppiExecutionCycle& cycle,
                      const Point3& hold_position, ProductionMppiExecutionReason reason,
                      ProductionMppiHoldOwnershipTransition3D ownership_transition);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishNoExecutablePathHold(const ProductionMppiExecutionCycle& cycle,
                              ProductionMppiExecutionReason reason);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishExecutionRevocation(ProductionMppiExecutionReason reason, std::int64_t now_ns,
                             bool retire_certified_route = false);
  [[nodiscard]] bool handleRequestedExecutionRevocation(std::int64_t now_ns);
  void publishFailClosedExecutionRevocation(ProductionMppiExecutionReason reason,
                                            std::int64_t now_ns);
  // Input callbacks only enqueue a monotonic request. The planning thread owns
  // the snapshot CAS and ROS publication so epoch-reset handling cannot race a
  // normal execution-owner commit.
  void requestExecutionRevocation(ProductionMppiExecutionReason reason) noexcept;
  [[nodiscard]] ProductionMppiExecutionPublication
  publishExplicitHold(const ProductionMppiExecutionCycle& cycle,
                      const Point3& hold_position,
                      ProductionMppiExecutionReason reason);

  [[nodiscard]] mppi::State
  selectTarget(std::span<const RouteSample3D> route,
               std::span<const mppi::RouteSample3D> mppi_route,
               double current_station_m, double lookahead_m, std::string& target_source,
               double& target_station_m) const;
  [[nodiscard]] ProductionMppiStability
  compareWithPrevious(const mppi::MppiTickResult& result) const;

  double tick_rate_hz_{50.0};
  double rviz_rate_hz_{10.0};
  double diagnostics_info_rate_hz_{5.0};
  double diagnostics_file_rate_hz_{5.0};
  double diagnostics_flush_period_s_{1.0};
  std::size_t diagnostics_error_ring_capacity_{25U};
  double deadline_ms_{20.0};
  double maximum_pose_age_ms_{150.0};
  double maximum_vehicle_status_age_ms_{1000.0};
  double maximum_pose_prediction_age_ms_{1000.0};
  double maximum_esdf_age_ms_{1000.0};
  double stale_esdf_execution_window_ms_{4000.0};
  double maximum_control_feedback_age_ms_{200.0};
  double latest_lidar_obstacle_maximum_age_ms_{250.0};
  double no_static_3d_esdf_update_rate_hz_{1.0};
  LocalObservedEsdfWindow3D no_static_3d_esdf_window_{};
  double no_static_3d_esdf_incremental_maximum_rebuild_ratio_{0.15};
  std::size_t no_static_3d_esdf_full_audit_interval_builds_{120U};
  std::size_t planner_worker_count_{4U};
  MppiRolloutBudgetConfig rollout_budget_config_{};
  double planning_tick_phase_offset_s_{0.0};
  MissionGoalCaptureConfig mission_goal_capture_config_{};
  MissionWaypointSequenceConfig mission_waypoint_sequence_config_{};
  MissionWaypointCaptureGateConfig mission_waypoint_capture_gate_config_{};
  Px4MapFrameTransform px4_map_transform_{};
  Point3 mission_start_{54.0, 54.0, 0.0};
  Point3 mission_goal_{216.0, 378.0, 18.0};
  FlightEnvelopeConfig flight_envelope_config_{};
  double dynamic_objective_replan_distance_m_{5.0};
  double dynamic_objective_replan_period_s_{0.25};
  double tracking_objective_ray_sample_spacing_m_{0.25};
  double tracking_capture_radius_m_{5.0};
  double static_tracking_esdf_refresh_margin_m_{15.0};
  TrackingLineOfSightLifecycle tracking_line_of_sight_lifecycle_{};
  DirectTrackingManeuverLifecycle direct_tracking_maneuver_lifecycle_{};
  bool use_static_map_{true};
  bool cooperative_traffic_enabled_{false};
  bool noncooperative_avoidance_enabled_{false};
  std::string noncooperative_tracks_topic_;
  std::string vehicle_id_;
  float constrained_route_speed_limit_mps_{10.0F};
  double route_constraint_diagnostics_distance_m_{30.0};
  std::string frame_id_{"map"};
  std::filesystem::path diagnostics_output_dir_{"log/mppi"};
  std::int64_t rviz_period_ns_{100000000};
  std::int64_t diagnostics_info_period_ns_{200000000};
  std::int64_t diagnostics_file_period_ns_{200000000};
  std::int64_t last_rviz_stamp_ns_{0};
  std::int64_t last_diagnostics_info_stamp_ns_{0};
  std::int64_t last_diagnostics_file_stamp_ns_{0};
  std::optional<ConstrainedRouteObservation> last_route_constraint_observation_;

  mppi::BenchmarkConfig mppi_config_{};
  ProductionNavigationOptionalConstraints optional_constraints_{};
  NavigationAngularDerivativeConfig navigation_angular_derivative_config_{};
  SweptFootprintConfig physical_footprint_config_{};
  TrackingErrorTubeConfig3D tracking_error_tube_config_{};
  std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      execution_validation_policy_;
  MppiLivenessConfig liveness_config_{};
  MppiSpeedPolicyConfig speed_policy_config_{};
  mppi::FiniteHorizonConfig finite_horizon_config_{};
  double stationary_hold_validity_s_{1.0};
  std::int64_t stationary_hold_validity_ns_{1'000'000'000LL};
  std::int64_t mission_goal_capture_hold_validity_ns_{0};
  RouteTrackingPolicy3D route_tracking_policy_{};
  RouteProgressConfig3D route_progress_config_{};
  bool route_stall_recovery_enabled_{false};
  std::unique_ptr<MppiLivenessSupervisor> liveness_supervisor_;
  std::unique_ptr<NavigationHealthSupervisor> navigation_health_supervisor_;
  MppiNominalReseedTracker nominal_reseed_tracker_{};
  std::unique_ptr<RouteProgressTracker3D> route_progress_tracker_;
  std::unique_ptr<MissionGoalCaptureLatch> mission_goal_capture_latch_;
  std::unique_ptr<MissionWaypointSequence> mission_waypoint_sequence_;
  std::unique_ptr<MissionWaypointCaptureGate> mission_waypoint_capture_gate_;
  PersistentPlannerConfig3D persistent_planner_config_{};
  double route_sampling_step_m_{0.5};
  double route_completion_tolerance_m_{2.0};
  double static_esdf_route_lookahead_m_{180.0};
  RouteEnvelopeConfig route_envelope_config_{};
  ConstrainedRouteControlConfig constrained_route_control_config_{};
  ConstrainedRouteCoordinator constrained_route_coordinator_{};
  PassageTraversalEvidenceTracker passage_traversal_evidence_tracker_{};
  PassageGeometryEvidenceTracker passage_geometry_evidence_tracker_{};
  StaticRouteExtensionConfig static_route_extension_config_{};
  FutureRouteConnectorConfig3D future_route_connector_config_{};
  CertifiedRouteSpliceConfig3D certified_route_splice_config_{};
  StaticRouteSearchRetryConfig static_route_search_retry_config_{};
  StaticRouteGeometryConfig static_route_geometry_config_{};
  PassageVolumeConfig cooperative_passage_volume_config_{};
  CooperativePassageRouteConfig cooperative_passage_route_config_{};
  CooperativePassageTimingConfig cooperative_passage_timing_config_{};
  CooperativePassageYieldConfig cooperative_passage_yield_config_{};
  NonCooperativeAvoidanceConfig noncooperative_avoidance_config_{};
  std::unique_ptr<NonCooperativeCollisionAvoidance> noncooperative_avoidance_;
  std::unique_ptr<BoundedWorkerPool> planning_worker_pool_;
  std::unique_ptr<PersistentDStarLitePlanner3D> persistent_planner_3d_;
  std::unique_ptr<mppi::MppiCudaEngine> engine_;
  std::shared_ptr<const OccupancyGrid3D> static_occupancy_3d_;
  std::optional<FreeSpaceTopology3D> static_free_space_topology_3d_;
  std::optional<StaticEsdfCache> static_esdf_cache_;
  std::shared_ptr<const std::vector<PassageTraversalEdge>> static_portal_edges_;
  std::shared_ptr<const std::vector<float>> static_esdf_3d_;
  mppi::EsdfGrid static_esdf_grid_{};
  bool static_esdf_uploaded_{false};
  std::mutex static_route_extension_mutex_;
  bool static_route_extension_request_in_flight_{false};
  std::uint64_t static_route_extension_in_flight_generation_{0U};
  std::uint64_t static_route_extension_last_request_generation_{0U};
  double static_route_extension_last_request_station_m_{0.0};
  std::int64_t static_route_extension_last_request_stamp_ns_{0};
  StaticRoutePlanningLatencyTracker static_route_planning_latency_tracker_{};
  StaticRouteDeferredReplanLatch static_route_deferred_replan_latch_{};
  StaticRouteReplanGate static_route_replan_gate_{};
  StaticRouteFailedSearchLatch static_route_failed_search_latch_{};
  StaticRouteRoiRefreshLifecycle static_roi_refresh_lifecycle_{};

  mutable std::mutex input_mutex_;
  ProductionMppiNavigation navigation_{};
  bool navigation_revision_exhausted_{false};
  // Cleared only by node restart. Safe recovery needs one coordinated
  // planner/offboard/world transform handoff, not a callback-local correction.
  bool navigation_frame_reset_unresolved_{false};
  ProductionMppiVehicleStatus vehicle_status_{};
  Px4TimestampEpochAdmissionState vehicle_status_timestamp_admission_{};
  bool vehicle_status_epoch_probation_{false};
  bool vehicle_status_revision_exhausted_{false};
  NavigationAngularDerivativeEstimator navigation_angular_derivative_estimator_{};
  ProductionMppiAppliedControl applied_control_{};
  std::uint64_t applied_control_discontinuity_generation_{0U};
  bool applied_control_discontinuity_generation_exhausted_{false};
  ProductionMppiExecutionHorizonOwner execution_horizon_owner_{};
  ExecutionHorizonWitnessState applied_control_admission_state_{};
  OffboardSessionAdmissionState offboard_session_admission_{};
  std::int64_t offboard_session_receive_stamp_ns_{0};
  std::optional<ProductionMppiCooperativeCommand> cooperative_command_;
  ProductionMppiNonCooperativeTracks noncooperative_tracks_{};
  LatestObservationTracker latest_observation_tracker_{};
  ProductionMppiPendingRawWorldUpdate pending_raw_world_update_{};
  bool raw_world_identity_conflicted_{false};
  std::atomic<std::shared_ptr<const ProductionNavigationObjective>>
      navigation_objective_;
  std::atomic<std::uint64_t> minimum_tracking_route_mission_epoch_{0U};
  std::atomic<std::uint64_t> minimum_tracking_route_sample_sequence_{0U};
  std::mutex objective_replan_mutex_;
  Point3 objective_replan_anchor_{};
  std::int64_t objective_replan_stamp_ns_{0};

  std::mutex raw_queue_mutex_;
  std::condition_variable_any raw_queue_condition_;
  // The committed immutable payload remains authoritative while an announced
  // successor is still pending its status/payload join.
  LatestWinsDeferredScheduler<std::shared_ptr<const ProductionMppiRawWorld3D>>
      raw_world_scheduler_3d_{};
  std::atomic<std::shared_ptr<const ProductionMppiRawWorld3D>> latest_raw_world_3d_;
  std::atomic<std::uint64_t> observed_route_blocked_raw_revision_{0U};
  std::atomic<std::uint64_t> observed_route_replan_dispatched_raw_revision_{0U};
  std::mutex execution_evidence_commit_mutex_;
  // Latest-lidar admission is independent from raw-world reconstruction. Active
  // execution publication locks both domains to validate one coherent boundary.
  std::mutex latest_lidar_evidence_commit_mutex_;
  LatestLidarEvidenceAdmissionState3D latest_lidar_evidence_admission_state_{};
  std::atomic_bool latest_lidar_evidence_identity_conflicted_{false};
  std::atomic<std::shared_ptr<const VersionedLatestLidarEvidence3D>>
      latest_lidar_evidence_;
  std::mutex raw_reconstruction_mutex_;
  RawObstacleDeltaAccumulator3D raw_delta_accumulator_3d_;
  std::chrono::steady_clock::time_point no_static_3d_esdf_last_build_time_{};
  LocalWorldGenerationCounter local_world_generation_counter_{};
  bool launch_support_evaluated_{false};
  std::optional<ProprioceptiveFreeSpaceSeed3D> launch_support_seed_;
  std::optional<LaunchSupportContact3D> launch_support_contact_;
  std::atomic_bool vehicle_land_contact_received_{false};
  std::atomic_bool vehicle_land_contact_{false};
  std::atomic_bool launch_support_confirmed_by_land_detector_{false};
  std::atomic<std::uint64_t> no_static_raw_updates_{0U};
  std::atomic<std::uint64_t> no_static_esdf_builds_{0U};
  std::atomic<std::uint64_t> no_static_esdf_throttled_updates_{0U};
  ObservedEsdf3DRuntimeCounters observed_esdf_3d_counters_{};
  std::atomic<std::uint64_t> rejected_lidar_obstacle_scans_{0U};
  bool pending_static_esdf_work_{false};
  bool static_esdf_work_in_progress_{false};
  std::atomic_bool vehicle_navigation_ready_{false};
  std::atomic_bool world_ready_{false};
  std::atomic<std::uint64_t> dropped_raw_snapshots_{0U};
  std::jthread esdf_worker_;
  std::mutex route_planning_queue_mutex_;
  std::condition_variable_any route_planning_queue_condition_;
  std::shared_ptr<const ProductionMppiPreparedEsdf> pending_route_planning_world_;
  std::atomic<std::uint64_t> dropped_route_planning_worlds_{0U};
  std::jthread route_planning_worker_;

  // Linearizes the active GPU ESDF with its exact immutable CPU world.
  mutable std::mutex world_generation_publication_mutex_;
  mutable std::mutex esdf_state_mutex_;
  std::optional<ProductionMppiPreparedEsdf> prepared_esdf_;
  std::atomic<std::uint64_t> superseded_world_generation_ticks_{0U};
  std::atomic<std::uint64_t> rejected_world_generation_publications_{0U};

  std::optional<mppi::MppiTickResult> previous_result_;
  ExecutionRouteSnapshotStore3D execution_route_store_{};
  PendingCertifiedRouteMailbox3D pending_certified_route_mailbox_{};
  std::atomic<std::uint64_t> pending_certified_route_sequence_{0U};
  std::atomic<std::uint64_t> requested_execution_revocation_{0U};
  std::uint64_t handled_execution_revocation_request_{0U};
  std::optional<mppi::State> previous_predicted_next_state_;
  std::int64_t previous_prediction_stamp_ns_{0};
  ProductionMppiPredictionError latest_prediction_error_{};
  std::uint64_t execution_input_capture_sequence_{0U};
  std::uint64_t tick_sequence_{0U};
  std::uint64_t execution_horizon_sequence_{0U};
  std::uint64_t execution_horizon_producer_instance_id_{0U};
  std::uint64_t navigation_health_producer_instance_id_{0U};
  std::uint64_t navigation_health_sequence_{0U};
  std::atomic<std::uint64_t> navigation_recovery_sequence_{0U};
  std::optional<NavigationHealthAssessment> last_navigation_health_assessment_;
  std::uint64_t mission_waypoint_acknowledgement_sequence_{0U};
  bool mission_goal_capture_attempt_invalidated_{false};
  std::uint64_t completed_ticks_{0U};
  std::uint64_t deadline_misses_{0U};
  std::uint64_t altitude_envelope_violation_horizons_{0U};
  std::uint64_t raw_collision_horizons_{0U};
  std::uint64_t solid_collision_horizons_{0U};
  std::uint64_t post_update_contract_violations_{0U};
  std::uint64_t no_progress_horizons_{0U};
  std::uint64_t liveness_reseeds_{0U};
  std::uint64_t mission_goal_position_hold_ticks_{0U};
  std::uint64_t no_executable_route_hold_ticks_{0U};
  std::uint64_t no_executable_horizon_hold_ticks_{0U};
  std::uint64_t terminal_rest_horizon_ticks_{0U};
  std::uint64_t finite_path_validation_backoff_ticks_{0U};
  std::uint64_t latest_lidar_path_validation_backoff_ticks_{0U};
  std::uint64_t retained_previous_finite_path_ticks_{0U};
  std::uint64_t arrival_control_total_{0U};
  std::uint64_t arrival_shaping_attempt_total_{0U};
  std::uint64_t full_rollout_ticks_{0U};
  std::uint64_t reduced_rollout_ticks_{0U};
  std::uint64_t active_rollout_total_{0U};
  RollingRouteTelemetry3D rolling_route_telemetry_{};
  std::vector<double> runtime_samples_ms_;
  std::int64_t last_summary_stamp_ns_{0};
  mutable std::mutex statistics_mutex_;
  std::ofstream diagnostics_stream_;
  std::ofstream diagnostics_error_stream_;
  std::deque<std::string> diagnostics_error_ring_;
  std::chrono::steady_clock::time_point last_diagnostics_flush_time_{};
  bool diagnostics_error_active_{false};
  LatestValueMailbox<ProductionMppiDiagnosticsSnapshot> diagnostics_mailbox_;
  std::atomic<std::uint64_t> dropped_diagnostics_snapshots_{0U};
  std::jthread diagnostics_worker_;

  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_evidence_callback_group_;
  rclcpp::CallbackGroup::SharedPtr world_input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
      vehicle_land_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_readiness_sub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot3D>::SharedPtr raw_snapshot_3d_sub_;
  rclcpp::Subscription<msg::RawObstacleDelta3D>::SharedPtr raw_delta_3d_sub_;
  rclcpp::Subscription<msg::LatestLidarObstacleScan>::SharedPtr
      latest_lidar_obstacle_scan_sub_;
  rclcpp::Subscription<msg::ObstacleMemoryStatus>::SharedPtr memory_status_sub_;
  rclcpp::Subscription<msg::MppiControlFeedback>::SharedPtr applied_control_sub_;
  rclcpp::Subscription<msg::NavigationObjective>::SharedPtr navigation_objective_sub_;
  rclcpp::Subscription<msg::CooperativeManeuverCommand>::SharedPtr
      cooperative_command_sub_;
  rclcpp::Subscription<msg::TargetTrackArray>::SharedPtr noncooperative_tracks_sub_;
  rclcpp::Publisher<msg::RadarTrackModeCommand>::SharedPtr
      radar_track_mode_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr world_readiness_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_health_pub_;
  rclcpp::Publisher<msg::NavigationHealth>::SharedPtr navigation_health_pub_;
  rclcpp::Publisher<msg::MppiTrajectoryHorizon>::SharedPtr execution_horizon_pub_;
  rclcpp::Publisher<msg::MissionWaypointAcknowledgement>::SharedPtr
      mission_waypoint_acknowledgement_pub_;
  rclcpp::Publisher<msg::CooperativePassageIntent>::SharedPtr
      cooperative_passage_state_pub_;
  rclcpp::TimerBase::SharedPtr planning_start_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr planner_health_timer_;
};

} // namespace drone_city_nav

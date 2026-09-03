#pragma once

#include "drone_city_nav/applied_control_admission.hpp"
#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/cooperative_mppi_adapter.hpp"
#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"
#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/intercept_guidance.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mission_waypoint_capture_gate.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
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
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/raw_obstacle_delta.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/route_planning_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/route_successor_improvement_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/static_route_geometry.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/tracking_objective.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/world_generation.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

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
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "execution_evidence_boundary_3d.hpp"
#include "production_mppi_config.hpp"
#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_execution_types.hpp"
#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"
#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_lifecycle_coordinator_3d.hpp"

namespace drone_city_nav {

struct ProductionMppiPlanningTickFinalization;
struct ProductionMppiControllerTick;
struct MppiControllerResult3D;
struct ProductionMppiDiagnosticsSnapshot;
class MppiController3D;
class PlanningCycleCoordinator3D;
class ExecutionHorizonAssembler3D;
struct HorizonCandidate3D;
struct ProductionMppiExecutionCycle;
struct ObservedWorldBuildRequest3D;
struct ObservedWorldEvidenceChange3D;
struct ObservedWorldUpdate3D;
struct StaticWorldBuildRequest3D;
struct StaticWorldCommitContext3D;
struct StaticWorldRefreshRequest3D;
struct StaticWorldResources3D;
struct StaticWorldUpdate3D;
enum class ProductionMppiHorizonCommitStatus : std::uint8_t;
class NavigationDiagnosticsSink;
class WorldPipeline3D;
class RawWorldIngressRos3D;
struct RawObstacleGridUpdate3D;

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
  void recordAppliedControlDiscontinuityLocked() noexcept;
  void invalidateAppliedControlWitnessLocked() noexcept;
  void onNavigationObjective(const msg::NavigationObjective& message);
  void onCooperativeManeuverCommand(const msg::CooperativeManeuverCommand& message);
  void publishRadarTrackModeCommand(const ProductionNavigationObjective& objective,
                                    RadarCadenceReason reason);
  void requestStaticEsdfWork();
  void markStaticWorldReady() noexcept;
  void publishWorldReadiness(bool ready);
  [[nodiscard]] NavigationHealthAssessment updateNavigationHealth(
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority,
      bool world_current, std::int64_t now_ns);
  void publishNavigationHealth(const NavigationHealthAssessment& assessment);
  [[nodiscard]] std::shared_ptr<const ProductionNavigationObjectiveState>
  navigationObjectiveState() const;
  [[nodiscard]] std::shared_ptr<const ProductionNavigationObjective>
  navigationObjective() const;
  void requestRouteRelease(RouteReleaseReason3D reason,
                           std::uint64_t route_generation = 0U);
  void handlePhysicalTrajectoryCollision(
      std::uint64_t route_generation,
      const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world,
      std::string_view source, ProductionMppiPhysicalTrajectoryAuthority authority);
  void requestStaticRouteReplan(RouteReleaseReason3D reason,
                                std::uint64_t route_generation);
  [[nodiscard]] bool
  requestInitialRouteSearch3D(const std::shared_ptr<const WorldSnapshot3D>& world,
                              const ProductionWorldBuildTelemetry3D& world_telemetry,
                              bool& replaced_pending);
  void maybeRequestStaticRouteExtensionFromExecution(
      const std::shared_ptr<const WorldSnapshot3D>& world,
      const ProductionWorldBuildTelemetry3D& world_build,
      const ProductionRouteExecutionSelection3D& route_execution,
      const ProductionMppiNavigation& navigation, std::int64_t now_ns);
  void maybeRequestStaticRouteExtension(
      const std::shared_ptr<const WorldSnapshot3D>& world,
      const ProductionWorldBuildTelemetry3D& world_build,
      const std::shared_ptr<const CertifiedRouteSuffix3D>& active_route,
      const ProductionMppiNavigation& navigation,
      const RouteProgressProjection3D& route_projection, std::int64_t now_ns);
  void maybeRequestStaticTrackingWorldRefresh(
      const std::shared_ptr<const WorldSnapshot3D>& world,
      const ProductionMppiNavigation& navigation,
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      std::int64_t now_ns);
  void logRouteLifecycleReplanOutcome3D(const RouteLifecycleReplanOutcome3D& outcome);
  [[nodiscard]] StaticWorldBuildRequest3D
  makeStaticWorldBuildRequest3D(const StaticWorldRefreshRequest3D& refresh);
  [[nodiscard]] StaticWorldCommitContext3D makeStaticWorldCommitContext3D();
  void handleStaticWorldUpdate3D(const StaticWorldUpdate3D& update);
  [[nodiscard]] std::optional<ObservedWorldBuildRequest3D>
  makeObservedWorldBuildRequest3D(
      std::shared_ptr<const ProductionMppiRawWorld3D> raw_world);
  void handleObservedWorldEvidenceChange3D(const ObservedWorldEvidenceChange3D& change);
  void handleObservedWorldUpdate3D(const ObservedWorldUpdate3D& update);
  [[nodiscard]] std::optional<ProprioceptiveFreeSpaceSeed3D>
  prepareObservedExecutionEvidence3D(
      const ProductionMppiRawWorld3D& raw_world,
      const ProductionMppiNavigation& navigation,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority);
  void queueLatestObservedWorldForPose(const ProductionMppiNavigation& navigation);
  void processRouteSearch3D(RouteLifecycleUpdate3D update);
  void handleRoutePlanningRejection3D(const RoutePlanningRejection3D& rejection);
  [[nodiscard]] RouteSegmentCompletionAssessment3D
  assessActiveRouteCompletion3D(const Point3& position);
  [[nodiscard]] ProductionRouteActivationSnapshot3D captureRouteActivationSnapshot3D();
  void startPlanningTimer();
  void initializeRuntimeInterfaces(StaticWorldResources3D&& static_world_resources);
  void createCooperativeTrafficInterfaces(
      const rclcpp::SubscriptionOptions& subscription_options);
  void createNonCooperativeAvoidanceInterface(
      const rclcpp::SubscriptionOptions& subscription_options);
  void onNonCooperativeTracks(const msg::TargetTrackArray& message);
  void logNonCooperativeUpdate(const ProductionMppiNonCooperativeUpdate& update);
  [[nodiscard]] MissionWaypointUpdate updateMissionWaypoint(
      const std::shared_ptr<const ProductionNavigationObjectiveState>& objective_state,
      const ProductionMppiNavigation& navigation,
      const ProductionMppiVehicleStatus& vehicle_status,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority,
      std::uint64_t applied_control_discontinuity_generation,
      bool applied_control_discontinuity_generation_valid,
      bool vehicle_status_epoch_stable, bool goal_capture_latched, std::int64_t now_ns);
  void publishMissionWaypointAcknowledgement(
      const ProductionNavigationObjective& completed_objective,
      const MissionWaypointUpdate& update,
      const AppliedControlEvidence3D& applied_control,
      const ExecutionOwnerIdentity3D& execution_horizon_owner, std::int64_t now_ns);
  void planningTick();
  [[nodiscard]] bool worldGenerationAvailableForPlanning(const WorldSnapshot3D& world,
                                                         std::int64_t now_ns);
  void finalizePlanningTick(const ProductionMppiPlanningTickFinalization& finalization);
  [[nodiscard]] std::optional<MppiControllerResult3D>
  runPlanningController(ProductionMppiControllerTick tick);
  void processDiagnostics(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void logDiagnosticsEvents(const ProductionMppiDiagnosticsSnapshot& snapshot,
                            const ConstrainedRouteObservation& route_constraint);
  void publishRviz(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void publishSummary();
  [[nodiscard]] ProductionMppiExecutionPublication publishExecutionHorizon(
      const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
      const WorldSnapshot3D& world,
      const ProductionRouteExecutionSelection3D& route_execution,
      const std::shared_ptr<const ProductionNavigationObjective>& objective,
      const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
      const std::shared_ptr<const VersionedLatestLidarEvidence3D>&
          latest_lidar_evidence,
      const OffboardSessionAdmissionState& offboard_session,
      std::int64_t offboard_session_receive_stamp_ns,
      ProductionMppiPlanningState planning_state, std::int64_t now_ns);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishPreparedExecutionCycle(const ProductionMppiExecutionCycle& cycle);
  void logPhysicalRejectionCells(const ProductionMppiExecutionCycle& cycle,
                                 const HorizonCandidate3D& candidate);
  void releaseRouteRejectedByCertificationWhileStationary(
      const ProductionMppiExecutionCycle& cycle, const HorizonCandidate3D& candidate);
  [[nodiscard]] msg::MppiTrajectoryHorizon
  makeExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                       std::int64_t valid_until_ns, ProductionMppiExecutionMode mode,
                       ProductionMppiExecutionReason reason);
  [[nodiscard]] ProductionMppiHorizonCommitStatus
  commitAndPublishExecutionHorizon(const ProductionMppiExecutionCycle& cycle,
                                   const msg::MppiTrajectoryHorizon& horizon,
                                   ExecutionHorizonLeaseCandidate3D candidate);
  [[nodiscard]] ProductionMppiHorizonCommitStatus commitExecutionSnapshotHorizon(
      const ProductionMppiExecutionCycle& cycle,
      const std::shared_ptr<const ExecutionPlan3D>& expected,
      const ExecutionRouteTransitionResult3D& transition,
      const msg::MppiTrajectoryHorizon& horizon,
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      const std::shared_ptr<const ExecutionPlan3D>& certification_snapshot,
      const std::shared_ptr<const ExecutionRouteTransitionResult3D>&
          progress_preparation,
      const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority);
  // Retains the active finite path with a braking tail. When retention fails
  // because the active trajectory itself collides with raw or lidar evidence,
  // physically_rejected reports it so the caller fails closed instead of
  // letting the resident owner run on.
  [[nodiscard]] std::optional<ProductionMppiExecutionPublication>
  retainActiveFinitePath(const ProductionMppiExecutionCycle& cycle,
                         ProductionMppiExecutionReason replacement_failure_reason,
                         bool* physically_rejected = nullptr);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishPositionHold(const ProductionMppiExecutionCycle& cycle,
                      const Point3& hold_position, ProductionMppiExecutionReason reason,
                      ExecutionHoldIntent3D intent);
  // physical_candidate_rejection: the replacement candidate was rejected by
  // raw or lidar collision evidence ahead of the vehicle. A resident owner is
  // never continued past physical evidence; the execution is revoked so the
  // offboard holds immediately.
  [[nodiscard]] ProductionMppiExecutionPublication
  publishNoExecutablePathHold(const ProductionMppiExecutionCycle& cycle,
                              ProductionMppiExecutionReason reason,
                              bool physical_candidate_rejection = false);
  [[nodiscard]] ProductionMppiExecutionPublication
  publishExecutionRevocation(ProductionMppiExecutionReason reason, std::int64_t now_ns,
                             bool physical_route_invalidation = false);
  // Retires an owner whose lease has expired without a replacement: the
  // offboard is already in its local terminal hold, so the plan moves to the
  // revoked state that a stationary capture rearm or a fresh activation takes
  // over. Nothing is published; the wire has no authority to revoke.
  // retire_route: the route itself is finished (the mission goal capture is
  // latched), so the plan is revoked outright instead of suspended.
  void expireResidentOwner(std::int64_t now_ns, bool retire_route);
  // Revokes the resident goal hold once its waypoint is acknowledged, so the
  // successor leg activates from an empty execution base.
  void retireGoalHoldForSuccessorLeg();
  // True when the resident hold lease ends within two planning ticks (nominal
  // or measured), the point at which it must be re-leased to keep authority.
  [[nodiscard]] bool residentHoldLeaseNearExpiry(std::int64_t now_ns) const;
  [[nodiscard]] ProductionMppiExecutionPublication residentOwnerContinuation(
      ProductionMppiExecutionReason replacement_failure_reason, std::int64_t now_ns,
      const ProductionMppiExecutionPublication& unpublished_revocation,
      bool retire_route_on_expiry);
  [[nodiscard]] bool handleRequestedExecutionRevocation(std::int64_t now_ns);
  void publishFailClosedExecutionRevocation(ProductionMppiExecutionReason reason,
                                            std::int64_t now_ns);
  // When the optional nonphysical revocation policy is enabled, input callbacks
  // only enqueue a monotonic request. The planning thread owns the snapshot CAS
  // and ROS publication so epoch-reset handling cannot race a normal owner
  // commit. With the default policy, these advisory requests are ignored.
  void requestExecutionRevocation(ProductionMppiExecutionReason reason) noexcept;
  [[nodiscard]] ProductionMppiExecutionPublication
  publishExplicitHold(const ProductionMppiExecutionCycle& cycle,
                      const Point3& hold_position,
                      ProductionMppiExecutionReason reason);

  [[nodiscard]] ProductionMppiStability
  compareWithPrevious(const mppi::MppiTickResult& result) const;

  const ProductionMppiConfig config_;
  Point3 mission_goal_{216.0, 378.0, 18.0};
  TrackingLineOfSightLifecycle tracking_line_of_sight_lifecycle_{};
  std::int64_t last_rviz_stamp_ns_{0};
  std::int64_t last_diagnostics_info_stamp_ns_{0};
  std::optional<ConstrainedRouteObservation> last_route_constraint_observation_;

  std::unique_ptr<NavigationHealthSupervisor> navigation_health_supervisor_;
  std::unique_ptr<MissionWaypointSequence> mission_waypoint_sequence_;
  std::unique_ptr<MissionWaypointCaptureGate> mission_waypoint_capture_gate_;
  std::uint64_t mission_capture_continuity_breaks_{0U};
  std::int64_t last_planning_tick_entry_ns_{0};
  std::int64_t last_planning_tick_period_ns_{0};
  // Owner identity of the goal hold observed by the previous capture update,
  // so feedback for the lease a renewal just superseded still counts.
  std::uint64_t last_capture_hold_id_{0U};
  std::uint64_t last_capture_hold_sequence_{0U};
  const char* mission_capture_last_break_reason_{"none"};
  std::unique_ptr<BoundedWorkerPool> planning_worker_pool_;
  std::unique_ptr<BoundedWorkerPool> world_worker_pool_;
  // Wall time of the last publication sub-phases on the planning tick thread.
  std::chrono::steady_clock::time_point latest_publication_started_{};
  double latest_horizon_assembly_ms_{0.0};
  double latest_horizon_commit_ms_{0.0};
  double latest_horizon_wire_ms_{0.0};
  std::unique_ptr<RouteLifecycleCoordinator3D> route_lifecycle_coordinator_;
  std::unique_ptr<PlanningCycleCoordinator3D> planning_cycle_coordinator_;
  std::unique_ptr<ExecutionHorizonAssembler3D> execution_horizon_assembler_;
  // Resident route generation already released after stationary candidates
  // were rejected by the route contract; one release per generation.
  std::uint64_t certification_release_route_generation_{0U};
  std::unique_ptr<MppiController3D> mppi_controller_;

  // Every writer that can change what a publication is allowed to claim enters
  // through this boundary, so the lock order lives in one ROS-free type.
  mutable ExecutionEvidenceBoundary3D evidence_boundary_;
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
  std::uint64_t applied_control_discontinuity_generation_{0U};
  bool applied_control_discontinuity_generation_exhausted_{false};
  ExecutionHorizonWitnessState applied_control_admission_state_{};
  OffboardSessionAdmissionState offboard_session_admission_{};
  std::int64_t offboard_session_receive_stamp_ns_{0};
  std::optional<ProductionMppiCooperativeCommand> cooperative_command_;
  ProductionMppiNonCooperativeTracks noncooperative_tracks_{};
  // Published as one immutable state so every reader observes the objective and
  // the tracking-route requirement produced by the same transition.
  std::atomic<std::shared_ptr<const ProductionNavigationObjectiveState>>
      navigation_objective_state_;
  Point3 objective_replan_anchor_{};
  std::int64_t objective_replan_stamp_ns_{0};

  std::atomic<std::uint64_t> observed_route_blocked_raw_revision_{0U};
  std::atomic<std::uint64_t> observed_route_replan_dispatched_raw_revision_{0U};
  std::atomic<std::uint64_t> physical_trajectory_replan_route_generation_{0U};
  LatestLidarEvidenceAdmissionState3D latest_lidar_evidence_admission_state_{};
  std::atomic_bool latest_lidar_evidence_identity_conflicted_{false};
  std::atomic<std::shared_ptr<const VersionedLatestLidarEvidence3D>>
      latest_lidar_evidence_;
  bool launch_support_evaluated_{false};
  std::optional<ProprioceptiveFreeSpaceSeed3D> launch_support_seed_;
  std::optional<LaunchSupportContact3D> launch_support_contact_;
  std::atomic_bool vehicle_land_contact_received_{false};
  std::atomic_bool vehicle_land_contact_{false};
  std::atomic_bool launch_support_confirmed_by_land_detector_{false};
  std::atomic<std::uint64_t> rejected_lidar_obstacle_scans_{0U};
  std::atomic_bool vehicle_navigation_ready_{false};
  std::atomic_bool world_ready_{false};
  std::unique_ptr<WorldPipeline3D> world_pipeline_;
  std::unique_ptr<RawWorldIngressRos3D> raw_world_ingress_;
  std::atomic<std::shared_ptr<const ProductionRouteActivationResult3D>>
      latest_route_pipeline_event_;

  std::optional<mppi::MppiTickResult> previous_result_;
  ExecutionSupervisor3D execution_supervisor_{};
  std::atomic<std::uint64_t> requested_execution_revocation_{0U};
  std::uint64_t handled_execution_revocation_request_{0U};
  std::optional<mppi::State> previous_predicted_next_state_;
  std::int64_t previous_prediction_stamp_ns_{0};
  ProductionMppiPredictionError latest_prediction_error_{};
  std::uint64_t execution_input_capture_sequence_{0U};
  std::uint64_t tick_sequence_{0U};
  std::uint64_t execution_horizon_sequence_{0U};
  ProductionMppiHorizonPublicationRecord latest_horizon_publication_{};
  // Newest planned-horizon identity the current offboard process reported.
  // Guarded by the input scope; replaced by every accepted horizon feedback.
  ProductionMppiHorizonAcknowledgement latest_horizon_acknowledgement_{};
  std::atomic<std::uint64_t> horizon_publications_{0U};
  std::atomic<std::uint64_t> horizon_commit_rejections_{0U};
  std::atomic<std::uint64_t> horizon_supersession_deferrals_{0U};
  std::atomic<std::uint64_t> horizon_supersession_grace_replacements_{0U};
  std::atomic<std::uint64_t> resident_owner_continuation_ticks_{0U};
  std::uint64_t execution_horizon_producer_instance_id_{0U};
  std::uint64_t navigation_health_producer_instance_id_{0U};
  std::uint64_t navigation_health_sequence_{0U};
  std::optional<NavigationHealthAssessment> last_navigation_health_assessment_;
  std::uint64_t mission_waypoint_acknowledgement_sequence_{0U};
  std::int64_t last_summary_stamp_ns_{0};
  std::unique_ptr<NavigationDiagnosticsSink> diagnostics_sink_;

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

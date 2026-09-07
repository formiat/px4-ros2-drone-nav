#pragma once

#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_rollout_budget.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/noncooperative_collision_avoidance.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mppi_controller_3d.hpp"
#include "production_mppi_node_execution_types.hpp"
#include "production_mppi_node_types.hpp"
#include "route_execution_selector_3d.hpp"

namespace drone_city_nav {

struct PlanningCycleCoordinatorConfig3D {
  RouteExecutionSelectorConfig3D route_execution{};
  MppiLivenessConfig liveness{};
  std::optional<RouteProgressConfig3D> route_progress;
  MissionGoalCaptureConfig goal_capture{};
  DirectTrackingManeuverConfig direct_tracking{};
  RouteEnvelopeConfig route_envelope{};
  ConstrainedRouteControlConfig constrained_route_control{};
  MppiSpeedPolicyConfig speed_policy{};
  MppiRolloutBudgetConfig rollout_budget{};
  CooperativePassageTimingConfig cooperative_timing{};
  CooperativePassageYieldConfig cooperative_yield{};
  NonCooperativeAvoidanceConfig noncooperative_avoidance{};
  FlightEnvelopeConfig flight_envelope{};
  mppi::DynamicsConfig dynamics{};
  // The body the executed-horizon clearance is measured for, and the clearance
  // below which a sample on that horizon constrains the reference speed.
  SweptFootprintConfig physical_footprint{};
  double executed_horizon_constraint_clearance_m{1.0};
  std::string vehicle_id;
  std::size_t horizon_steps{0U};
  double tracking_capture_radius_m{0.0};
  double route_constraint_diagnostics_distance_m{0.0};
  bool cooperative_traffic_enabled{false};
  bool noncooperative_avoidance_enabled{false};
  bool route_progress_replan_enabled{false};
  bool route_cross_track_constraints_enabled{false};
  bool stochastic_trajectory_selection_enabled{false};
};

struct PlanningCycleRequest3D {
  const WorldSnapshot3D* world{nullptr};
  const ProductionNavigationObjective* objective{nullptr};
  const mppi::MppiTickResult* previous_result{nullptr};
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::optional<ProductionMppiCooperativeCommand> cooperative_command;
  ProductionMppiNonCooperativeTracks noncooperative_tracks{};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
  Point3 mission_goal{};
  std::chrono::steady_clock::time_point tick_started{};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::uint64_t physically_invalidated_through_generation{0U};
  std::uint64_t effective_route_generation{0U};
  std::uint64_t line_of_sight_generation{0U};
  std::uint64_t world_revision{0U};
  std::int64_t now_ns{0};
  double observation_age_ms{0.0};
  bool control_feedback_fresh{false};
  bool terminal_hold_enabled{true};
  bool direct_tracking_interception{false};
  bool use_static_map{false};
  bool observed_3d_world{false};

  [[nodiscard]] bool valid() const noexcept;
};

struct PassageGeometryProximity3D {
  PassageTraversalId passage_traversal_id;
  double entry_distance_m{0.0};
  double projection_station_m{0.0};
  double projection_cross_track_m{0.0};
  double minimum_clearance_m{0.0};
  bool projection_valid{false};
  bool within_corridor{false};
};

struct PlanningCycleEffects3D {
  std::vector<RouteExecutionSelectorEffect3D> route_execution;
  std::vector<PassageTraversalEvidenceEvent> passage_traversal_events;
  std::vector<PassageGeometryEvidenceEvent> passage_geometry_events;
  std::optional<PassageGeometryProximity3D> passage_geometry_proximity;
  bool request_pending_successor{false};
  bool request_stalled_route_release{false};
  bool request_static_tracking_world_refresh{false};
  bool request_route_extension{false};
};

struct PlanningRouteDecision3D {
  ProductionRouteExecutionSelection3D execution{};
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> controller_route;
  std::shared_ptr<const std::vector<PassageTraversalId>> selected_passage_traversal_ids;
  RouteProgressProjection3D projection{};
  RouteExecutionStatus3D execution_status{RouteExecutionStatus3D::kNoActiveRoute};
  std::uint64_t generation{0U};
  bool usable{false};
  bool local_stop_is_terminal{false};
};

struct PlanningControllerCycle3D {
  MppiControllerRequest3D request{};
  MppiSpeedPolicyResult speed_policy{};
  MppiLivenessResult liveness{};
  DirectTrackingManeuverUpdate direct_tracking_maneuver{};
  RouteProgressUpdate3D route_progress{};
  MissionGoalCaptureResult goal_capture{};
  MppiRolloutBudgetDecision rollout_budget{};
  ProductionMppiCooperativeUpdate cooperative{};
  ProductionMppiNonCooperativeUpdate noncooperative{};
  mppi::RiskTier route_required_risk_tier{mppi::RiskTier::kPreferred};
  ProductionMppiPlanningState planning_state{ProductionMppiPlanningState::kPlanned};
  std::string target_source;
};

enum class PlanningCycleStatus3D : std::uint8_t {
  kReady,
  kInvalidRequest,
  kTrackingHandoffRetained,
};

struct PlanningCycleOutcome3D {
  PlanningCycleStatus3D status{PlanningCycleStatus3D::kInvalidRequest};
  PlanningRouteDecision3D route{};
  PlanningControllerCycle3D controller{};
  PlanningCycleEffects3D effects{};

  [[nodiscard]] bool ready() const noexcept {
    return status == PlanningCycleStatus3D::kReady;
  }
};

[[nodiscard]] const char*
planningCycleStatus3DName(PlanningCycleStatus3D status) noexcept;

// Owns the stateful, ROS-free planning-cycle policy. Runtime capture, ROS
// publication, GPU residency, and execution commit stay in the node adapter.
class PlanningCycleCoordinator3D final {
public:
  PlanningCycleCoordinator3D(ExecutionSupervisor3D& execution_supervisor,
                             PlanningCycleCoordinatorConfig3D config);

  PlanningCycleCoordinator3D(const PlanningCycleCoordinator3D&) = delete;
  PlanningCycleCoordinator3D& operator=(const PlanningCycleCoordinator3D&) = delete;
  PlanningCycleCoordinator3D(PlanningCycleCoordinator3D&&) = delete;
  PlanningCycleCoordinator3D& operator=(PlanningCycleCoordinator3D&&) = delete;

  [[nodiscard]] bool goalCaptureLatchedFor(const Point3& mission_goal) const noexcept;
  [[nodiscard]] PlanningCycleOutcome3D prepare(const PlanningCycleRequest3D& request);

private:
  // Clearance of the horizon execution currently owns, against the world as it
  // stands now. Absent when nothing owns vehicle motion.
  [[nodiscard]] std::optional<ExecutedHorizonClearance3D>
  measureResidentExecutionClearance(const PlanningCycleRequest3D& request) const;

  ExecutionSupervisor3D& execution_supervisor_;
  PlanningCycleCoordinatorConfig3D config_{};
  RouteExecutionSelector3D route_execution_selector_;
  mppi::TrajectoryReferenceAdapter3D trajectory_reference_adapter_{};
  MppiLivenessSupervisor liveness_supervisor_;
  std::unique_ptr<RouteProgressTracker3D> route_progress_tracker_;
  MissionGoalCaptureLatch goal_capture_latch_;
  DirectTrackingManeuverLifecycle direct_tracking_maneuver_lifecycle_;
  ConstrainedRouteCoordinator constrained_route_coordinator_{};
  // The reference speed the previous cycle published and when, for the rise
  // limit the speed policy applies.
  std::optional<double> previous_reference_speed_mps_;
  std::int64_t previous_reference_stamp_ns_{0};
  PassageTraversalEvidenceTracker passage_traversal_evidence_tracker_{};
  PassageGeometryEvidenceTracker passage_geometry_evidence_tracker_{};
  std::unique_ptr<NonCooperativeCollisionAvoidance> noncooperative_avoidance_;
};

} // namespace drone_city_nav

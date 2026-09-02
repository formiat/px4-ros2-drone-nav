#pragma once

#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_nominal_reseed.hpp"
#include "drone_city_nav/mppi_rollout_budget.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/rolling_route_telemetry_3d.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_execution_types.hpp"
#include "production_mppi_node_types.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

// Wall-clock cost of the planning tick outside the GPU. The controller deadline
// measures only the MPPI backend; these phases make CPU-side validation,
// assembly, and publication visible in the same diagnostics.
struct ProductionMppiTickPhaseTimings {
  double snapshot_ms{0.0};
  double controller_ms{0.0};
  double publication_ms{0.0};
  double total_ms{0.0};
};

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
  std::shared_ptr<const WorldSnapshot3D> world;
  ProductionWorldBuildTelemetry3D world_build{};
  std::shared_ptr<const ProductionRouteActivationResult3D> route_pipeline;
  std::shared_ptr<const CertifiedRouteSuffix3D> execution_route;
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
  ProductionMppiTickPhaseTimings phases{};
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

} // namespace drone_city_nav

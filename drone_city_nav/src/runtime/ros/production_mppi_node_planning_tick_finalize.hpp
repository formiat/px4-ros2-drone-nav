#pragma once

#include "drone_city_nav/mppi_nominal_reseed.hpp"

#include <chrono>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

struct ProductionMppiPlanningTickFinalization {
  const mppi::MppiTickInput& input;
  mppi::MppiTickResult& result;
  const std::shared_ptr<const WorldSnapshot3D>& world;
  const ProductionWorldBuildTelemetry3D& world_build;
  const std::shared_ptr<const ProductionRouteActivationResult3D>& route_pipeline;
  const ProductionRouteExecutionSelection3D& route_execution;
  const std::shared_ptr<const VersionedExecutionInput3D>& execution_input;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence;
  const OffboardSessionAdmissionState& offboard_session;
  std::int64_t offboard_session_receive_stamp_ns;
  const ProductionMppiNavigation& navigation;
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>>& execution_mppi_route;
  const std::shared_ptr<const std::vector<PassageTraversalId>>&
      execution_selected_passage_traversal_ids;
  const std::shared_ptr<const ProductionNavigationObjective>& objective;
  const ProductionMppiPredictionError& prediction;
  const MppiLivenessResult& liveness;
  const DirectTrackingManeuverUpdate& direct_tracking_maneuver;
  const MppiSpeedPolicyResult& speed_policy;
  const RouteProgressUpdate3D& route_progress;
  const MppiEligibleRolloutUpdate& no_eligible_recovery;
  const MissionGoalCaptureResult& goal_capture;
  const MppiRolloutBudgetDecision& rollout_budget;
  const ProductionMppiCooperativeUpdate& cooperative;
  const ProductionMppiNonCooperativeUpdate& noncooperative;
  const RouteProgressProjection3D& route_projection;
  const Point3& mission_goal;
  const std::string& target_source;
  std::uint64_t route_generation;
  std::uint64_t memory_sequence;
  std::int64_t now_ns;
  double pose_age_ms;
  double esdf_age_ms;
  double observation_age_ms;
  double control_feedback_age_ms;
  double snapshot_ms;
  double controller_ms;
  std::chrono::steady_clock::time_point tick_started;
  RouteExecutionStatus3D route_execution_status;
  ProductionMppiPlanningState planning_state;
  ProductionMppiPreviousControlSource previous_control_source;
  mppi::RiskTier route_required_risk_tier;
  bool route_usable;
  bool direct_tracking_interception;
  bool local_route_stop_is_terminal;
  bool pose_predicted;
};

} // namespace drone_city_nav

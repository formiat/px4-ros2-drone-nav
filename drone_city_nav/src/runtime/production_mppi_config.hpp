#pragma once

#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/direct_tracking_maneuver_lifecycle.hpp"
#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mission_waypoint_capture_gate.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_rollout_budget.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/navigation_angular_derivative.hpp"
#include "drone_city_nav/navigation_health_supervisor.hpp"
#include "drone_city_nav/noncooperative_collision_avoidance.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/route_successor_improvement_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/static_route_geometry.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "production_mppi_node_types.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

struct ProductionMppiWorldTopics {
  std::string px4_local_position{"/fmu/out/vehicle_local_position_v1"};
  std::string navigation_readiness{"/drone_city_nav/navigation_ready"};
  std::string raw_obstacle_snapshot_3d{"/drone_city_nav/raw_obstacle_snapshot_3d"};
  std::string raw_obstacle_delta_3d{"/drone_city_nav/raw_obstacle_delta_3d"};
  std::string latest_lidar_obstacle_scan{"/drone_city_nav/latest_lidar_obstacle_scan"};
  std::string obstacle_memory_status{"/drone_city_nav/obstacle_memory_status"};
};

struct ProductionMppiPlanningTopics {
  std::string navigation_objective{"/drone_city_nav/navigation_objective"};
  std::string cooperative_maneuver_command{"/drone_city_nav/cooperative/command"};
  std::string cooperative_passage_state{"/drone_city_nav/cooperative/passage_state"};
  std::string noncooperative_tracks{"/drone_city_nav/noncooperative_tracks"};
  std::string radar_track_mode_command{"/drone_city_nav/radar/track_mode_command"};
};

struct ProductionMppiExecutionTopics {
  std::string px4_vehicle_status{"/fmu/out/vehicle_status_v1"};
  std::string px4_vehicle_land_detected{"/fmu/out/vehicle_land_detected"};
  std::string applied_control_feedback{"/drone_city_nav/mppi/applied_control"};
  std::string execution_horizon{"/drone_city_nav/mppi/execution_horizon"};
  std::string mission_waypoint_acknowledgement{
      "/drone_city_nav/mission_waypoint_acknowledgement"};
};

struct ProductionMppiDiagnosticsTopics {
  std::string path{"/drone_city_nav/mppi/path"};
  std::string markers{"/drone_city_nav/mppi/markers"};
  std::string status{"/drone_city_nav/mppi/status"};
  std::string world_readiness{"/drone_city_nav/mppi/world_ready"};
  std::string planner_health{"/drone_city_nav/mppi/planner_alive"};
  std::string navigation_health{"/drone_city_nav/mppi/navigation_health"};
};

struct ProductionMppiConfig final {
  struct World final {
    bool use_static_map{true};
    double maximum_esdf_age_ms{1000.0};
    double no_static_3d_esdf_update_rate_hz{1.0};
    // Threads of the dedicated world pool. World builds (static ESDF, observed
    // distance transforms) never share planner_worker_count with planner
    // continuations, so a long transform cannot starve a D* Lite repair.
    std::size_t world_worker_count{2U};
    LocalObservedEsdfWindow3D no_static_3d_esdf_window{};
    std::string frame_id{"map"};
    // RViz overlay convention for the `gazebo_map` fixed frame; see
    // visualization_marker_helpers.hpp.
    bool gazebo_aligned_rviz_axes_swapped{true};
    Px4MapFrameTransform px4_map_transform{};
    FlightEnvelopeConfig flight_envelope{};
    // The one swept body every validator answers to, planner and execution
    // alike: the configured hull, whose envelope contains the hull at every
    // tilt the dynamics reach, enveloped once here so that every validator
    // can stand it upright.
    SweptFootprintConfig physical_footprint{};
    // The hull as configured, before the envelope grew with the tilt: the
    // body a departure is validated with, since a vehicle leaves a tight
    // spot the way it entered it, at hover and upright.
    SweptFootprintConfig hull_footprint{};
    std::filesystem::path static_occupancy_3d_path{"worlds/generated_city.occupancy3d"};
    std::filesystem::path static_esdf_3d_cache_path{"worlds/generated_city.esdf3d"};
    std::filesystem::path static_free_space_topology_3d_path{};
    ProductionMppiWorldTopics topics{};
  };

  struct Planning final {
    double tick_rate_hz{50.0};
    double deadline_ms{20.0};
    double planning_tick_phase_offset_s{0.0};
    std::size_t planner_worker_count{4U};
    Point3 mission_start{54.0, 54.0, 0.0};
    MissionGoalCaptureConfig mission_goal_capture{};
    MissionWaypointSequenceConfig mission_waypoint_sequence{};
    std::vector<Point3> mission_waypoints;
    bool configured_mission_objective_enabled{false};
    double dynamic_objective_replan_distance_m{5.0};
    double dynamic_objective_replan_period_s{0.25};
    double tracking_objective_ray_sample_spacing_m{0.25};
    double tracking_capture_radius_m{5.0};
    double static_tracking_esdf_refresh_margin_m{15.0};
    DirectTrackingManeuverConfig direct_tracking_maneuver{};
    ProductionNavigationOptionalConstraints optional_constraints{};
    bool cooperative_traffic_enabled{false};
    std::string vehicle_id;
    PassageVolumeConfig cooperative_passage_volume{};
    CooperativePassageRouteConfig cooperative_passage_route{};
    CooperativePassageTimingConfig cooperative_passage_timing{};
    CooperativePassageYieldConfig cooperative_passage_yield{};
    bool noncooperative_avoidance_enabled{false};
    NonCooperativeAvoidanceConfig noncooperative_avoidance{};
    float constrained_route_speed_limit_mps{10.0F};
    PersistentPlannerConfig3D persistent_planner{};
    double route_sampling_step_m{0.5};
    double route_completion_tolerance_m{2.0};
    double static_esdf_route_lookahead_m{180.0};
    RouteEnvelopeConfig route_envelope{};
    StaticRouteExtensionConfig static_route_extension{};
    RouteSuccessorImprovementConfig3D route_successor_improvement{};
    FutureRouteConnectorConfig3D future_route_connector{};
    CertifiedRouteSpliceConfig3D certified_route_splice{};
    StaticRouteSearchRetryConfig static_route_search_retry{};
    StaticRouteGeometryConfig static_route_geometry{};
    RouteTrackingPolicy3D route_tracking_policy{};
    RouteProgressConfig3D route_progress{};
    bool route_stall_recovery_enabled{false};
    MppiLivenessConfig liveness{};
    ProductionMppiPlanningTopics topics{};
  };

  struct Execution final {
    double maximum_pose_age_ms{150.0};
    double maximum_vehicle_status_age_ms{1000.0};
    double maximum_pose_prediction_age_ms{1000.0};
    double maximum_control_feedback_age_ms{200.0};
    // How long an unacknowledged planned lease stays the wire owner before the
    // planner may replace it anyway. Bounded so one lost feedback sample cannot
    // stall replanning until the lease expires.
    double horizon_acknowledgement_grace_ms{100.0};
    std::int64_t horizon_acknowledgement_grace_ns{100'000'000LL};
    double latest_lidar_obstacle_maximum_age_ms{1000.0};
    double stale_esdf_execution_window_ms{4000.0};
    double stationary_hold_validity_s{1.0};
    std::int64_t stationary_hold_validity_ns{1'000'000'000LL};
    std::int64_t mission_goal_capture_hold_validity_ns{0};
    MissionWaypointCaptureGateConfig mission_waypoint_capture_gate{};
    NavigationHealthConfig navigation_health{};
    mppi::FiniteHorizonConfig finite_horizon{};
    // Wall-clock bound on one horizon assembly. The arrival-shaping search
    // rebuilds and revalidates the horizon once per shortened prefix, and past
    // this point the planning cycle is already late: the search returns what
    // it has and the vehicle holds instead of receiving a stale horizon.
    double maximum_assembly_ms{0.0};
    std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
    ProductionMppiExecutionTopics topics{};
  };

  struct Control final {
    mppi::BenchmarkConfig mppi{};
    MppiRolloutBudgetConfig rollout_budget{};
    MppiSpeedPolicyConfig speed_policy{};
    NavigationAngularDerivativeConfig navigation_angular_derivative{};
    ConstrainedRouteControlConfig constrained_route{};
    TrackingErrorTubeConfig3D tracking_error_tube{};
  };

  struct Diagnostics final {
    double rviz_rate_hz{10.0};
    double info_rate_hz{5.0};
    double file_rate_hz{5.0};
    double flush_period_s{1.0};
    std::size_t error_ring_capacity{25U};
    double route_constraint_distance_m{30.0};
    std::filesystem::path output_dir{"log/mppi"};
    std::int64_t rviz_period_ns{100'000'000LL};
    std::int64_t info_period_ns{200'000'000LL};
    std::int64_t file_period_ns{200'000'000LL};
    ProductionMppiDiagnosticsTopics topics{};
  };

  World world{};
  Planning planning{};
  Execution execution{};
  Control control{};
  Diagnostics diagnostics{};

  [[nodiscard]] bool valid() const noexcept;
};

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/observation_frontier.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/portal_graph.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

enum class Lattice3DStatus : std::uint8_t {
  kInvalidInput,
  kReachedPlanningGoal,
  kViableFrontier,
  kSearchIncomplete,
  kMotionGraphExhausted,
};

enum class Lattice3DRiskStage : std::uint8_t {
  kPreferredOnly,
  kPlanningAllowed,
  kCriticalAllowed,
};

enum class Lattice3DSearchTermination : std::uint8_t {
  kInvalidInput,
  kPlanningGoalReached,
  kOpenSetExhausted,
  kExpansionBudgetExhausted,
  kDeadlineReached,
};

enum class Lattice3DRoutePurpose : std::uint8_t {
  kMissionTransit,
  kLaunchDeparture,
  kObservationFrontier,
  kTopologicalBacktrack,
};

struct Lattice3DSuccessorDiagnostics {
  std::size_t lattice_generated{0U};
  std::size_t lattice_accepted{0U};
  std::size_t lattice_rejected_edge{0U};
  std::size_t lattice_rejected_zero_length{0U};
  std::size_t lattice_rejected_outside_grid{0U};
  std::size_t lattice_rejected_unknown_space{0U};
  std::size_t lattice_rejected_flight_envelope{0U};
  std::size_t lattice_rejected_invalid_esdf{0U};
  std::size_t lattice_rejected_raw_collision{0U};
  std::size_t lattice_rejected_risk_stage{0U};
  std::size_t lattice_rejected_no_cost_improvement{0U};
  std::size_t soft_tabu_penalties_applied{0U};
  std::size_t passage_generated{0U};
  std::size_t passage_accepted{0U};
  std::size_t passage_rejected{0U};
  std::size_t passage_rejected_connection_distance{0U};
  std::size_t passage_rejected_outside_grid{0U};
  std::size_t passage_rejected_unknown_space{0U};
  std::size_t passage_rejected_flight_envelope{0U};
  std::size_t passage_rejected_invalid_esdf{0U};
  std::size_t passage_rejected_raw_collision{0U};
  std::size_t passage_rejected_risk_stage{0U};
  std::size_t passage_rejected_no_cost_improvement{0U};
};

struct Lattice3DSoftTabuEntry {
  Point3 point{};
  double approach_heading_rad{0.0};
  double radius_m{0.0};
  double heading_tolerance_rad{0.0};
  double penalty_cost{0.0};
};

struct Lattice3DSuccessorBatchProfile {
  std::size_t collection_calls{0U};
  std::size_t parallel_collection_calls{0U};
  std::size_t candidates{0U};
  std::size_t parallel_candidates{0U};
  std::size_t maximum_candidates{0U};
  double worker_ms{0.0};
};

struct Lattice3DSuccessorProfiling {
  Lattice3DSuccessorBatchProfile search{};
  Lattice3DSuccessorBatchProfile continuation{};
};

struct Lattice3DRawValidationContext {
  // The immutable owner must outlive the synchronous lattice search.
  const ObservedOccupancyGrid3D* occupancy{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  std::optional<Point3> initial_connector_start;
  SweptFootprintConfig initial_connector_footprint{};
};

struct RiskAwareLattice3DConfig {
  double horizontal_step_m{2.0};
  double vertical_step_m{1.0};
  double sample_step_m{0.5};
  double physical_footprint_radius_m{0.82};
  double physical_footprint_lower_extent_m{0.23};
  double physical_footprint_upper_extent_m{0.35};
  std::size_t physical_footprint_samples{12U};
  std::size_t physical_footprint_radial_rings{2U};
  std::size_t physical_footprint_axial_samples{3U};
  double physical_footprint_sweep_step_m{0.25};
  Lattice3DRawValidationContext raw_validation{};
  FlightEnvelopeConfig flight_envelope{};
  double planning_goal_distance_m{180.0};
  double goal_tolerance_m{2.0};
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  bool require_known_free_space{false};
  bool reject_invalid_esdf{false};
  bool clearance_tier_constraints_enabled{true};
  double heading_bias_cost_per_rad{0.5};
  double nominal_horizontal_speed_mps{20.0};
  double nominal_vertical_speed_mps{4.0};
  double vertical_alignment_cost_weight{0.35};
  double route_shape_turn_cost_per_rad{0.10};
  double route_shape_vertical_turn_cost_per_rad{0.20};
  double passage_topology_transition_cost{0.0};
  double planning_exposure_cost_per_m{0.05};
  double critical_exposure_cost_per_m{0.50};
  double passage_connection_distance_m{3.0};
  double frontier_minimum_reachable_depth_m{8.0};
  double frontier_minimum_endpoint_displacement_m{2.0};
  std::size_t frontier_validation_maximum_states{2048U};
  double frontier_validation_maximum_time_ms{50.0};
  double observation_frontier_replacement_minimum_score_improvement{0.5};
  SensorObservabilityConfig sensor_observability{};
  std::size_t maximum_topology_search_groups{3U};
  std::size_t maximum_expansions{200000U};
  double maximum_search_time_ms{250.0};
};

struct Lattice3DTopologyCandidate {
  std::size_t candidate_rank{0U};
  std::string topology;
  Lattice3DRiskStage risk_stage{Lattice3DRiskStage::kPreferredOnly};
  Lattice3DStatus status{Lattice3DStatus::kInvalidInput};
  Lattice3DSearchTermination termination{Lattice3DSearchTermination::kInvalidInput};
  double objective_cost{0.0};
  double route_length_m{0.0};
  double estimated_travel_time_s{0.0};
  double vertical_alignment_time_s{0.0};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
  double turn_cost{0.0};
  double achieved_progress_m{0.0};
  double minimum_clearance_m{0.0};
  std::size_t expansions{0U};
  std::size_t stale_queue_pops{0U};
  std::size_t open_peak{0U};
  std::size_t records_peak{0U};
  std::size_t terminal_successor_count{0U};
  std::size_t continuation_reachable_states{0U};
  double continuation_reachable_depth_m{0.0};
  bool continuation_validation_budget_exhausted{false};
  std::string decision_reason;
  bool selected{false};
};

struct RiskAwareLattice3DResult {
  Lattice3DStatus status{Lattice3DStatus::kInvalidInput};
  Lattice3DRiskStage risk_stage{Lattice3DRiskStage::kPreferredOnly};
  Lattice3DSearchTermination termination{Lattice3DSearchTermination::kInvalidInput};
  std::vector<Point3> points;
  std::vector<RouteSample3D> route;
  Point3 planning_goal{};
  std::size_t expansions{0U};
  std::size_t stale_queue_pops{0U};
  std::size_t open_peak{0U};
  std::size_t records_peak{0U};
  std::size_t terminal_successor_count{0U};
  std::size_t continuation_reachable_states{0U};
  double continuation_reachable_depth_m{0.0};
  bool continuation_validation_budget_exhausted{false};
  bool reached_mission_goal{false};
  double achieved_progress_m{0.0};
  double minimum_clearance_m{0.0};
  double objective_cost{0.0};
  double route_length_m{0.0};
  double estimated_travel_time_s{0.0};
  double vertical_alignment_time_s{0.0};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
  double turn_cost{0.0};
  double search_time_budget_ms{0.0};
  double search_ms{0.0};
  std::size_t topology_searches{0U};
  std::size_t parallel_topology_searches{0U};
  double topology_search_worker_ms{0.0};
  double continuation_validation_ms{0.0};
  double frontier_endpoint_displacement_m{0.0};
  double frontier_selection_score{0.0};
  std::uint64_t route_fingerprint{0U};
  Lattice3DRoutePurpose route_purpose{Lattice3DRoutePurpose::kMissionTransit};
  std::optional<ObservationFrontier> observation_frontier;
  Lattice3DSuccessorDiagnostics successor_diagnostics{};
  Lattice3DSuccessorProfiling successor_profiling{};
  std::vector<SelectedPassageTraversal> selected_passage_traversals;
  std::vector<Lattice3DTopologyCandidate> topology_candidates;
};

struct Lattice3DStrategicDirective {
  Point3 planning_goal{};
  Vec3 preferred_direction{};
  Lattice3DRoutePurpose route_purpose{Lattice3DRoutePurpose::kMissionTransit};
  std::optional<ObservationFrontier> observation_frontier;
  double selection_score{0.0};
  bool reaches_mission_goal{false};
};

[[nodiscard]] RiskAwareLattice3DResult planRiskAwareLattice3D(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, const Point3& start,
    const Vec3& preferred_direction, const Point3& mission_goal,
    std::span<const PassageTraversalEdge> passage_traversals,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* worker_pool = nullptr,
    const Lattice3DStrategicDirective* strategic_directive = nullptr,
    std::span<const Lattice3DSoftTabuEntry> soft_tabu = {});

[[nodiscard]] const char* lattice3DStatusName(Lattice3DStatus status) noexcept;

[[nodiscard]] const char* lattice3DRiskStageName(Lattice3DRiskStage stage) noexcept;

[[nodiscard]] const char*
lattice3DSearchTerminationName(Lattice3DSearchTermination termination) noexcept;

[[nodiscard]] const char*
lattice3DRoutePurposeName(Lattice3DRoutePurpose purpose) noexcept;

} // namespace drone_city_nav

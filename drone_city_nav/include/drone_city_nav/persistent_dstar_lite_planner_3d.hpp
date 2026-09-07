#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace drone_city_nav {

enum class PlannerInputStatus3D : std::uint8_t {
  kAccepted,
  kInvalidInput,
  kStartUnavailable,
  kGoalUnavailable,
};

enum class SearchProgress3D : std::uint8_t {
  kRunning,
  kConverged,
  kNoRoute,
  kInvalidated,
};

enum class SpatialRouteCandidateSource3D : std::uint8_t {
  kFeasibilitySearch,
  kExecutionTimeRefinement,
};

struct PersistentPlannerWorld3D {
  std::shared_ptr<const ObservedOccupancyGrid3D> observed_occupancy;
  std::shared_ptr<const OccupancyGrid3D> static_occupancy;
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  // Exact predecessor for which dirty_chunks is complete. A skipped world
  // publication therefore forces a full repair instead of applying an
  // incomplete delta to an older resident planner world.
  std::uint64_t incremental_parent_revision{0U};
  std::uint64_t occupied_fingerprint{0U};
  // The world carries no incremental lineage (for example a raw search
  // overlay). The planner then derives the change set from an exact occupied
  // grid difference against its resident world instead of trusting
  // dirty_chunks; it never discards search state merely because of this flag.
  bool full_reset{false};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const GridBounds3D* bounds() const noexcept;
};

struct PersistentPlannerConfig3D {
  // Level zero is the complete 26-connected lattice. Each higher level adds a
  // world-aligned 26-connected overlay with twice the stride. Long edges remain
  // lazy and are admitted only by exact raw swept-footprint validation.
  double minimum_horizontal_step_m{2.0};
  double minimum_vertical_step_m{1.0};
  std::size_t maximum_adaptive_lattice_level{2U};
  FlightTimeModel3D time_model{};
  double minimum_continuous_turn_alignment{0.7071067811865476};
  double goal_tolerance_m{1.0};
  std::size_t connector_search_radius_cells{2U};
  // Subdivisions of one lattice cell the departure refinement probes when no
  // node in the connector radius is reachable in a single segment. Zero
  // disables the refinement, and start_unavailable is then terminal for as
  // long as the vehicle stays where it is.
  std::size_t departure_refinement_subdivisions{4U};
  // Upper bound on the waypoints that refinement examines, nearest first. The
  // refinement runs only on updates the planner would otherwise abandon, but
  // it still has to fit inside the update budget.
  std::size_t maximum_departure_refinement_probes{512U};
  // The feasibility-first search tries the direct raw connector to the exact
  // goal only from nodes within this distance; a raw sweep across the whole
  // remaining route on every expansion would dominate the search budget.
  double feasibility_goal_connector_reach_m{40.0};
  // Clearance reach of the feasibility-first search's ranking. The search
  // prices its edges with the same soft clearance ranking as the D* search,
  // but derives node clearances only within this shorter reach, so the first
  // route keeps its body out of the critical band without paying the full
  // ranking reach on every explored node.
  double feasibility_clearance_ranking_distance_m{2.0};
  // Feasibility-first search may publish any complete raw-valid route before
  // the persistent graph proves translation- or execution-time optimality.
  bool feasibility_first_enabled{true};
  std::size_t maximum_feasibility_expansions_per_update{4096U};
  double maximum_feasibility_compute_time_ms{50.0};
  // Shared per-call graph-work cap. Pending repair vertices consume this
  // budget before new shortest-path expansions are allowed.
  std::size_t maximum_expansions_per_update{200000U};
  // The execution-time refinement stops once no remaining state can improve
  // the incumbent by more than the larger of these two margins. They mirror
  // the successor admission thresholds: a smaller gain could never replace an
  // active route, so searching for it only burns the planner budget.
  double execution_time_refinement_minimum_improvement_s{1.0};
  double execution_time_refinement_minimum_improvement_ratio{0.05};
  std::size_t maximum_incremental_changed_voxels{32768U};
  std::size_t maximum_extracted_path_nodes{8192U};
  std::size_t maximum_shortcut_checks{8192U};
  // Every published route has its interior vertices slid across the passage
  // toward the local clearance maximum before it is offered. A lattice node
  // lands wherever the grid puts it, so a route through a narrow doorway runs
  // against the jamb: execution has to crawl through it, and the first freshly
  // observed voxel of that jamb blocks the route. The move keeps the path
  // length and is only kept when both incident segments still validate.
  std::size_t clearance_centering_passes{3U};
  std::size_t maximum_clearance_centering_queries{4096U};
  double maximum_compute_time_ms{150.0};
  // Share of one update reserved for the shortest-path search, whatever the
  // stages before it spend. Repair, change scheduling and the feasibility
  // search all run first against the same budget, and between them they took
  // all of it: D* was measured expanding nothing in over half the updates, so
  // the route the vehicle flew came from the unranked feasibility branch again
  // and again. Zero restores the previous first-come division.
  double guaranteed_spatial_search_fraction{0.34};
  // Share of one update's expansions the execution-time refinement is
  // guaranteed. The refinement searches its own graph, so charging it only
  // what the spatial search left means it runs only while D* is idle — and
  // the refinement is what turns a first-found feasibility route into a
  // ranked one.
  double guaranteed_refinement_expansion_fraction{0.34};
  // Soft clearance ranking. An edge whose endpoints' body surface lies within
  // clearance_ranking_distance_m of raw occupied evidence costs its flight
  // time scaled by 1 + weight * (1 - clearance / distance)^2, where the
  // clearance is the raw distance from the node centre less the footprint
  // radius. Zero weight disables it. Reachability is untouched: only the raw
  // body check rejects.
  double clearance_ranking_weight{0.0};
  double clearance_ranking_distance_m{6.0};
  // Critical band of the same ranking: below clearance_ranking_critical_distance_m
  // the factor additionally grows by critical_weight * (1 - clearance /
  // critical_distance)^2. It mirrors the execution risk model, whose critical
  // exposure makes a route inside that band nearly unexecutable, so the
  // planner prefers a long detour over a critical metre just as the executor
  // does. Zero weight disables the band; it never rejects an edge.
  double clearance_ranking_critical_distance_m{1.0};
  double clearance_ranking_critical_weight{0.0};
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
};

struct PersistentPlannerRequest3D {
  Point3 start{};
  Vec3 velocity{};
  Point3 mission_goal{};
  std::uint64_t mission_epoch{0U};
  PersistentPlannerWorld3D world{};
  // Identifies the consumer's search session. The first update of a new
  // session delivers the resident incumbent even without an improvement: a
  // consumer that opened a session holds no route of this search yet, and an
  // incumbent it never received is not an improvement it can skip. Zero keeps
  // improvement-only publication.
  std::uint64_t session_id{0U};
  // The consumer rejected the incumbent this planner last delivered (its
  // execution found it blocked on newer evidence). The first update of the
  // session drops the resident incumbent so the search delivers a route found
  // on the current world instead of re-offering the rejected one.
  bool discard_incumbent{false};
  // Counts the incumbents this consumer rejected at activation for reasons
  // relative to the vehicle (a handoff it cannot fly, a connector its raw
  // evidence blocks). A sequence newer than the one the planner last applied
  // drops the resident incumbent and restarts the feasibility search from the
  // current start, so a session whose incremental repair lags behind the
  // evidence still delivers a route the vehicle can enter.
  std::uint64_t incumbent_rejection_sequence{0U};
};

struct SpatialRouteCandidate3D {
  std::vector<Point3> points;
  SpatialRouteCandidateSource3D source{
      SpatialRouteCandidateSource3D::kFeasibilitySearch};
  double path_length_m{0.0};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};
  // The execution time with every segment scaled by the clearance ranking of
  // the body along it, on the world the candidate was evaluated on. Candidates
  // are compared by it, so a faster route that hugs raw occupied evidence
  // never displaces a slightly slower clear one. Zero when not evaluated.
  double ranked_execution_time_s{0.0};

  [[nodiscard]] bool valid() const noexcept;
  // The objective candidates compete on: the ranked time when evaluated,
  // otherwise the plain execution time.
  [[nodiscard]] double objectiveS() const noexcept;
};

struct PlannerTelemetry3D {
  // Names the first failing input predicate when the update is not accepted.
  const char* input_failure{"none"};
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
  std::size_t clearance_centering_queries{0U};
  std::size_t clearance_centering_moves{0U};
  std::size_t lattice_edge_queries{0U};
  std::size_t raw_edge_validation_checks{0U};
  std::size_t adaptive_edge_queries{0U};
  std::size_t adaptive_edges_in_extracted_path{0U};
  std::size_t maximum_queried_lattice_level{0U};
  std::size_t execution_time_search_expansions{0U};
  std::size_t execution_time_search_records{0U};
  std::size_t execution_time_search_open_entries{0U};
  double execution_time_search_objective_s{0.0};
  double world_update_ms{0.0};
  // Breakdown of the world update: occupied-difference computation and
  // world installation.
  double world_diff_ms{0.0};
  double world_install_ms{0.0};
  double search_ms{0.0};
  // Breakdown of the search phases of this update: the feasibility-first
  // search, the affected-vertex repair, the persistent spatial search, and
  // the execution-time refinement.
  double feasibility_ms{0.0};
  double repair_ms{0.0};
  double spatial_search_ms{0.0};
  double refinement_ms{0.0};
  // Repair scheduling breakdown of this update.
  double schedule_ms{0.0};
  double schedule_ranking_ms{0.0};
  std::size_t schedule_edges_forgotten{0U};
  std::size_t schedule_clearances_tightened{0U};
  std::size_t schedule_clearances_rederived{0U};
  // The search had to leave the vehicle through a refined free step because no
  // lattice node was reachable from where it stands.
  bool departure_waypoint_used{false};
  bool search_state_reused{false};
  bool occupied_world_unchanged{false};
  bool incumbent_retained{false};
  bool repair_pending{false};
  bool feasibility_attempted{false};
  bool feasibility_route_found{false};
  // The feasibility frontier emptied without a raw-valid candidate: every
  // lattice node reachable from the anchor was explored.
  bool feasibility_frontier_exhausted{false};
  std::size_t feasibility_explored_nodes{0U};
  double feasibility_closest_goal_distance_m{0.0};
  std::size_t feasibility_restarts{0U};
  // Labels dropped because a lattice edge on their chain stopped surviving
  // the resident world, since the planner was constructed.
  std::size_t feasibility_invalidated_labels{0U};
  std::size_t feasibility_adopted_labels{0U};
  std::size_t feasibility_last_invalid_segment{0U};
  Point3 feasibility_anchor{};
  bool execution_time_search_complete{false};
  bool incumbent_available{false};
};

struct PlannerUpdate3D {
  std::optional<SpatialRouteCandidate3D> improved_incumbent;
  PlannerInputStatus3D input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D progress{SearchProgress3D::kInvalidated};
  PlannerTelemetry3D telemetry{};

  [[nodiscard]] bool publishable() const noexcept;
  [[nodiscard]] bool running() const noexcept;
};

struct PlannerDispatch3D {
  bool publish_incumbent{false};
  bool continue_search{false};
  bool terminal{false};
};

[[nodiscard]] PlannerDispatch3D
coordinatePlannerUpdate3D(const PlannerUpdate3D& update) noexcept;

// Share of the remaining update budget the feasibility-first search may
// spend. It runs only while no route exists, so the resident session's work
// improves a route nobody can fly: the session keeps a reserve (the configured
// feasibility time, at most a third of the remainder) to absorb the world, and
// the search takes the rest of the update.
[[nodiscard]] std::chrono::steady_clock::duration
feasibilitySearchBudget3D(std::chrono::steady_clock::duration remaining,
                          std::chrono::steady_clock::duration configured,
                          bool persistent_search_has_work) noexcept;

namespace detail {
class PersistentDStarLitePlanner3DImpl;
}

class PersistentDStarLitePlanner3D final {
public:
  explicit PersistentDStarLitePlanner3D(PersistentPlannerConfig3D config = {});
  ~PersistentDStarLitePlanner3D();

  PersistentDStarLitePlanner3D(const PersistentDStarLitePlanner3D&) = delete;
  PersistentDStarLitePlanner3D& operator=(const PersistentDStarLitePlanner3D&) = delete;
  PersistentDStarLitePlanner3D(PersistentDStarLitePlanner3D&&) noexcept;
  PersistentDStarLitePlanner3D& operator=(PersistentDStarLitePlanner3D&&) noexcept;

  [[nodiscard]] PlannerUpdate3D plan(const PersistentPlannerRequest3D& request);
  void reset() noexcept;
  [[nodiscard]] const PersistentPlannerConfig3D& config() const noexcept;

private:
  std::unique_ptr<detail::PersistentDStarLitePlanner3DImpl> implementation_;
};

[[nodiscard]] const char*
plannerInputStatus3DName(PlannerInputStatus3D status) noexcept;
[[nodiscard]] const char* searchProgress3DName(SearchProgress3D progress) noexcept;
[[nodiscard]] const char*
spatialRouteCandidateSource3DName(SpatialRouteCandidateSource3D source) noexcept;

} // namespace drone_city_nav

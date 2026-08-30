#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

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
  // Feasibility-first search may publish any complete raw-valid route before
  // the persistent graph proves translation- or execution-time optimality.
  bool feasibility_first_enabled{true};
  std::size_t maximum_feasibility_expansions_per_update{4096U};
  double maximum_feasibility_compute_time_ms{50.0};
  // Shared per-call graph-work cap. Pending repair vertices consume this
  // budget before new shortest-path expansions are allowed.
  std::size_t maximum_expansions_per_update{200000U};
  std::size_t maximum_incremental_changed_voxels{32768U};
  std::size_t maximum_extracted_path_nodes{8192U};
  std::size_t maximum_shortcut_checks{8192U};
  double maximum_compute_time_ms{150.0};
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
};

struct PersistentPlannerRequest3D {
  Point3 start{};
  Vec3 velocity{};
  Point3 mission_goal{};
  std::uint64_t mission_epoch{0U};
  PersistentPlannerWorld3D world{};
};

struct SpatialRouteCandidate3D {
  std::vector<Point3> points;
  SpatialRouteCandidateSource3D source{
      SpatialRouteCandidateSource3D::kFeasibilitySearch};
  double path_length_m{0.0};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

struct PlannerTelemetry3D {
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
  double execution_time_search_objective_s{0.0};
  double world_update_ms{0.0};
  double search_ms{0.0};
  bool search_state_reused{false};
  bool occupied_world_unchanged{false};
  bool incumbent_retained{false};
  bool repair_pending{false};
  bool feasibility_attempted{false};
  bool feasibility_route_found{false};
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

#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

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

struct RiskAwareLattice3DConfig {
  double horizontal_step_m{2.0};
  double vertical_step_m{1.0};
  double sample_step_m{0.5};
  double planning_goal_distance_m{180.0};
  double goal_tolerance_m{2.0};
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  double heading_bias_cost_per_rad{0.5};
  double nominal_horizontal_speed_mps{20.0};
  double nominal_vertical_speed_mps{4.0};
  double vertical_alignment_cost_weight{0.0};
  double turn_cost_per_rad{0.10};
  double planning_exposure_cost_per_m{0.05};
  double critical_exposure_cost_per_m{0.50};
  double channel_connection_distance_m{3.0};
  double frontier_minimum_reachable_depth_m{8.0};
  std::size_t frontier_validation_maximum_states{2048U};
  std::size_t maximum_expansions{200000U};
  double maximum_search_time_ms{250.0};
};

struct Lattice3DTopologyCandidate {
  std::string topology;
  Lattice3DRiskStage risk_stage{Lattice3DRiskStage::kPreferredOnly};
  Lattice3DStatus status{Lattice3DStatus::kInvalidInput};
  double objective_cost{0.0};
  double route_length_m{0.0};
  double estimated_travel_time_s{0.0};
  double vertical_alignment_time_s{0.0};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
  double turn_cost{0.0};
  std::string decision_reason;
  bool selected{false};
};

struct RiskAwareLattice3DResult {
  Lattice3DStatus status{Lattice3DStatus::kInvalidInput};
  Lattice3DRiskStage risk_stage{Lattice3DRiskStage::kPreferredOnly};
  std::vector<Point3> points;
  std::vector<RouteSample3D> route;
  std::size_t expansions{0U};
  std::size_t stale_queue_pops{0U};
  std::size_t open_peak{0U};
  std::size_t terminal_successor_count{0U};
  std::size_t continuation_reachable_states{0U};
  double continuation_reachable_depth_m{0.0};
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
  std::vector<SelectedChannelTraversal> selected_channels;
  std::vector<Lattice3DTopologyCandidate> topology_candidates;
};

[[nodiscard]] RiskAwareLattice3DResult
planRiskAwareLattice3D(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& start, const Vec3& preferred_direction,
                       const Point3& mission_goal,
                       std::span<const ConstrainedFreeSpaceEdge> channel_edges,
                       const RiskAwareLattice3DConfig& config);

[[nodiscard]] const char* lattice3DStatusName(Lattice3DStatus status) noexcept;

} // namespace drone_city_nav

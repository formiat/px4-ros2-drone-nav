#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/semantic_portal_route.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class LatticePlanStatus : std::uint8_t {
  kInvalidInput,
  kReachedPlanningGoal,
  kViableFrontier,
  kDeadEnd,
};

enum class LatticeSearchTermination : std::uint8_t {
  kInvalidInput,
  kPlanningGoalReached,
  kOpenSetExhausted,
  kExpansionBudgetExhausted,
};

struct RiskAwareLatticeConfig {
  int heading_bins{16};
  double primitive_length_m{4.0};
  double primitive_sample_step_m{0.5};
  double goal_tolerance_m{5.0};
  double receding_goal_distance_m{60.0};
  double collision_radius_m{0.5};
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  double critical_cost_per_m{100000.0};
  double planning_cost_per_m{1000.0};
  double turn_cost{0.5};
  double heuristic_weight{2.0};
  std::size_t maximum_expansions{60000U};
  std::size_t minimum_frontier_guide_points{3U};
  double minimum_frontier_guide_length_m{8.0};
  double minimum_frontier_progress_m{4.0};
  double portal_lateral_margin_m{0.5};
  double portal_entry_capture_distance_m{6.0};
  double portal_exit_extension_m{4.0};
  int portal_maximum_heading_delta_bins{4};
};

struct RiskAwareLatticeResult {
  bool valid{false};
  bool reached_mission_goal{false};
  bool planning_goal_reached{false};
  bool exact_terminal_connector{false};
  Point2 planning_goal{};
  std::vector<Point2> guide;
  std::size_t expansions{0U};
  double cost{0.0};
  LatticePlanStatus status{LatticePlanStatus::kInvalidInput};
  LatticeSearchTermination termination{LatticeSearchTermination::kInvalidInput};
  double achieved_progress_m{0.0};
  double guide_length_m{0.0};
  double remaining_goal_distance_m{0.0};
  std::size_t terminal_successor_count{0U};
};

[[nodiscard]] RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, Point2 start,
    double start_heading_rad, Point2 mission_goal, const RiskAwareLatticeConfig& config,
    std::span<const SemanticPortalPrimitive> portals = {});

[[nodiscard]] const char* latticePlanStatusName(LatticePlanStatus status) noexcept;

[[nodiscard]] const char*
latticeSearchTerminationName(LatticeSearchTermination termination) noexcept;

} // namespace drone_city_nav

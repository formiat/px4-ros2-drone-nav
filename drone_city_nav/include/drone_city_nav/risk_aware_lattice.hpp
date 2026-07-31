#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/semantic_portal_route.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class LatticePlanStatus : std::uint8_t {
  kInvalidInput,
  kReachedPlanningGoal,
  kViableFrontier,
  kSearchIncomplete,
  kMotionGraphExhausted,
};

enum class LatticeSearchTermination : std::uint8_t {
  kInvalidInput,
  kPlanningGoalReached,
  kOpenSetExhausted,
  kExpansionBudgetExhausted,
  kDeadlineReached,
  kRoiBoundaryReached,
};

enum class LatticeRiskStage : std::uint8_t {
  kPreferredOnly,
  kPlanningAllowed,
  kCriticalAllowed,
};

struct LatticeFrontierBlacklistEntry {
  Point2 failure_point{};
  double approach_heading_rad{0.0};
  std::int64_t expires_at_ns{0};
};

struct LatticeSuccessorDiagnostics {
  std::size_t generated{0U};
  std::size_t accepted{0U};
  std::size_t rejected_outside_roi{0U};
  std::size_t rejected_outside_grid{0U};
  std::size_t rejected_invalid_clearance{0U};
  std::size_t rejected_raw_collision{0U};
  std::size_t rejected_portal_footprint{0U};
  std::size_t rejected_risk_stage{0U};
  std::size_t rejected_blacklisted_failure{0U};
  std::size_t rejected_no_cost_improvement{0U};
};

struct RiskAwareLatticeConfig {
  int heading_bins{16};
  double primitive_length_m{4.0};
  double short_primitive_length_m{2.0};
  double primitive_sample_step_m{0.5};
  double goal_tolerance_m{5.0};
  double receding_goal_distance_m{60.0};
  double critical_distance_m{1.0};
  double preferred_distance_m{6.0};
  double planning_exposure_tie_break_per_m{1.0};
  double critical_exposure_tie_break_per_m{10.0};
  double turn_cost{0.5};
  double heuristic_weight{2.0};
  std::size_t maximum_expansions{60000U};
  double maximum_search_time_ms{100.0};
  double maximum_search_roi_halo_m{90.0};
  std::size_t maximum_frontier_candidates{64U};
  std::size_t minimum_frontier_guide_points{3U};
  double minimum_frontier_guide_length_m{8.0};
  double minimum_frontier_progress_m{4.0};
  double minimum_frontier_reachable_depth_m{8.0};
  double frontier_blacklist_radius_m{6.0};
  int frontier_blacklist_heading_tolerance_bins{1};
  double portal_lateral_margin_m{0.5};
  double portal_entry_capture_distance_m{6.0};
  double portal_exit_extension_m{4.0};
  double portal_center_preference_cost{2.0};
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
  std::size_t stale_queue_pops{0U};
  std::size_t open_peak{0U};
  std::size_t records_peak{0U};
  double cost{0.0};
  LatticeRiskStage risk_stage{LatticeRiskStage::kPreferredOnly};
  LatticePlanStatus status{LatticePlanStatus::kInvalidInput};
  LatticeSearchTermination termination{LatticeSearchTermination::kInvalidInput};
  double achieved_progress_m{0.0};
  double guide_length_m{0.0};
  double remaining_goal_distance_m{0.0};
  std::size_t terminal_successor_count{0U};
  std::size_t two_step_reachable_states{0U};
  double reachable_depth_m{0.0};
  std::size_t frontier_candidates_considered{0U};
  LatticeSuccessorDiagnostics successor_diagnostics{};
  bool search_session_resumed{false};
};

class RiskAwareLatticeSearchSession final {
public:
  RiskAwareLatticeSearchSession();
  ~RiskAwareLatticeSearchSession();

  RiskAwareLatticeSearchSession(const RiskAwareLatticeSearchSession&) = delete;
  RiskAwareLatticeSearchSession&
  operator=(const RiskAwareLatticeSearchSession&) = delete;
  RiskAwareLatticeSearchSession(RiskAwareLatticeSearchSession&&) noexcept;
  RiskAwareLatticeSearchSession& operator=(RiskAwareLatticeSearchSession&&) noexcept;

  void reset();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  friend RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
      const mppi::EsdfGrid&, std::span<const float>, Point2, double, Point2,
      const RiskAwareLatticeConfig&, std::span<const SemanticPortalPrimitive>,
      std::span<const LatticeFrontierBlacklistEntry>, RiskAwareLatticeSearchSession*);
};

[[nodiscard]] RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, Point2 start,
    double preferred_heading_rad, Point2 mission_goal,
    const RiskAwareLatticeConfig& config,
    std::span<const SemanticPortalPrimitive> portals = {},
    std::span<const LatticeFrontierBlacklistEntry> frontier_blacklist = {},
    RiskAwareLatticeSearchSession* session = nullptr);

[[nodiscard]] const char* latticePlanStatusName(LatticePlanStatus status) noexcept;

[[nodiscard]] const char*
latticeSearchTerminationName(LatticeSearchTermination termination) noexcept;

[[nodiscard]] const char* latticeRiskStageName(LatticeRiskStage stage) noexcept;

} // namespace drone_city_nav

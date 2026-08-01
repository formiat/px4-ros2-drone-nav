#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
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
  double vertical_cost_per_m{0.25};
  double risk_exposure_tie_break_per_m{1.0};
  std::size_t maximum_expansions{200000U};
  double maximum_search_time_ms{250.0};
};

struct RiskAwareLattice3DResult {
  Lattice3DStatus status{Lattice3DStatus::kInvalidInput};
  Lattice3DRiskStage risk_stage{Lattice3DRiskStage::kPreferredOnly};
  std::vector<Point3> points;
  std::vector<RouteSample3D> route;
  std::size_t expansions{0U};
  std::size_t stale_queue_pops{0U};
  std::size_t open_peak{0U};
  bool reached_mission_goal{false};
  double achieved_progress_m{0.0};
  double minimum_clearance_m{0.0};
};

[[nodiscard]] RiskAwareLattice3DResult
planRiskAwareLattice3D(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& start, const Vec3& preferred_direction,
                       const Point3& mission_goal,
                       const RiskAwareLattice3DConfig& config);

[[nodiscard]] const char* lattice3DStatusName(Lattice3DStatus status) noexcept;

} // namespace drone_city_nav

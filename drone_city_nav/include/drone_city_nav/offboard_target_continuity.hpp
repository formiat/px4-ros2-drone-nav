#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <span>

namespace drone_city_nav {

enum class TargetContinuityDecisionReason {
  kNoPreviousTarget,
  kSwitchedToNewWaypoint,
  kKeptPreviousTarget,
  kForcedSwitchUnsafePrevious,
  kForcedSwitchPreviousBehindPath,
};

struct TargetContinuityDecision {
  Point2 target{};
  TargetContinuityDecisionReason reason{
      TargetContinuityDecisionReason::kNoPreviousTarget};
  std::size_t target_waypoint_index{0U};
  double target_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double path_error_m{std::numeric_limits<double>::quiet_NaN()};
  bool target_waypoint_index_valid{false};
  bool previous_target_safe{false};
  bool selected_target_traversable{false};
};

[[nodiscard]] const char*
targetContinuityDecisionReasonName(TargetContinuityDecisionReason reason) noexcept;

[[nodiscard]] TargetContinuityDecision
decideTargetAfterReplan(std::span<const Point2> path, Point2 current_position,
                        Point2 previous_target, Point2 proposed_target,
                        bool had_previous_target, std::size_t current_waypoint_index,
                        std::size_t fallback_waypoint_index, double hysteresis_m,
                        const OccupancyGrid2D* prohibited_grid);

} // namespace drone_city_nav

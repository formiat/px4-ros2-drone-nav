#include "drone_city_nav/offboard_target_continuity.hpp"

#include "drone_city_nav/offboard_path_follower.hpp"
#include "drone_city_nav/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

struct TraversablePathTarget {
  Point2 target{};
  std::size_t waypoint_index{0U};
};

[[nodiscard]] std::optional<TraversablePathTarget> firstTraversablePathTarget(
    const std::span<const Point2> path, const Point2 current_position,
    const std::size_t start_waypoint_index, const OccupancyGrid2D& prohibited_grid) {
  if (path.empty()) {
    return std::nullopt;
  }

  const std::size_t first_index = std::min(start_waypoint_index, path.size() - 1U);
  for (std::size_t index = first_index; index < path.size(); ++index) {
    if (pathSegmentIsTraversable(prohibited_grid, current_position, path[index])) {
      return TraversablePathTarget{path[index], index};
    }
  }

  return std::nullopt;
}

void selectPathTarget(TargetContinuityDecision& decision,
                      const TraversablePathTarget& target) {
  decision.target = target.target;
  decision.target_waypoint_index = target.waypoint_index;
  decision.target_waypoint_index_valid = true;
  decision.selected_target_traversable = true;
}

} // namespace

const char* targetContinuityDecisionReasonName(
    const TargetContinuityDecisionReason reason) noexcept {
  switch (reason) {
    case TargetContinuityDecisionReason::kNoPreviousTarget:
      return "no_previous_target";
    case TargetContinuityDecisionReason::kSwitchedToNewWaypoint:
      return "switched_to_new_waypoint";
    case TargetContinuityDecisionReason::kKeptPreviousTarget:
      return "kept_previous_target";
    case TargetContinuityDecisionReason::kForcedSwitchUnsafePrevious:
      return "forced_switch_unsafe_previous";
    case TargetContinuityDecisionReason::kForcedSwitchPreviousBehindPath:
      return "forced_switch_previous_behind_path";
  }
  return "unknown";
}

TargetContinuityDecision decideTargetAfterReplan(
    const std::span<const Point2> path, const Point2 current_position,
    const Point2 previous_target, const Point2 proposed_target,
    const bool had_previous_target, const std::size_t current_waypoint_index,
    const std::size_t fallback_waypoint_index, const double hysteresis_m,
    const OccupancyGrid2D* const prohibited_grid) {
  TargetContinuityDecision decision{};
  decision.target = proposed_target;
  decision.target_delta_m = had_previous_target
                                ? distance(previous_target, proposed_target)
                                : std::numeric_limits<double>::quiet_NaN();

  if (!had_previous_target || path.empty() || !(hysteresis_m > 0.0) ||
      !std::isfinite(hysteresis_m)) {
    decision.reason = TargetContinuityDecisionReason::kNoPreviousTarget;
    return decision;
  }

  const auto unrestricted_projection =
      closestOffboardPathProjection(path, previous_target, 0U);
  if (!unrestricted_projection.has_value()) {
    decision.reason = TargetContinuityDecisionReason::kSwitchedToNewWaypoint;
    return decision;
  }

  if (current_waypoint_index > 0U &&
      unrestricted_projection->segment_start_index + 1U < current_waypoint_index) {
    decision.path_error_m = std::sqrt(unrestricted_projection->distance_sq);
    decision.reason = TargetContinuityDecisionReason::kForcedSwitchPreviousBehindPath;
    return decision;
  }

  const std::size_t minimum_segment_start =
      current_waypoint_index > 0U ? current_waypoint_index - 1U : 0U;
  const auto projection =
      closestOffboardPathProjection(path, previous_target, minimum_segment_start);
  if (!projection.has_value()) {
    decision.reason = TargetContinuityDecisionReason::kSwitchedToNewWaypoint;
    return decision;
  }

  decision.path_error_m = std::sqrt(projection->distance_sq);
  if (decision.path_error_m > hysteresis_m) {
    decision.reason = TargetContinuityDecisionReason::kSwitchedToNewWaypoint;
    return decision;
  }

  if (prohibited_grid == nullptr) {
    decision.reason = TargetContinuityDecisionReason::kSwitchedToNewWaypoint;
    return decision;
  }
  decision.selected_target_traversable =
      pathSegmentIsTraversable(*prohibited_grid, current_position, proposed_target);
  if (!pathSegmentIsTraversable(*prohibited_grid, current_position, previous_target)) {
    decision.reason = TargetContinuityDecisionReason::kForcedSwitchUnsafePrevious;
    if (decision.selected_target_traversable) {
      return decision;
    }
    const std::optional<TraversablePathTarget> fallback_target =
        firstTraversablePathTarget(path, current_position, fallback_waypoint_index,
                                   *prohibited_grid);
    if (fallback_target.has_value()) {
      selectPathTarget(decision, *fallback_target);
      return decision;
    }
    if (pathSegmentIsTraversable(*prohibited_grid, current_position,
                                 current_position)) {
      decision.target = current_position;
      decision.selected_target_traversable = true;
    }
    return decision;
  }

  decision.previous_target_safe = true;
  decision.target = previous_target;
  decision.selected_target_traversable = true;
  decision.reason = TargetContinuityDecisionReason::kKeptPreviousTarget;
  return decision;
}

} // namespace drone_city_nav

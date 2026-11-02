#include "mission_monitor_node.hpp"
#include "mission_monitor_node_internal.hpp"

namespace drone_city_nav {

using namespace mission_monitor_node_internal;

// The operator's channel: the commands the monitor takes and what each does to the
// mission (moved whole on 2026-11-02, the source size contract).

void MissionMonitorNode::onOperatorCommand(const std::string& text) {
  const std::optional<OperatorCommand> command = parseOperatorCommand(text);
  if (!command.has_value() || result_reported_ || !navigation_mission_ready_) {
    return;
  }
  RCLCPP_WARN(get_logger(), "OPERATOR_COMMAND accepted=true command='%s'",
              text.c_str());
  publishMissionEvent("OPERATOR_COMMAND " + text);
  switch (command->kind) {
    case OperatorCommandKind::kStop:
      operator_hold_ = true;
      publishObjective(latest_map_position_, nextObjectiveEpoch(), 1U, true);
      break;
    case OperatorCommandKind::kGoal:
      operator_hold_ = false;
      goal_substituted_ = false;
      waypoints_ = {command->goal};
      goal_ = Point2{command->goal.x, command->goal.y};
      active_waypoint_index_ = 0U;
      completed_waypoint_count_ = 0U;
      minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
      publishObjective(command->goal, nextObjectiveEpoch(), 1U, false);
      break;
    case OperatorCommandKind::kHome:
      operator_hold_ = false;
      static_cast<void>(substituteGoalWithStart("operator", now().nanoseconds()));
      break;
    case OperatorCommandKind::kLand:
      operator_landing_ = true;
      break;
  }
}

} // namespace drone_city_nav

#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onMemorySnapshot(const msg::ObstacleMemorySnapshot& message) {
  MemoryProvenanceParseResult parsed = parseObstacleMemorySnapshotMessage(message);
  if (!parsed.snapshot.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid atomic obstacle memory snapshot: reason=%s detail=%s",
        memoryProvenanceUnavailableReasonName(parsed.reason), parsed.detail.c_str());
    return;
  }

  if (!applyMemoryGrid(message.grid)) {
    return;
  }
  memory_provenance_snapshot_ = std::move(*parsed.snapshot);
}

} // namespace drone_city_nav

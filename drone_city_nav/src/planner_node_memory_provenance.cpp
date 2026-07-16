#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onMemoryProvenance(const msg::ObstacleMemoryProvenance& message) {
  MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(message);
  if (!parsed.snapshot.has_value()) {
    latest_memory_provenance_error_ = parsed.reason;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid obstacle memory provenance snapshot: reason=%s",
        memoryProvenanceUnavailableReasonName(parsed.reason));
    return;
  }

  memory_provenance_cache_.insert(std::move(*parsed.snapshot));
  latest_memory_provenance_error_ = MemoryProvenanceUnavailableReason::kNone;
}

} // namespace drone_city_nav

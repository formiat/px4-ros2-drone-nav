#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

void logAuditOutcomes(const rclcpp::Logger& logger,
                      const std::vector<MemoryProvenanceAuditOutcome>& outcomes) {
  for (const MemoryProvenanceAuditOutcome& outcome : outcomes) {
    if (outcome.reason == MemoryProvenanceUnavailableReason::kNone) {
      RCLCPP_INFO(logger,
                  "Memory blocker provenance terminal: status=matched audit_id=%llu "
                  "snapshot_stamp_ns=%lld grid_hash=%llu occupied=%llu cell=(%d,%d) %s",
                  static_cast<unsigned long long>(outcome.audit_id),
                  static_cast<long long>(outcome.identity.stamp_ns),
                  static_cast<unsigned long long>(outcome.identity.raw_grid_data_hash),
                  static_cast<unsigned long long>(outcome.identity.occupied_cell_count),
                  outcome.cell.x, outcome.cell.y, outcome.diagnostic.c_str());
      continue;
    }
    RCLCPP_WARN(logger,
                "Memory blocker provenance terminal: status=unavailable audit_id=%llu "
                "reason=%s snapshot_stamp_ns=%lld grid_hash=%llu occupied=%llu "
                "cell=(%d,%d) %s",
                static_cast<unsigned long long>(outcome.audit_id),
                memoryProvenanceUnavailableReasonName(outcome.reason),
                static_cast<long long>(outcome.identity.stamp_ns),
                static_cast<unsigned long long>(outcome.identity.raw_grid_data_hash),
                static_cast<unsigned long long>(outcome.identity.occupied_cell_count),
                outcome.cell.x, outcome.cell.y, outcome.diagnostic.c_str());
  }
}

} // namespace

void PlannerNode::onMemoryProvenance(const msg::ObstacleMemoryProvenance& message) {
  MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(message);
  if (!parsed.snapshot.has_value()) {
    latest_memory_provenance_error_ = parsed.reason;
    if (const auto identity = memoryProvenanceMessageIdentity(message);
        identity.has_value()) {
      logAuditOutcomes(get_logger(),
                       memory_provenance_audit_tracker_.observeUnavailable(
                           *identity, parsed.reason));
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid obstacle memory provenance snapshot: reason=%s detail=%s",
        memoryProvenanceUnavailableReasonName(parsed.reason), parsed.detail.c_str());
    return;
  }

  const std::vector<MemoryProvenanceAuditOutcome> outcomes =
      memory_provenance_audit_tracker_.insert(std::move(*parsed.snapshot));
  latest_memory_provenance_error_ = MemoryProvenanceUnavailableReason::kNone;
  logAuditOutcomes(get_logger(), outcomes);
}

} // namespace drone_city_nav

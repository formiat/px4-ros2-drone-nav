#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onMemoryProvenance(const msg::ObstacleMemoryProvenance& message) {
  MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(message);
  if (!parsed.snapshot.has_value()) {
    latest_memory_provenance_error_ = parsed.reason;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring invalid obstacle memory provenance snapshot: reason=%s detail=%s",
        memoryProvenanceUnavailableReasonName(parsed.reason), parsed.detail.c_str());
    return;
  }

  const std::vector<MemoryProvenanceAuditEnrichment> enrichments =
      memory_provenance_audit_tracker_.insert(std::move(*parsed.snapshot));
  latest_memory_provenance_error_ = MemoryProvenanceUnavailableReason::kNone;
  for (const MemoryProvenanceAuditEnrichment& enrichment : enrichments) {
    RCLCPP_INFO(
        get_logger(),
        "Memory blocker provenance enrichment: audit_id=%llu "
        "snapshot_stamp_ns=%lld grid_hash=%llu occupied=%llu cell=(%d,%d) "
        "%s",
        static_cast<unsigned long long>(enrichment.audit_id),
        static_cast<long long>(enrichment.identity.stamp_ns),
        static_cast<unsigned long long>(enrichment.identity.raw_grid_data_hash),
        static_cast<unsigned long long>(enrichment.identity.occupied_cell_count),
        enrichment.cell.x, enrichment.cell.y, enrichment.diagnostic.c_str());
  }
}

} // namespace drone_city_nav

#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onMemorySnapshot(const msg::ObstacleMemorySnapshot& message) {
  const auto callback_started = std::chrono::steady_clock::now();
  const std::int64_t receive_ns = get_clock()->now().nanoseconds();
  ++memory_snapshot_received_;
  last_memory_snapshot_interval_ms_ =
      last_memory_snapshot_receive_ns_ > 0 &&
              receive_ns > last_memory_snapshot_receive_ns_
          ? static_cast<double>(receive_ns - last_memory_snapshot_receive_ns_) / 1.0e6
          : std::numeric_limits<double>::quiet_NaN();
  last_memory_snapshot_receive_ns_ = receive_ns;

  if (message.producer_instance_id == 0U || message.sequence == 0U) {
    ++memory_snapshot_rejected_;
    RCLCPP_WARN(get_logger(),
                "Ignoring obstacle memory snapshot with invalid transport identity: "
                "producer_instance=%" PRIu64 " sequence=%" PRIu64,
                message.producer_instance_id, message.sequence);
    logMemorySnapshotTransportSummary(receive_ns);
    return;
  }
  if (last_memory_snapshot_received_producer_instance_id_ != 0U &&
      message.producer_instance_id !=
          last_memory_snapshot_received_producer_instance_id_) {
    ++memory_snapshot_producer_restarts_;
    RCLCPP_WARN(get_logger(),
                "Obstacle memory snapshot producer changed: previous_instance=%" PRIu64
                " new_instance=%" PRIu64 " previous_sequence=%" PRIu64,
                last_memory_snapshot_received_producer_instance_id_,
                message.producer_instance_id, last_memory_snapshot_received_sequence_);
    last_memory_snapshot_received_sequence_ = 0U;
  }
  last_memory_snapshot_received_producer_instance_id_ = message.producer_instance_id;

  if (last_memory_snapshot_received_sequence_ > 0U &&
      message.sequence <= last_memory_snapshot_received_sequence_) {
    ++memory_snapshot_rejected_;
    ++memory_snapshot_out_of_order_;
    RCLCPP_WARN(get_logger(),
                "Ignoring non-monotonic obstacle memory snapshot: sequence=%" PRIu64
                " last_received_sequence=%" PRIu64,
                message.sequence, last_memory_snapshot_received_sequence_);
    logMemorySnapshotTransportSummary(receive_ns);
    return;
  }
  if (last_memory_snapshot_received_sequence_ > 0U &&
      message.sequence > last_memory_snapshot_received_sequence_ + 1U) {
    memory_snapshot_sequence_gaps_ +=
        message.sequence - last_memory_snapshot_received_sequence_ - 1U;
  }
  last_memory_snapshot_received_sequence_ = message.sequence;

  MemoryProvenanceParseResult parsed = parseObstacleMemorySnapshotMessage(message);
  if (!parsed.snapshot.has_value()) {
    ++memory_snapshot_rejected_;
    RCLCPP_WARN(get_logger(),
                "Ignoring invalid atomic obstacle memory snapshot: sequence=%" PRIu64
                " reason=%s detail=%s",
                message.sequence, memoryProvenanceUnavailableReasonName(parsed.reason),
                parsed.detail.c_str());
    logMemorySnapshotTransportSummary(receive_ns);
    return;
  }

  if (!applyMemoryGrid(message.grid)) {
    ++memory_snapshot_rejected_;
    RCLCPP_WARN(get_logger(),
                "Ignoring atomic obstacle memory snapshot with invalid grid: "
                "sequence=%" PRIu64,
                message.sequence);
    logMemorySnapshotTransportSummary(receive_ns);
    return;
  }
  memory_provenance_snapshot_ = std::move(*parsed.snapshot);
  last_memory_snapshot_applied_producer_instance_id_ = message.producer_instance_id;
  last_memory_snapshot_applied_sequence_ = message.sequence;
  last_memory_snapshot_stamp_ns_ =
      static_cast<std::int64_t>(stampNanoseconds(message.grid.header.stamp));
  last_memory_snapshot_age_ms_ =
      last_memory_snapshot_stamp_ns_ > 0 && receive_ns > last_memory_snapshot_stamp_ns_
          ? static_cast<double>(receive_ns - last_memory_snapshot_stamp_ns_) / 1.0e6
          : 0.0;
  last_memory_snapshot_callback_ms_ = elapsedMilliseconds(callback_started);
  ++memory_snapshot_applied_;
  memory_snapshot_max_age_since_report_ms_ =
      std::max(memory_snapshot_max_age_since_report_ms_, last_memory_snapshot_age_ms_);
  memory_snapshot_max_callback_since_report_ms_ = std::max(
      memory_snapshot_max_callback_since_report_ms_, last_memory_snapshot_callback_ms_);

  RCLCPP_INFO(get_logger(),
              "Planner memory snapshot applied: producer_instance=%" PRIu64
              " sequence=%" PRIu64 " stamp_ns=%" PRId64
              " age_ms=%.3f callback_ms=%.3f interval_ms=%.3f "
              "producer_assembly_ms=%.3f received=%" PRIu64 " applied=%" PRIu64
              " replacements=%" PRIu64 " rejected=%" PRIu64,
              message.producer_instance_id, message.sequence,
              last_memory_snapshot_stamp_ns_, last_memory_snapshot_age_ms_,
              last_memory_snapshot_callback_ms_, last_memory_snapshot_interval_ms_,
              static_cast<double>(message.producer_assembly_duration_ns) / 1.0e6,
              memory_snapshot_received_, memory_snapshot_applied_,
              memory_snapshot_sequence_gaps_, memory_snapshot_rejected_);
  logMemorySnapshotTransportSummary(receive_ns);
}

void PlannerNode::logMemorySnapshotTransportSummary(const std::int64_t now_ns) {
  if (last_memory_snapshot_diagnostic_ns_ <= 0) {
    last_memory_snapshot_diagnostic_ns_ = now_ns;
    memory_snapshot_applied_at_last_diagnostic_ = memory_snapshot_applied_;
    return;
  }
  const double elapsed_s =
      now_ns > last_memory_snapshot_diagnostic_ns_
          ? static_cast<double>(now_ns - last_memory_snapshot_diagnostic_ns_) / 1.0e9
          : 0.0;
  if (elapsed_s < memory_snapshot_diagnostic_period_s_) {
    return;
  }

  const std::uint64_t applied_since_report =
      memory_snapshot_applied_ - memory_snapshot_applied_at_last_diagnostic_;
  last_memory_snapshot_apply_rate_hz_ =
      elapsed_s > 0.0 ? static_cast<double>(applied_since_report) / elapsed_s : 0.0;
  const bool within_budget =
      memory_snapshot_max_age_since_report_ms_ <= memory_snapshot_max_age_ms_ &&
      memory_snapshot_max_callback_since_report_ms_ <=
          memory_snapshot_max_callback_time_ms_ &&
      last_memory_snapshot_apply_rate_hz_ >= memory_snapshot_min_apply_rate_hz_;
  const char* status = within_budget ? "within_budget" : "exceeded";
  if (within_budget) {
    RCLCPP_INFO(get_logger(),
                "Planner memory snapshot budget: status=%s producer_instance=%" PRIu64
                " last_sequence=%" PRIu64 " received=%" PRIu64 " applied=%" PRIu64
                " replacements=%" PRIu64 " rejected=%" PRIu64 " out_of_order=%" PRIu64
                " apply_rate_hz=%.3f max_age_ms=%.3f max_callback_ms=%.3f",
                status, last_memory_snapshot_applied_producer_instance_id_,
                last_memory_snapshot_applied_sequence_, memory_snapshot_received_,
                memory_snapshot_applied_, memory_snapshot_sequence_gaps_,
                memory_snapshot_rejected_, memory_snapshot_out_of_order_,
                last_memory_snapshot_apply_rate_hz_,
                memory_snapshot_max_age_since_report_ms_,
                memory_snapshot_max_callback_since_report_ms_);
  } else {
    RCLCPP_WARN(get_logger(),
                "Planner memory snapshot budget: status=%s producer_instance=%" PRIu64
                " last_sequence=%" PRIu64 " received=%" PRIu64 " applied=%" PRIu64
                " replacements=%" PRIu64 " rejected=%" PRIu64 " out_of_order=%" PRIu64
                " apply_rate_hz=%.3f min_apply_rate_hz=%.3f max_age_ms=%.3f "
                "age_budget_ms=%.3f max_callback_ms=%.3f callback_budget_ms=%.3f",
                status, last_memory_snapshot_applied_producer_instance_id_,
                last_memory_snapshot_applied_sequence_, memory_snapshot_received_,
                memory_snapshot_applied_, memory_snapshot_sequence_gaps_,
                memory_snapshot_rejected_, memory_snapshot_out_of_order_,
                last_memory_snapshot_apply_rate_hz_, memory_snapshot_min_apply_rate_hz_,
                memory_snapshot_max_age_since_report_ms_, memory_snapshot_max_age_ms_,
                memory_snapshot_max_callback_since_report_ms_,
                memory_snapshot_max_callback_time_ms_);
  }
  last_memory_snapshot_diagnostic_ns_ = now_ns;
  memory_snapshot_applied_at_last_diagnostic_ = memory_snapshot_applied_;
  memory_snapshot_max_age_since_report_ms_ = 0.0;
  memory_snapshot_max_callback_since_report_ms_ = 0.0;
}

std::string
PlannerNode::memorySnapshotTransportDiagnostic(const std::int64_t now_ns) const {
  const double current_age_ms =
      last_memory_snapshot_stamp_ns_ > 0 && now_ns > last_memory_snapshot_stamp_ns_
          ? static_cast<double>(now_ns - last_memory_snapshot_stamp_ns_) / 1.0e6
          : std::numeric_limits<double>::quiet_NaN();
  std::ostringstream stream;
  stream << "memory_snapshot_transport[producer_instance="
         << last_memory_snapshot_applied_producer_instance_id_
         << " sequence=" << last_memory_snapshot_applied_sequence_
         << " stamp_ns=" << last_memory_snapshot_stamp_ns_
         << " current_age_ms=" << current_age_ms
         << " callback_age_ms=" << last_memory_snapshot_age_ms_
         << " callback_ms=" << last_memory_snapshot_callback_ms_
         << " interval_ms=" << last_memory_snapshot_interval_ms_
         << " apply_rate_hz=" << last_memory_snapshot_apply_rate_hz_
         << " received=" << memory_snapshot_received_
         << " applied=" << memory_snapshot_applied_
         << " replacements=" << memory_snapshot_sequence_gaps_
         << " rejected=" << memory_snapshot_rejected_
         << " out_of_order=" << memory_snapshot_out_of_order_
         << " producer_restarts=" << memory_snapshot_producer_restarts_ << ']';
  return stream.str();
}

} // namespace drone_city_nav

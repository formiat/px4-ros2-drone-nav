#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onMemorySnapshot(
    msg::ObstacleMemorySnapshot::ConstSharedPtr message) {
  const auto callback_started = std::chrono::steady_clock::now();
  const std::int64_t receive_ns = get_clock()->now().nanoseconds();

  {
    std::scoped_lock lock{memory_snapshot_mutex_};
    ++memory_snapshot_received_;
    last_memory_snapshot_interval_ms_ =
        last_memory_snapshot_receive_ns_ > 0 &&
                receive_ns > last_memory_snapshot_receive_ns_
            ? static_cast<double>(receive_ns - last_memory_snapshot_receive_ns_) / 1.0e6
            : std::numeric_limits<double>::quiet_NaN();
    last_memory_snapshot_receive_ns_ = receive_ns;

    if (message->producer_instance_id == 0U || message->sequence == 0U) {
      ++memory_snapshot_rejected_;
      RCLCPP_WARN(get_logger(),
                  "Ignoring obstacle memory snapshot with invalid transport "
                  "identity: producer_instance=%" PRIu64 " sequence=%" PRIu64,
                  message->producer_instance_id, message->sequence);
      return;
    }
    if (last_memory_snapshot_received_producer_instance_id_ != 0U &&
        message->producer_instance_id !=
            last_memory_snapshot_received_producer_instance_id_) {
      ++memory_snapshot_producer_restarts_;
      RCLCPP_WARN(
          get_logger(),
          "Obstacle memory snapshot producer changed: previous_instance=%" PRIu64
          " new_instance=%" PRIu64 " previous_sequence=%" PRIu64,
          last_memory_snapshot_received_producer_instance_id_,
          message->producer_instance_id, last_memory_snapshot_received_sequence_);
      last_memory_snapshot_received_sequence_ = 0U;
    }
    last_memory_snapshot_received_producer_instance_id_ = message->producer_instance_id;

    if (last_memory_snapshot_received_sequence_ > 0U &&
        message->sequence <= last_memory_snapshot_received_sequence_) {
      ++memory_snapshot_rejected_;
      ++memory_snapshot_out_of_order_;
      RCLCPP_WARN(get_logger(),
                  "Ignoring non-monotonic obstacle memory snapshot: sequence=%" PRIu64
                  " last_received_sequence=%" PRIu64,
                  message->sequence, last_memory_snapshot_received_sequence_);
      return;
    }
    if (last_memory_snapshot_received_sequence_ > 0U &&
        message->sequence > last_memory_snapshot_received_sequence_ + 1U) {
      memory_snapshot_sequence_gaps_ +=
          message->sequence - last_memory_snapshot_received_sequence_ - 1U;
    }
    last_memory_snapshot_received_sequence_ = message->sequence;
  }

  MemoryProvenanceParseResult parsed = parseObstacleMemorySnapshotMessage(*message);
  if (!parsed.snapshot.has_value()) {
    {
      std::scoped_lock lock{memory_snapshot_mutex_};
      ++memory_snapshot_rejected_;
    }
    RCLCPP_WARN(get_logger(),
                "Ignoring invalid atomic obstacle memory snapshot: sequence=%" PRIu64
                " reason=%s detail=%s",
                message->sequence, memoryProvenanceUnavailableReasonName(parsed.reason),
                parsed.detail.c_str());
    return;
  }

  RawOccupancyGridFromRosResult converted = rawOccupancyGridFromRos(
      message->grid,
      RawOccupancyGridFromRosConfig{memory_occupied_value_, memory_free_value_});
  if (!converted.grid.has_value()) {
    {
      std::scoped_lock lock{memory_snapshot_mutex_};
      ++memory_snapshot_rejected_;
    }
    if (converted.error == OccupancyGridFromRosError::kMismatchedDataSize) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring obstacle memory snapshot grid with mismatched data "
                  "size: sequence=%" PRIu64 " expected=%zu got=%zu",
                  message->sequence, converted.expected_data_size,
                  converted.actual_data_size);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Ignoring obstacle memory snapshot with invalid grid metadata: "
                  "sequence=%" PRIu64,
                  message->sequence);
    }
    return;
  }
  if (converted.intermediate_value_cells > 0U) {
    RCLCPP_WARN(get_logger(),
                "Ignored intermediate values while reading raw obstacle memory "
                "snapshot: sequence=%" PRIu64
                " intermediate_cells=%zu occupied=%d free=%d",
                message->sequence, converted.intermediate_value_cells,
                memory_occupied_value_, memory_free_value_);
  }

  const std::int64_t stamp_ns =
      static_cast<std::int64_t>(stampNanoseconds(message->grid.header.stamp));
  const double receive_age_ms = stamp_ns > 0 && receive_ns > stamp_ns
                                    ? static_cast<double>(receive_ns - stamp_ns) / 1.0e6
                                    : 0.0;
  const double callback_ms = elapsedMilliseconds(callback_started);
  std::uint64_t received = 0U;
  std::uint64_t pending_replacements = 0U;
  std::uint64_t dds_sequence_gaps = 0U;
  {
    std::scoped_lock lock{memory_snapshot_mutex_};
    if (pending_memory_snapshot_.has_value()) {
      ++memory_snapshot_pending_replacements_;
    }
    pending_memory_snapshot_ = PendingMemorySnapshot{
        .grid = std::move(*converted.grid),
        .provenance = std::move(*parsed.snapshot),
        .producer_instance_id = message->producer_instance_id,
        .sequence = message->sequence,
        .producer_assembly_duration_ns = message->producer_assembly_duration_ns,
        .stamp_ns = stamp_ns,
        .receive_ns = receive_ns,
        .receive_age_ms = receive_age_ms,
        .callback_ms = callback_ms};
    memory_snapshot_max_callback_since_report_ms_ =
        std::max(memory_snapshot_max_callback_since_report_ms_, callback_ms);
    received = memory_snapshot_received_;
    pending_replacements = memory_snapshot_pending_replacements_;
    dds_sequence_gaps = memory_snapshot_sequence_gaps_;
  }

  RCLCPP_INFO(get_logger(),
              "Planner memory snapshot queued: producer_instance=%" PRIu64
              " sequence=%" PRIu64 " stamp_ns=%" PRId64
              " receive_age_ms=%.3f callback_ms=%.3f interval_ms=%.3f "
              "producer_assembly_ms=%.3f received=%" PRIu64
              " pending_replacements=%" PRIu64 " dds_sequence_gaps=%" PRIu64,
              message->producer_instance_id, message->sequence, stamp_ns,
              receive_age_ms, callback_ms, last_memory_snapshot_interval_ms_,
              static_cast<double>(message->producer_assembly_duration_ns) / 1.0e6,
              received, pending_replacements, dds_sequence_gaps);
}

void PlannerNode::applyPendingMemorySnapshot(const std::int64_t now_ns) {
  std::optional<PendingMemorySnapshot> pending;
  {
    std::scoped_lock lock{memory_snapshot_mutex_};
    if (pending_memory_snapshot_.has_value()) {
      pending = std::move(pending_memory_snapshot_);
      pending_memory_snapshot_.reset();
    }
  }
  if (!pending.has_value()) {
    logMemorySnapshotTransportSummary(now_ns);
    return;
  }

  memory_grid_ = std::move(pending->grid);
  memory_provenance_snapshot_ = std::move(pending->provenance);
  if (!memory_grid_seen_) {
    memory_grid_seen_ = true;
    RCLCPP_INFO(get_logger(),
                "First obstacle memory grid: size=%dx%d resolution=%.2f "
                "origin=(%.2f, %.2f)",
                memory_grid_->width(), memory_grid_->height(),
                memory_grid_->resolution(), memory_grid_->originX(),
                memory_grid_->originY());
  }

  const double apply_age_ms =
      pending->stamp_ns > 0 && now_ns > pending->stamp_ns
          ? static_cast<double>(now_ns - pending->stamp_ns) / 1.0e6
          : 0.0;
  const double apply_delay_ms =
      pending->receive_ns > 0 && now_ns > pending->receive_ns
          ? static_cast<double>(now_ns - pending->receive_ns) / 1.0e6
          : 0.0;
  std::uint64_t applied = 0U;
  std::uint64_t received = 0U;
  std::uint64_t pending_replacements = 0U;
  std::uint64_t rejected = 0U;
  {
    std::scoped_lock lock{memory_snapshot_mutex_};
    last_memory_snapshot_applied_producer_instance_id_ = pending->producer_instance_id;
    last_memory_snapshot_applied_sequence_ = pending->sequence;
    last_memory_snapshot_stamp_ns_ = pending->stamp_ns;
    last_memory_snapshot_age_ms_ = apply_age_ms;
    last_memory_snapshot_receive_age_ms_ = pending->receive_age_ms;
    last_memory_snapshot_callback_ms_ = pending->callback_ms;
    last_memory_snapshot_apply_delay_ms_ = apply_delay_ms;
    ++memory_snapshot_applied_;
    memory_snapshot_max_age_since_report_ms_ =
        std::max(memory_snapshot_max_age_since_report_ms_, apply_age_ms);
    memory_snapshot_max_apply_delay_since_report_ms_ =
        std::max(memory_snapshot_max_apply_delay_since_report_ms_, apply_delay_ms);
    applied = memory_snapshot_applied_;
    received = memory_snapshot_received_;
    pending_replacements = memory_snapshot_pending_replacements_;
    rejected = memory_snapshot_rejected_;
  }

  RCLCPP_INFO(
      get_logger(),
      "Planner memory snapshot applied: producer_instance=%" PRIu64 " sequence=%" PRIu64
      " stamp_ns=%" PRId64 " receive_age_ms=%.3f apply_age_ms=%.3f callback_ms=%.3f "
      "apply_delay_ms=%.3f producer_assembly_ms=%.3f received=%" PRIu64
      " applied=%" PRIu64 " pending_replacements=%" PRIu64 " rejected=%" PRIu64,
      pending->producer_instance_id, pending->sequence, pending->stamp_ns,
      pending->receive_age_ms, apply_age_ms, pending->callback_ms, apply_delay_ms,
      static_cast<double>(pending->producer_assembly_duration_ns) / 1.0e6, received,
      applied, pending_replacements, rejected);
  logMemorySnapshotTransportSummary(now_ns);
}

void PlannerNode::logMemorySnapshotTransportSummary(const std::int64_t now_ns) {
  std::scoped_lock lock{memory_snapshot_mutex_};
  if (memory_snapshot_received_ == 0U || memory_snapshot_applied_ == 0U) {
    last_memory_snapshot_diagnostic_ns_ = now_ns;
    memory_snapshot_applied_at_last_diagnostic_ = memory_snapshot_applied_;
    memory_snapshot_received_at_last_diagnostic_ = memory_snapshot_received_;
    memory_snapshot_max_age_since_report_ms_ = 0.0;
    memory_snapshot_max_callback_since_report_ms_ = 0.0;
    memory_snapshot_max_apply_delay_since_report_ms_ = 0.0;
    return;
  }
  if (last_memory_snapshot_diagnostic_ns_ <= 0) {
    last_memory_snapshot_diagnostic_ns_ = now_ns;
    memory_snapshot_applied_at_last_diagnostic_ = memory_snapshot_applied_;
    memory_snapshot_received_at_last_diagnostic_ = memory_snapshot_received_;
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
  const std::uint64_t received_since_report =
      memory_snapshot_received_ - memory_snapshot_received_at_last_diagnostic_;
  last_memory_snapshot_apply_rate_hz_ =
      elapsed_s > 0.0 ? static_cast<double>(applied_since_report) / elapsed_s : 0.0;
  last_memory_snapshot_receive_rate_hz_ =
      elapsed_s > 0.0 ? static_cast<double>(received_since_report) / elapsed_s : 0.0;
  const bool within_budget =
      memory_snapshot_max_age_since_report_ms_ <= memory_snapshot_max_age_ms_ &&
      memory_snapshot_max_callback_since_report_ms_ <=
          memory_snapshot_max_callback_time_ms_ &&
      memory_snapshot_max_apply_delay_since_report_ms_ <=
          memory_snapshot_max_apply_delay_ms_ &&
      last_memory_snapshot_apply_rate_hz_ >= memory_snapshot_min_apply_rate_hz_;
  const char* status = within_budget ? "within_budget" : "exceeded";
  if (within_budget) {
    RCLCPP_INFO(get_logger(),
                "Planner memory snapshot budget: status=%s producer_instance=%" PRIu64
                " last_sequence=%" PRIu64 " received=%" PRIu64 " applied=%" PRIu64
                " pending_replacements=%" PRIu64 " dds_sequence_gaps=%" PRIu64
                " rejected=%" PRIu64 " out_of_order=%" PRIu64
                " receive_rate_hz=%.3f apply_rate_hz=%.3f min_apply_rate_hz=%.3f "
                "max_apply_age_ms=%.3f age_budget_ms=%.3f max_callback_ms=%.3f "
                "callback_budget_ms=%.3f max_apply_delay_ms=%.3f "
                "apply_delay_budget_ms=%.3f",
                status, last_memory_snapshot_applied_producer_instance_id_,
                last_memory_snapshot_applied_sequence_, memory_snapshot_received_,
                memory_snapshot_applied_, memory_snapshot_pending_replacements_,
                memory_snapshot_sequence_gaps_, memory_snapshot_rejected_,
                memory_snapshot_out_of_order_, last_memory_snapshot_receive_rate_hz_,
                last_memory_snapshot_apply_rate_hz_, memory_snapshot_min_apply_rate_hz_,
                memory_snapshot_max_age_since_report_ms_, memory_snapshot_max_age_ms_,
                memory_snapshot_max_callback_since_report_ms_,
                memory_snapshot_max_callback_time_ms_,
                memory_snapshot_max_apply_delay_since_report_ms_,
                memory_snapshot_max_apply_delay_ms_);
  } else {
    RCLCPP_WARN(get_logger(),
                "Planner memory snapshot budget: status=%s producer_instance=%" PRIu64
                " last_sequence=%" PRIu64 " received=%" PRIu64 " applied=%" PRIu64
                " pending_replacements=%" PRIu64 " dds_sequence_gaps=%" PRIu64
                " rejected=%" PRIu64 " out_of_order=%" PRIu64
                " receive_rate_hz=%.3f apply_rate_hz=%.3f min_apply_rate_hz=%.3f "
                "max_apply_age_ms=%.3f age_budget_ms=%.3f max_callback_ms=%.3f "
                "callback_budget_ms=%.3f max_apply_delay_ms=%.3f "
                "apply_delay_budget_ms=%.3f",
                status, last_memory_snapshot_applied_producer_instance_id_,
                last_memory_snapshot_applied_sequence_, memory_snapshot_received_,
                memory_snapshot_applied_, memory_snapshot_pending_replacements_,
                memory_snapshot_sequence_gaps_, memory_snapshot_rejected_,
                memory_snapshot_out_of_order_, last_memory_snapshot_receive_rate_hz_,
                last_memory_snapshot_apply_rate_hz_, memory_snapshot_min_apply_rate_hz_,
                memory_snapshot_max_age_since_report_ms_, memory_snapshot_max_age_ms_,
                memory_snapshot_max_callback_since_report_ms_,
                memory_snapshot_max_callback_time_ms_,
                memory_snapshot_max_apply_delay_since_report_ms_,
                memory_snapshot_max_apply_delay_ms_);
  }
  last_memory_snapshot_diagnostic_ns_ = now_ns;
  memory_snapshot_applied_at_last_diagnostic_ = memory_snapshot_applied_;
  memory_snapshot_received_at_last_diagnostic_ = memory_snapshot_received_;
  memory_snapshot_max_age_since_report_ms_ = 0.0;
  memory_snapshot_max_callback_since_report_ms_ = 0.0;
  memory_snapshot_max_apply_delay_since_report_ms_ = 0.0;
}

std::string
PlannerNode::memorySnapshotTransportDiagnostic(const std::int64_t now_ns) const {
  std::scoped_lock lock{memory_snapshot_mutex_};
  const double current_age_ms =
      last_memory_snapshot_stamp_ns_ > 0 && now_ns > last_memory_snapshot_stamp_ns_
          ? static_cast<double>(now_ns - last_memory_snapshot_stamp_ns_) / 1.0e6
          : std::numeric_limits<double>::quiet_NaN();
  const std::uint64_t pending_sequence =
      pending_memory_snapshot_.has_value() ? pending_memory_snapshot_->sequence : 0U;
  const double pending_age_ms =
      pending_memory_snapshot_.has_value() && pending_memory_snapshot_->stamp_ns > 0 &&
              now_ns > pending_memory_snapshot_->stamp_ns
          ? static_cast<double>(now_ns - pending_memory_snapshot_->stamp_ns) / 1.0e6
          : std::numeric_limits<double>::quiet_NaN();
  std::ostringstream stream;
  stream << "memory_snapshot_transport[producer_instance="
         << last_memory_snapshot_applied_producer_instance_id_
         << " active_sequence=" << last_memory_snapshot_applied_sequence_
         << " active_stamp_ns=" << last_memory_snapshot_stamp_ns_
         << " active_current_age_ms=" << current_age_ms
         << " active_apply_age_ms=" << last_memory_snapshot_age_ms_
         << " receive_age_ms=" << last_memory_snapshot_receive_age_ms_
         << " callback_ms=" << last_memory_snapshot_callback_ms_
         << " apply_delay_ms=" << last_memory_snapshot_apply_delay_ms_
         << " pending_sequence=" << pending_sequence
         << " pending_age_ms=" << pending_age_ms
         << " receive_rate_hz=" << last_memory_snapshot_receive_rate_hz_
         << " apply_rate_hz=" << last_memory_snapshot_apply_rate_hz_
         << " received=" << memory_snapshot_received_
         << " applied=" << memory_snapshot_applied_
         << " pending_replacements=" << memory_snapshot_pending_replacements_
         << " dds_sequence_gaps=" << memory_snapshot_sequence_gaps_
         << " rejected=" << memory_snapshot_rejected_
         << " out_of_order=" << memory_snapshot_out_of_order_
         << " producer_restarts=" << memory_snapshot_producer_restarts_ << ']';
  return stream.str();
}

} // namespace drone_city_nav

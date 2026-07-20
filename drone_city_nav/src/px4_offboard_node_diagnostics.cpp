#include "drone_city_nav/offboard_trajectory_delivery_diagnostics.hpp"

#include "px4_offboard_node.hpp"
#include "scoped_offboard_callback_duration.hpp"

namespace drone_city_nav {

void Px4OffboardNode::onPathId(const std_msgs::msg::UInt64& msg) {
  if (crashed_) {
    return;
  }
  latest_planner_path_id_ = msg.data;
  latest_planner_path_id_seen_ = true;
}

void Px4OffboardNode::onTrajectoryDiagnostics(const std_msgs::msg::String& msg) {
  if (crashed_) {
    return;
  }
  ScopedOffboardCallbackDuration callback_duration{
      get_logger(), "trajectory_diagnostics", msg.data.size()};
  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> diagnostics =
      parseTrajectoryPlannerDiagnosticsJson(msg.data);
  if (!diagnostics.has_value()) {
    callback_duration.setOutcome("malformed");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring malformed trajectory diagnostics message: bytes=%zu",
                         msg.data.size());
    return;
  }

  latest_trajectory_diagnostics_ = diagnostics;
  const auto receipt =
      std::find_if(recent_path_receipts_.rbegin(), recent_path_receipts_.rend(),
                   [&diagnostics](const PathReceiptDiagnostic& candidate) {
                     return candidate.path_stamp_ns == diagnostics->path_stamp_ns;
                   });
  if (receipt != recent_path_receipts_.rend()) {
    const std::int64_t diagnostics_receive_stamp_ns = get_clock()->now().nanoseconds();
    const std::string correlated_delivery = formatTrajectoryDeliveryAtReceive(
        &diagnostics->delivery, diagnostics->path_stamp_ns, receipt->receive_stamp_ns,
        receipt->position);
    const double diagnostics_after_path_ms =
        receipt->receive_stamp_ns > 0
            ? 1.0e-6 * static_cast<double>(diagnostics_receive_stamp_ns -
                                           receipt->receive_stamp_ns)
            : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(
        get_logger(),
        "REPLAN_DELIVERY event=diagnostics_correlated_to_path_receipt "
        "planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
        " path_receive_stamp_ns=%" PRId64 " diagnostics_receive_stamp_ns=%" PRId64
        " diagnostics_after_path_ms=%.1f points=%zu %s",
        diagnostics->planner_path_id, diagnostics->path_stamp_ns,
        receipt->receive_stamp_ns, diagnostics_receive_stamp_ns,
        diagnostics_after_path_ms, receipt->point_count, correlated_delivery.c_str());
  }
  if (!path_valid_ || !trajectoryDiagnosticsMatchesCurrentPath(*diagnostics)) {
    callback_duration.setTrajectoryIdentity(diagnostics->planner_path_id,
                                            diagnostics->path_stamp_ns);
    callback_duration.setOutcome("not_current_path");
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const std::string delivery_diagnostic = formatTrajectoryDeliveryAtReceive(
        &diagnostics->delivery, diagnostics->path_stamp_ns, receive_stamp_ns,
        current_position_);
    RCLCPP_INFO(get_logger(),
                "REPLAN_DELIVERY event=diagnostics_received_not_current "
                "planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " path_valid=%s current_path_stamp_ns=%" PRIu64 " %s",
                diagnostics->planner_path_id, diagnostics->path_stamp_ns,
                path_valid_ ? "true" : "false", last_received_path_stamp_ns_,
                delivery_diagnostic.c_str());
    return;
  }

  callback_duration.setTrajectoryIdentity(diagnostics->planner_path_id,
                                          diagnostics->path_stamp_ns);
  mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*diagnostics);
  const bool runtime_speed_policy_mismatch = configFingerprintMismatch(
      last_trajectory_planner_stats_.runtime_speed_policy_config_fingerprint,
      diagnostics->stats.runtime_speed_policy_config_fingerprint);
  const bool runtime_velocity_control_mismatch = configFingerprintMismatch(
      last_trajectory_planner_stats_.runtime_velocity_control_config_fingerprint,
      diagnostics->stats.runtime_velocity_control_config_fingerprint);
  RCLCPP_INFO(get_logger(),
              "Applied planner trajectory diagnostics: planner_path_id=%" PRIu64
              " path_stamp_ns=%" PRIu64 " corridor_width[min=%.2f mean=%.2f max=%.2f] "
              "optimizer[length=%.2f time=%.2f max_offset=%.2f] "
              "runtime_fingerprint_mismatch[speed_policy=%s velocity_control=%s]",
              diagnostics->planner_path_id, diagnostics->path_stamp_ns,
              diagnostics->stats.corridor.min_width_m,
              diagnostics->stats.corridor.mean_width_m,
              diagnostics->stats.corridor.max_width_m,
              diagnostics->stats.trajectory_optimizer.final_length_m,
              diagnostics->stats.trajectory_optimizer.estimated_time_s,
              diagnostics->stats.trajectory_optimizer.max_abs_offset_m,
              runtime_speed_policy_mismatch ? "true" : "false",
              runtime_velocity_control_mismatch ? "true" : "false");
  callback_duration.setOutcome("applied");
}

} // namespace drone_city_nav

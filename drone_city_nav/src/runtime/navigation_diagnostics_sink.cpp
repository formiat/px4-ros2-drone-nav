#include "navigation_diagnostics_sink.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {

NavigationDiagnosticsSink::NavigationDiagnosticsSink(
    NavigationDiagnosticsSinkConfig config, SnapshotProcessor processor,
    ProcessingFailureHandler failure_handler)
    : config_{std::move(config)},
      processor_{std::move(processor)},
      failure_handler_{std::move(failure_handler)} {
  if (!processor_) {
    throw std::invalid_argument{"navigation diagnostics processor is required"};
  }
  if (config_.output_directory.empty() || config_.file_period_ns <= 0 ||
      !(config_.flush_period.count() > 0.0) || config_.error_ring_capacity == 0U ||
      config_.configured_rollouts == 0U || !(config_.deadline_ms > 0.0)) {
    throw std::invalid_argument{"invalid navigation diagnostics sink configuration"};
  }
  std::filesystem::create_directories(config_.output_directory);
  diagnostics_stream_.open(config_.output_directory / "mppi_ticks.jsonl",
                           std::ios::trunc);
  diagnostics_error_stream_.open(config_.output_directory / "mppi_error_context.jsonl",
                                 std::ios::trunc);
  last_flush_time_ = std::chrono::steady_clock::now();
}

NavigationDiagnosticsSink::~NavigationDiagnosticsSink() {
  stop();
}

void NavigationDiagnosticsSink::start() {
  const std::scoped_lock lock{lifecycle_mutex_};
  if (worker_.joinable()) {
    return;
  }
  accepting_.store(true, std::memory_order_release);
  worker_ = std::jthread{[this](const std::stop_token stop_token) { run(stop_token); }};
}

void NavigationDiagnosticsSink::stop() noexcept {
  const std::scoped_lock lock{lifecycle_mutex_};
  accepting_.store(false, std::memory_order_release);
  if (worker_.joinable()) {
    worker_.request_stop();
    mailbox_.notifyAll();
    worker_.join();
  }
  const std::scoped_lock file_lock{file_mutex_};
  flushFilesLocked();
}

bool NavigationDiagnosticsSink::enqueue(ProductionMppiDiagnosticsSnapshot snapshot) {
  const std::scoped_lock lock{lifecycle_mutex_};
  if (!accepting_.load(std::memory_order_acquire)) {
    rejected_after_stop_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  if (mailbox_.push(std::move(snapshot))) {
    dropped_snapshots_.fetch_add(1U, std::memory_order_relaxed);
  }
  return true;
}

void NavigationDiagnosticsSink::recordTick(
    const mppi::MppiTickResult& result,
    const ProductionMppiPlanningState planning_state,
    const ProductionMppiExecutionPublication& execution,
    const bool liveness_reseed_requested,
    const RollingRouteTelemetryObservation3D& rolling_route,
    const ProductionMppiTickPhaseTimings& phases) {
  const std::scoped_lock lock{statistics_mutex_};
  ++statistics_.completed_ticks;
  statistics_.runtime_samples_ms.push_back(result.timings.host_total_ms);
  statistics_.snapshot_phase_samples_ms.push_back(phases.snapshot_ms);
  statistics_.controller_phase_samples_ms.push_back(phases.controller_ms);
  statistics_.publication_phase_samples_ms.push_back(phases.publication_ms);
  statistics_.tick_total_samples_ms.push_back(phases.total_ms);
  statistics_.deadline_misses +=
      result.timings.host_total_ms > config_.deadline_ms ? 1U : 0U;
  statistics_.altitude_envelope_violation_horizons +=
      result.altitude_envelope_violation ? 1U : 0U;
  statistics_.post_update_contract_violations +=
      planning_state == ProductionMppiPlanningState::kPlanned &&
              !result.post_update_classification.executable
          ? 1U
          : 0U;
  statistics_.no_progress_horizons += result.head_progress_m <= 0.0F ? 1U : 0U;
  statistics_.liveness_reseeds += liveness_reseed_requested ? 1U : 0U;
  statistics_.mission_goal_position_hold_ticks +=
      planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold ? 1U : 0U;
  statistics_.no_executable_route_hold_ticks +=
      planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold ? 1U : 0U;
  statistics_.no_executable_horizon_hold_ticks +=
      execution.reason == ProductionMppiExecutionReason::kNoExecutableHorizon &&
              !execution.resident_owner_continues
          ? 1U
          : 0U;
  statistics_.resident_owner_continuation_ticks +=
      execution.resident_owner_continues ? 1U : 0U;
  statistics_.terminal_rest_horizon_ticks += execution.terminal_rest_state ? 1U : 0U;
  statistics_.finite_path_validation_backoff_ticks +=
      execution.finite_path_validation_backoff ? 1U : 0U;
  statistics_.latest_lidar_path_validation_backoff_ticks +=
      execution.latest_lidar_path_validation_backoff ? 1U : 0U;
  statistics_.retained_previous_finite_path_ticks +=
      execution.retained_previous_finite_path ? 1U : 0U;
  statistics_.arrival_control_total += execution.arrival_control_count;
  statistics_.arrival_shaping_attempt_total += execution.arrival_shaping_attempts;
  if (result.active_rollouts > 0U) {
    statistics_.active_rollout_total += result.active_rollouts;
    statistics_.full_rollout_ticks +=
        result.active_rollouts == config_.configured_rollouts ? 1U : 0U;
    statistics_.reduced_rollout_ticks +=
        result.active_rollouts < config_.configured_rollouts ? 1U : 0U;
  }
  rolling_route_telemetry_.observe(rolling_route);
}

NavigationDiagnosticsStatistics NavigationDiagnosticsSink::statistics() const {
  const std::scoped_lock lock{statistics_mutex_};
  NavigationDiagnosticsStatistics snapshot = statistics_;
  snapshot.rolling_route = rolling_route_telemetry_.snapshot();
  return snapshot;
}

NavigationDiagnosticsFileRecordDecision
NavigationDiagnosticsSink::assessFileRecord(const std::int64_t now_ns,
                                            const bool diagnostics_error) {
  const std::scoped_lock lock{file_mutex_};
  const bool new_error_episode = diagnostics_error && !diagnostics_error_active_;
  if (!diagnostics_error) {
    diagnostics_error_active_ = false;
  }
  const bool file_due = last_file_stamp_ns_ <= 0 || now_ns < last_file_stamp_ns_ ||
                        now_ns - last_file_stamp_ns_ >= config_.file_period_ns;
  return NavigationDiagnosticsFileRecordDecision{
      .stamp_ns = now_ns,
      .required = diagnostics_stream_.good() && (file_due || new_error_episode),
      .new_error_episode = new_error_episode,
      .diagnostics_error = diagnostics_error,
  };
}

void NavigationDiagnosticsSink::appendFileRecord(
    const NavigationDiagnosticsFileRecordDecision& decision,
    const std::uint64_t trigger_tick, std::string json_line) {
  if (!decision.required) {
    return;
  }
  const std::scoped_lock lock{file_mutex_};
  diagnostics_stream_ << json_line;
  last_file_stamp_ns_ = decision.stamp_ns;
  diagnostics_error_ring_.push_back(std::move(json_line));
  while (diagnostics_error_ring_.size() > config_.error_ring_capacity) {
    diagnostics_error_ring_.pop_front();
  }
  if (decision.new_error_episode && diagnostics_error_stream_) {
    diagnostics_error_stream_
        << "{\"event\":\"diagnostics_error_context\",\"trigger_tick\":" << trigger_tick
        << ",\"records\":" << diagnostics_error_ring_.size() << "}\n";
    for (const std::string& record : diagnostics_error_ring_) {
      diagnostics_error_stream_ << record;
    }
    flushFilesLocked();
  }
  diagnostics_error_active_ = decision.diagnostics_error;
}

void NavigationDiagnosticsSink::flushFileIfDue() {
  const std::scoped_lock lock{file_mutex_};
  const auto now = std::chrono::steady_clock::now();
  if (diagnostics_stream_ && now - last_flush_time_ >= config_.flush_period) {
    diagnostics_stream_.flush();
    last_flush_time_ = now;
  }
}

std::uint64_t NavigationDiagnosticsSink::droppedSnapshots() const noexcept {
  return dropped_snapshots_.load(std::memory_order_relaxed);
}

std::uint64_t NavigationDiagnosticsSink::rejectedAfterStop() const noexcept {
  return rejected_after_stop_.load(std::memory_order_relaxed);
}

std::uint64_t NavigationDiagnosticsSink::processingFailures() const noexcept {
  return processing_failures_.load(std::memory_order_relaxed);
}

std::uint64_t NavigationDiagnosticsSink::failureHandlerFailures() const noexcept {
  return failure_handler_failures_.load(std::memory_order_relaxed);
}

bool NavigationDiagnosticsSink::accepting() const noexcept {
  return accepting_.load(std::memory_order_acquire);
}

void NavigationDiagnosticsSink::run(const std::stop_token stop_token) noexcept {
  while (!stop_token.stop_requested()) {
    std::optional<ProductionMppiDiagnosticsSnapshot> snapshot =
        mailbox_.waitPop(stop_token);
    if (!snapshot.has_value()) {
      break;
    }
    process(*snapshot);
  }
  if (std::optional<ProductionMppiDiagnosticsSnapshot> pending = mailbox_.tryPop();
      pending.has_value()) {
    process(*pending);
  }
}

void NavigationDiagnosticsSink::process(
    const ProductionMppiDiagnosticsSnapshot& snapshot) noexcept {
  try {
    processor_(snapshot);
  } catch (...) {
    processing_failures_.fetch_add(1U, std::memory_order_relaxed);
    if (failure_handler_) {
      try {
        failure_handler_(std::current_exception());
      } catch (...) {
        failure_handler_failures_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
  }
}

void NavigationDiagnosticsSink::flushFilesLocked() {
  if (diagnostics_stream_) {
    diagnostics_stream_.flush();
  }
  if (diagnostics_error_stream_) {
    diagnostics_error_stream_.flush();
  }
  last_flush_time_ = std::chrono::steady_clock::now();
}

} // namespace drone_city_nav

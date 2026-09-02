#pragma once

#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/rolling_route_telemetry_3d.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "production_mppi_diagnostics_snapshot.hpp"

namespace drone_city_nav {

struct NavigationDiagnosticsSinkConfig {
  std::filesystem::path output_directory;
  std::int64_t file_period_ns{200'000'000LL};
  std::chrono::duration<double> flush_period{1.0};
  std::size_t error_ring_capacity{25U};
  std::size_t configured_rollouts{0U};
  double deadline_ms{20.0};
};

struct NavigationDiagnosticsFileRecordDecision {
  std::int64_t stamp_ns{0};
  bool required{false};
  bool new_error_episode{false};
  bool diagnostics_error{false};
};

struct NavigationDiagnosticsStatistics {
  std::vector<double> runtime_samples_ms;
  std::vector<double> snapshot_phase_samples_ms;
  std::vector<double> controller_phase_samples_ms;
  std::vector<double> publication_phase_samples_ms;
  std::vector<double> tick_total_samples_ms;
  RollingRouteTelemetrySnapshot3D rolling_route{};
  std::uint64_t completed_ticks{0U};
  std::uint64_t deadline_misses{0U};
  std::uint64_t altitude_envelope_violation_horizons{0U};
  std::uint64_t post_update_contract_violations{0U};
  std::uint64_t no_progress_horizons{0U};
  std::uint64_t liveness_reseeds{0U};
  std::uint64_t mission_goal_position_hold_ticks{0U};
  std::uint64_t no_executable_route_hold_ticks{0U};
  std::uint64_t no_executable_horizon_hold_ticks{0U};
  std::uint64_t resident_owner_continuation_ticks{0U};
  std::uint64_t terminal_rest_horizon_ticks{0U};
  std::uint64_t finite_path_validation_backoff_ticks{0U};
  std::uint64_t latest_lidar_path_validation_backoff_ticks{0U};
  std::uint64_t retained_previous_finite_path_ticks{0U};
  std::uint64_t arrival_control_total{0U};
  std::uint64_t arrival_shaping_attempt_total{0U};
  std::uint64_t full_rollout_ticks{0U};
  std::uint64_t reduced_rollout_ticks{0U};
  std::uint64_t active_rollout_total{0U};
};

class NavigationDiagnosticsSink final {
public:
  using SnapshotProcessor =
      std::function<void(const ProductionMppiDiagnosticsSnapshot&)>;
  using ProcessingFailureHandler = std::function<void(const std::exception_ptr&)>;

  NavigationDiagnosticsSink(NavigationDiagnosticsSinkConfig config,
                            SnapshotProcessor processor,
                            ProcessingFailureHandler failure_handler = {});
  ~NavigationDiagnosticsSink();

  NavigationDiagnosticsSink(const NavigationDiagnosticsSink&) = delete;
  NavigationDiagnosticsSink& operator=(const NavigationDiagnosticsSink&) = delete;
  NavigationDiagnosticsSink(NavigationDiagnosticsSink&&) = delete;
  NavigationDiagnosticsSink& operator=(NavigationDiagnosticsSink&&) = delete;

  void start();
  void stop() noexcept;
  [[nodiscard]] bool enqueue(ProductionMppiDiagnosticsSnapshot snapshot);

  void recordTick(const mppi::MppiTickResult& result,
                  ProductionMppiPlanningState planning_state,
                  const ProductionMppiExecutionPublication& execution,
                  bool liveness_reseed_requested,
                  const RollingRouteTelemetryObservation3D& rolling_route,
                  const ProductionMppiTickPhaseTimings& phases = {});
  [[nodiscard]] NavigationDiagnosticsStatistics statistics() const;

  [[nodiscard]] NavigationDiagnosticsFileRecordDecision
  assessFileRecord(std::int64_t now_ns, bool diagnostics_error);
  void appendFileRecord(const NavigationDiagnosticsFileRecordDecision& decision,
                        std::uint64_t trigger_tick, std::string json_line);
  void flushFileIfDue();

  [[nodiscard]] std::uint64_t droppedSnapshots() const noexcept;
  [[nodiscard]] std::uint64_t rejectedAfterStop() const noexcept;
  [[nodiscard]] std::uint64_t processingFailures() const noexcept;
  [[nodiscard]] std::uint64_t failureHandlerFailures() const noexcept;
  [[nodiscard]] bool accepting() const noexcept;

private:
  void run(std::stop_token stop_token) noexcept;
  void process(const ProductionMppiDiagnosticsSnapshot& snapshot) noexcept;
  void flushFilesLocked();

  NavigationDiagnosticsSinkConfig config_;
  SnapshotProcessor processor_;
  ProcessingFailureHandler failure_handler_;

  mutable std::mutex lifecycle_mutex_;
  LatestValueMailbox<ProductionMppiDiagnosticsSnapshot> mailbox_;
  std::jthread worker_;
  std::atomic_bool accepting_{false};
  std::atomic<std::uint64_t> dropped_snapshots_{0U};
  std::atomic<std::uint64_t> rejected_after_stop_{0U};
  std::atomic<std::uint64_t> processing_failures_{0U};
  std::atomic<std::uint64_t> failure_handler_failures_{0U};

  mutable std::mutex statistics_mutex_;
  NavigationDiagnosticsStatistics statistics_;
  RollingRouteTelemetry3D rolling_route_telemetry_;

  mutable std::mutex file_mutex_;
  std::ofstream diagnostics_stream_;
  std::ofstream diagnostics_error_stream_;
  std::deque<std::string> diagnostics_error_ring_;
  std::chrono::steady_clock::time_point last_flush_time_{};
  std::int64_t last_file_stamp_ns_{0};
  bool diagnostics_error_active_{false};
};

} // namespace drone_city_nav

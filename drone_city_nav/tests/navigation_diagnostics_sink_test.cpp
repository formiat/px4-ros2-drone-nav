#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "navigation_diagnostics_sink.hpp"

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

class TemporaryDiagnosticsDirectory final {
public:
  explicit TemporaryDiagnosticsDirectory(const std::string& name)
      : path_{std::filesystem::path{::testing::TempDir()} / name} {
    std::filesystem::remove_all(path_);
  }

  ~TemporaryDiagnosticsDirectory() {
    std::filesystem::remove_all(path_);
  }

  TemporaryDiagnosticsDirectory(const TemporaryDiagnosticsDirectory&) = delete;
  TemporaryDiagnosticsDirectory&
  operator=(const TemporaryDiagnosticsDirectory&) = delete;
  TemporaryDiagnosticsDirectory(TemporaryDiagnosticsDirectory&&) = delete;
  TemporaryDiagnosticsDirectory& operator=(TemporaryDiagnosticsDirectory&&) = delete;

  [[nodiscard]] const std::filesystem::path& path() const noexcept {
    return path_;
  }

private:
  std::filesystem::path path_;
};

[[nodiscard]] NavigationDiagnosticsSinkConfig
testConfig(const std::filesystem::path& output_directory) {
  return NavigationDiagnosticsSinkConfig{
      .output_directory = output_directory,
      .file_period_ns = 1'000,
      .flush_period = std::chrono::duration<double>{0.001},
      .error_ring_capacity = 2U,
      .configured_rollouts = 8U,
      .deadline_ms = 20.0,
  };
}

[[nodiscard]] std::string readFile(const std::filesystem::path& path) {
  std::ifstream input{path};
  return std::string{std::istreambuf_iterator<char>{input},
                     std::istreambuf_iterator<char>{}};
}

TEST(NavigationDiagnosticsSinkTest, LatestValueOverloadIsBoundedAndObservable) {
  TemporaryDiagnosticsDirectory directory{"navigation_diagnostics_sink_overload"};
  std::mutex mutex;
  std::condition_variable condition;
  bool first_entered{false};
  bool release_first{false};
  std::vector<std::uint64_t> processed;
  NavigationDiagnosticsSink sink{
      testConfig(directory.path()),
      [&](const ProductionMppiDiagnosticsSnapshot& snapshot) {
        std::unique_lock lock{mutex};
        processed.push_back(snapshot.tick_sequence);
        if (snapshot.tick_sequence == 1U) {
          first_entered = true;
          condition.notify_all();
          condition.wait(lock, [&]() noexcept { return release_first; });
        }
        condition.notify_all();
      }};
  sink.start();

  ProductionMppiDiagnosticsSnapshot first;
  first.tick_sequence = 1U;
  ASSERT_TRUE(sink.enqueue(std::move(first)));
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() noexcept { return first_entered; }));
  }
  for (std::uint64_t tick = 2U; tick <= 4U; ++tick) {
    ProductionMppiDiagnosticsSnapshot snapshot;
    snapshot.tick_sequence = tick;
    EXPECT_TRUE(sink.enqueue(std::move(snapshot)));
  }
  {
    const std::scoped_lock lock{mutex};
    release_first = true;
  }
  condition.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s,
                                   [&]() noexcept { return processed.size() == 2U; }));
  }
  sink.stop();

  EXPECT_EQ(processed, (std::vector<std::uint64_t>{1U, 4U}));
  EXPECT_EQ(sink.droppedSnapshots(), 2U);
}

TEST(NavigationDiagnosticsSinkTest, StopDrainsTheLastAcceptedSnapshot) {
  TemporaryDiagnosticsDirectory directory{"navigation_diagnostics_sink_stop"};
  std::mutex mutex;
  std::condition_variable condition;
  bool first_entered{false};
  bool release_first{false};
  std::vector<std::uint64_t> processed;
  NavigationDiagnosticsSink sink{
      testConfig(directory.path()),
      [&](const ProductionMppiDiagnosticsSnapshot& snapshot) {
        std::unique_lock lock{mutex};
        processed.push_back(snapshot.tick_sequence);
        if (snapshot.tick_sequence == 1U) {
          first_entered = true;
          condition.notify_all();
          condition.wait(lock, [&]() noexcept { return release_first; });
        }
      }};
  sink.start();

  ProductionMppiDiagnosticsSnapshot first;
  first.tick_sequence = 1U;
  ASSERT_TRUE(sink.enqueue(std::move(first)));
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() noexcept { return first_entered; }));
  }
  ProductionMppiDiagnosticsSnapshot pending;
  pending.tick_sequence = 2U;
  ASSERT_TRUE(sink.enqueue(std::move(pending)));

  std::jthread stopper{[&sink]() { sink.stop(); }};
  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (sink.accepting() && std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::yield();
  }
  ASSERT_FALSE(sink.accepting());
  {
    const std::scoped_lock lock{mutex};
    release_first = true;
  }
  condition.notify_all();
  stopper.join();

  EXPECT_EQ(processed, (std::vector<std::uint64_t>{1U, 2U}));
  EXPECT_FALSE(sink.enqueue(ProductionMppiDiagnosticsSnapshot{}));
  EXPECT_EQ(sink.rejectedAfterStop(), 1U);
}

TEST(NavigationDiagnosticsSinkTest, OwnsRateLimitedFilesAndErrorContextRing) {
  TemporaryDiagnosticsDirectory directory{"navigation_diagnostics_sink_files"};
  NavigationDiagnosticsSink sink{testConfig(directory.path()),
                                 [](const ProductionMppiDiagnosticsSnapshot&) {}};

  const NavigationDiagnosticsFileRecordDecision first =
      sink.assessFileRecord(100, false);
  ASSERT_TRUE(first.required);
  sink.appendFileRecord(first, 1U, "{\"tick\":1}\n");
  EXPECT_FALSE(sink.assessFileRecord(200, false).required);

  const NavigationDiagnosticsFileRecordDecision first_error =
      sink.assessFileRecord(300, true);
  ASSERT_TRUE(first_error.required);
  ASSERT_TRUE(first_error.new_error_episode);
  sink.appendFileRecord(first_error, 2U, "{\"tick\":2}\n");
  EXPECT_FALSE(sink.assessFileRecord(400, true).required);
  EXPECT_FALSE(sink.assessFileRecord(500, false).required);

  const NavigationDiagnosticsFileRecordDecision second_error =
      sink.assessFileRecord(600, true);
  ASSERT_TRUE(second_error.required);
  ASSERT_TRUE(second_error.new_error_episode);
  sink.appendFileRecord(second_error, 3U, "{\"tick\":3}\n");
  sink.stop();

  const std::string ticks = readFile(directory.path() / "mppi_ticks.jsonl");
  EXPECT_EQ(ticks, "{\"tick\":1}\n{\"tick\":2}\n{\"tick\":3}\n");
  const std::string errors = readFile(directory.path() / "mppi_error_context.jsonl");
  EXPECT_NE(errors.find("\"trigger_tick\":2"), std::string::npos);
  EXPECT_NE(errors.find("\"trigger_tick\":3"), std::string::npos);
  const std::size_t second_event = errors.find("\"trigger_tick\":3");
  ASSERT_NE(second_event, std::string::npos);
  EXPECT_EQ(errors.find("{\"tick\":1}", second_event), std::string::npos);
  EXPECT_NE(errors.find("{\"tick\":2}", second_event), std::string::npos);
  EXPECT_NE(errors.find("{\"tick\":3}", second_event), std::string::npos);
}

TEST(NavigationDiagnosticsSinkTest, PublishesOneCoherentStatisticsSnapshot) {
  TemporaryDiagnosticsDirectory directory{"navigation_diagnostics_sink_statistics"};
  NavigationDiagnosticsSink sink{testConfig(directory.path()),
                                 [](const ProductionMppiDiagnosticsSnapshot&) {}};
  mppi::MppiTickResult result;
  result.timings.host_total_ms = 25.0;
  result.active_rollouts = 8U;
  result.altitude_envelope_violation = true;
  result.head_progress_m = 0.0F;
  ProductionMppiExecutionPublication execution;
  execution.reason = ProductionMppiExecutionReason::kNoExecutableHorizon;
  execution.terminal_rest_state = true;
  execution.finite_path_validation_backoff = true;
  execution.latest_lidar_path_validation_backoff = true;
  execution.retained_previous_finite_path = true;
  execution.arrival_control_count = 3U;
  execution.arrival_shaping_attempts = 2U;

  sink.recordTick(result, ProductionMppiPlanningState::kPlanned, execution, true,
                  RollingRouteTelemetryObservation3D{});
  const NavigationDiagnosticsStatistics statistics = sink.statistics();

  ASSERT_EQ(statistics.runtime_samples_ms.size(), 1U);
  EXPECT_DOUBLE_EQ(statistics.runtime_samples_ms.front(), 25.0);
  EXPECT_EQ(statistics.completed_ticks, 1U);
  EXPECT_EQ(statistics.deadline_misses, 1U);
  EXPECT_EQ(statistics.altitude_envelope_violation_horizons, 1U);
  EXPECT_EQ(statistics.post_update_contract_violations, 1U);
  EXPECT_EQ(statistics.no_progress_horizons, 1U);
  EXPECT_EQ(statistics.liveness_reseeds, 1U);
  EXPECT_EQ(statistics.no_executable_horizon_hold_ticks, 1U);
  EXPECT_EQ(statistics.terminal_rest_horizon_ticks, 1U);
  EXPECT_EQ(statistics.finite_path_validation_backoff_ticks, 1U);
  EXPECT_EQ(statistics.latest_lidar_path_validation_backoff_ticks, 1U);
  EXPECT_EQ(statistics.retained_previous_finite_path_ticks, 1U);
  EXPECT_EQ(statistics.arrival_control_total, 3U);
  EXPECT_EQ(statistics.arrival_shaping_attempt_total, 2U);
  EXPECT_EQ(statistics.full_rollout_ticks, 1U);
  EXPECT_EQ(statistics.reduced_rollout_ticks, 0U);
  EXPECT_EQ(statistics.active_rollout_total, 8U);
}

TEST(NavigationDiagnosticsSinkTest, ProcessingFailureCannotTerminateTheWorker) {
  TemporaryDiagnosticsDirectory directory{"navigation_diagnostics_sink_failure"};
  std::mutex mutex;
  std::condition_variable condition;
  std::uint64_t processed{0U};
  NavigationDiagnosticsSink sink{
      testConfig(directory.path()),
      [&](const ProductionMppiDiagnosticsSnapshot&) {
        const std::scoped_lock lock{mutex};
        ++processed;
        condition.notify_all();
        throw std::runtime_error{"expected diagnostics failure"};
      },
      [](const std::exception_ptr&) {
        throw std::runtime_error{"expected failure-handler failure"};
      }};
  sink.start();
  ASSERT_TRUE(sink.enqueue(ProductionMppiDiagnosticsSnapshot{}));
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return processed == 1U; }));
  }
  sink.stop();
  EXPECT_EQ(sink.processingFailures(), 1U);
  EXPECT_EQ(sink.failureHandlerFailures(), 1U);
}

} // namespace
} // namespace drone_city_nav

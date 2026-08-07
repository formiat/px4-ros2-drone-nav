#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace drone_city_nav::mppi {

struct TimingStatistics {
  double mean_ms{0.0};
  double p50_ms{0.0};
  double p90_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double maximum_ms{0.0};
};

struct StageTimingStatistics {
  TimingStatistics noise_generation{};
  TimingStatistics rollout_simulation{};
  TimingStatistics risk_reduction{};
  TimingStatistics weight_calculation{};
  TimingStatistics control_update{};
  TimingStatistics warm_start{};
  TimingStatistics gpu_total{};
  TimingStatistics post_update_evaluation{};
  TimingStatistics horizon_reconstruction{};
  TimingStatistics host_total{};
};

struct BenchmarkResult {
  std::string gpu_name;
  int compute_major{0};
  int compute_minor{0};
  std::size_t allocated_bytes{0U};
  double esdf_build_ms{0.0};
  double esdf_upload_ms{0.0};
  double allocation_ms{0.0};
  StageTimingStatistics timings{};
  std::size_t deadline_misses{0U};
  double deadline_miss_ratio{0.0};
  RolloutMetrics selected{};
  bool reference_check_passed{false};
  bool deterministic_replay_passed{false};
};

[[nodiscard]] BenchmarkResult runCudaBenchmark(const BenchmarkConfig& config);
[[nodiscard]] BenchmarkResult runPersistentCudaBenchmark(const BenchmarkConfig& config);
[[nodiscard]] std::vector<std::string> benchmarkScenarioNames();
[[nodiscard]] std::string benchmarkResultJson(const BenchmarkConfig& config,
                                              const BenchmarkResult& result);

} // namespace drone_city_nav::mppi

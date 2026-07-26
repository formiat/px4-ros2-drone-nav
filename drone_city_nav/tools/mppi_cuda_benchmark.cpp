#include "drone_city_nav/mppi/mppi_cuda.hpp"

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

using drone_city_nav::mppi::BenchmarkConfig;
using drone_city_nav::mppi::BenchmarkResult;

[[nodiscard]] std::string_view requireValue(const int argc, char** argv, int& index) {
  if (index + 1 >= argc) {
    throw std::invalid_argument{std::string{"missing value after "} + argv[index]};
  }
  ++index;
  return argv[index];
}

template<typename T>
[[nodiscard]] T parseNumber(const std::string_view value, const char* const name) {
  T parsed{};
  const auto result =
      std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::invalid_argument{std::string{"invalid "} + name + ": " +
                                std::string{value}};
  }
  return parsed;
}

[[nodiscard]] double parseDouble(const std::string_view value, const char* const name) {
  std::size_t consumed = 0U;
  const double parsed = std::stod(std::string{value}, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument{std::string{"invalid "} + name + ": " +
                                std::string{value}};
  }
  return parsed;
}

void printUsage() {
  std::cout << "Usage: mppi_cuda_benchmark [options]\n"
            << "  --scenario NAME\n"
            << "  --rollouts COUNT\n"
            << "  --steps COUNT\n"
            << "  --dt SECONDS\n"
            << "  --ticks COUNT\n"
            << "  --warmup-ticks COUNT\n"
            << "  --deadline-ms MILLISECONDS\n"
            << "  --seed VALUE\n"
            << "  --no-early-exit\n"
            << "  --json PATH\n"
            << "  --matrix default\n"
            << "  --list-scenarios\n";
}

void printSummary(const BenchmarkConfig& config, const BenchmarkResult& result) {
  const auto& simulation = result.timings.rollout_simulation;
  const auto& gpu = result.timings.gpu_total;
  const auto& host = result.timings.host_total;
  std::cout << "GPU: " << result.gpu_name << '\n'
            << "compute capability: " << result.compute_major << '.'
            << result.compute_minor << '\n'
            << "scenario: " << config.scenario << '\n'
            << "rollouts: " << config.rollouts << '\n'
            << "steps: " << config.steps << '\n'
            << "ticks: " << config.measured_ticks << '\n'
            << "allocated MiB: "
            << static_cast<double>(result.allocated_bytes) / (1024.0 * 1024.0) << '\n'
            << "esdf build ms: " << result.esdf_build_ms << '\n'
            << "esdf upload ms: " << result.esdf_upload_ms << '\n'
            << "simulation ms: p50=" << simulation.p50_ms
            << " p95=" << simulation.p95_ms << " max=" << simulation.maximum_ms << '\n'
            << "gpu total ms: p50=" << gpu.p50_ms << " p95=" << gpu.p95_ms
            << " p99=" << gpu.p99_ms << " max=" << gpu.maximum_ms << '\n'
            << "host total ms: p50=" << host.p50_ms << " p95=" << host.p95_ms
            << " p99=" << host.p99_ms << " max=" << host.maximum_ms << '\n'
            << "deadline: target=" << config.deadline_ms
            << " missed=" << result.deadline_misses
            << " ratio=" << result.deadline_miss_ratio << '\n'
            << "selected: collision=" << (result.selected.collision ? "true" : "false")
            << " tier=" << static_cast<int>(result.selected.worst_tier)
            << " critical_m=" << result.selected.critical_exposure_m
            << " planning_m=" << result.selected.planning_exposure_m
            << " min_clearance_m=" << result.selected.minimum_clearance_m
            << " terminal=(" << result.selected.terminal_state.x << ','
            << result.selected.terminal_state.y << ")\n"
            << "checks: reference="
            << (result.reference_check_passed ? "passed" : "failed")
            << " deterministic="
            << (result.deterministic_replay_passed ? "passed" : "failed") << '\n';
}

void writeJson(const std::filesystem::path& path, const BenchmarkConfig& config,
               const BenchmarkResult& result) {
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream{path};
  if (!stream) {
    throw std::runtime_error{"failed to open benchmark JSON output: " + path.string()};
  }
  stream << drone_city_nav::mppi::benchmarkResultJson(config, result);
}

[[nodiscard]] BenchmarkConfig parseArguments(const int argc, char** argv,
                                             std::filesystem::path& json_path,
                                             bool& matrix_requested) {
  BenchmarkConfig config;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument{argv[index]};
    if (argument == "--scenario") {
      config.scenario = requireValue(argc, argv, index);
    } else if (argument == "--rollouts") {
      config.rollouts =
          parseNumber<std::size_t>(requireValue(argc, argv, index), "rollouts");
    } else if (argument == "--steps") {
      config.steps = parseNumber<std::size_t>(requireValue(argc, argv, index), "steps");
    } else if (argument == "--dt") {
      config.dynamics.dt_s =
          static_cast<float>(parseDouble(requireValue(argc, argv, index), "dt"));
    } else if (argument == "--ticks") {
      config.measured_ticks =
          parseNumber<std::size_t>(requireValue(argc, argv, index), "ticks");
    } else if (argument == "--warmup-ticks") {
      config.warmup_ticks =
          parseNumber<std::size_t>(requireValue(argc, argv, index), "warmup ticks");
    } else if (argument == "--deadline-ms") {
      config.deadline_ms = parseDouble(requireValue(argc, argv, index), "deadline");
    } else if (argument == "--seed") {
      config.seed = parseNumber<std::uint64_t>(requireValue(argc, argv, index), "seed");
    } else if (argument == "--no-early-exit") {
      config.early_exit_on_collision = false;
    } else if (argument == "--json") {
      json_path = requireValue(argc, argv, index);
    } else if (argument == "--matrix") {
      const std::string_view matrix = requireValue(argc, argv, index);
      if (matrix != "default") {
        throw std::invalid_argument{"only --matrix default is supported"};
      }
      matrix_requested = true;
    } else if (argument == "--list-scenarios") {
      for (const std::string& scenario :
           drone_city_nav::mppi::benchmarkScenarioNames()) {
        std::cout << scenario << '\n';
      }
      std::exit(0);
    } else if (argument == "--help" || argument == "-h") {
      printUsage();
      std::exit(0);
    } else {
      throw std::invalid_argument{"unknown argument: " + std::string{argument}};
    }
  }
  return config;
}

int runMatrix(BenchmarkConfig base, const std::filesystem::path& output_path) {
  const std::vector<std::size_t> rollout_counts{2048U, 4096U, 8192U, 16384U};
  const std::vector<std::size_t> step_counts{60U, 80U, 100U};
  const std::vector<std::string> scenarios{"open_space", "urban_blocks",
                                           "narrow_corridor"};
  std::filesystem::path directory = output_path;
  if (directory.empty()) {
    directory = "artifacts/mppi_benchmark";
  }
  for (const std::string& scenario : scenarios) {
    for (const std::size_t rollouts : rollout_counts) {
      for (const std::size_t steps : step_counts) {
        BenchmarkConfig config = base;
        config.scenario = scenario;
        config.rollouts = rollouts;
        config.steps = steps;
        std::cout << "\n=== " << scenario << ' ' << rollouts << "x" << steps
                  << " ===\n";
        const BenchmarkResult result =
            drone_city_nav::mppi::runPersistentCudaBenchmark(config);
        printSummary(config, result);
        writeJson(directory / (scenario + "_" + std::to_string(rollouts) + "x" +
                               std::to_string(steps) + ".json"),
                  config, result);
      }
    }
  }
  return 0;
}

} // namespace

int main(const int argc, char** argv) {
  try {
    std::filesystem::path json_path;
    bool matrix_requested = false;
    BenchmarkConfig config = parseArguments(argc, argv, json_path, matrix_requested);
    if (matrix_requested) {
      return runMatrix(config, json_path);
    }
    const BenchmarkResult result =
        drone_city_nav::mppi::runPersistentCudaBenchmark(config);
    printSummary(config, result);
    if (!json_path.empty()) {
      writeJson(json_path, config, result);
    }
    return result.reference_check_passed && result.deterministic_replay_passed ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "mppi_cuda_benchmark: " << error.what() << '\n';
    return 1;
  }
}

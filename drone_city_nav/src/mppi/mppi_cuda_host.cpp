#include "drone_city_nav/mppi/mppi_cuda.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] std::string escapeJson(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

void appendStatisticsJson(std::ostringstream& stream, const char* const name,
                          const TimingStatistics& stats, const bool trailing) {
  stream << "    \"" << name << "\": {\"mean_ms\": " << stats.mean_ms
         << ", \"p50_ms\": " << stats.p50_ms << ", \"p90_ms\": " << stats.p90_ms
         << ", \"p95_ms\": " << stats.p95_ms << ", \"p99_ms\": " << stats.p99_ms
         << ", \"max_ms\": " << stats.maximum_ms << "}" << (trailing ? "," : "")
         << '\n';
}

} // namespace

std::vector<std::string> benchmarkScenarioNames() {
  return {"open_space",      "single_wall",         "parallel_walls",
          "narrow_corridor", "building_block",      "u_shaped_obstacle",
          "urban_blocks",    "passage_lower_upper", "random_occupancy"};
}

std::string benchmarkResultJson(const BenchmarkConfig& config,
                                const BenchmarkResult& result) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "{\n"
         << "  \"gpu\": \"" << escapeJson(result.gpu_name) << "\",\n"
         << "  \"compute_capability\": \"" << result.compute_major << '.'
         << result.compute_minor << "\",\n"
         << "  \"scenario\": \"" << escapeJson(config.scenario) << "\",\n"
         << "  \"rollouts\": " << config.rollouts << ",\n"
         << "  \"steps\": " << config.steps << ",\n"
         << "  \"dt_s\": " << config.dynamics.dt_s << ",\n"
         << "  \"warmup_ticks\": " << config.warmup_ticks << ",\n"
         << "  \"measured_ticks\": " << config.measured_ticks << ",\n"
         << "  \"deadline_ms\": " << config.deadline_ms << ",\n"
         << "  \"allocated_bytes\": " << result.allocated_bytes << ",\n"
         << "  \"esdf_build_ms\": " << result.esdf_build_ms << ",\n"
         << "  \"esdf_upload_ms\": " << result.esdf_upload_ms << ",\n"
         << "  \"allocation_ms\": " << result.allocation_ms << ",\n"
         << "  \"timings\": {\n";
  appendStatisticsJson(stream, "noise_generation", result.timings.noise_generation,
                       true);
  appendStatisticsJson(stream, "rollout_simulation", result.timings.rollout_simulation,
                       true);
  appendStatisticsJson(stream, "risk_reduction", result.timings.risk_reduction, true);
  appendStatisticsJson(stream, "weight_calculation", result.timings.weight_calculation,
                       true);
  appendStatisticsJson(stream, "control_update", result.timings.control_update, true);
  appendStatisticsJson(stream, "warm_start", result.timings.warm_start, true);
  appendStatisticsJson(stream, "gpu_total", result.timings.gpu_total, true);
  appendStatisticsJson(stream, "host_total", result.timings.host_total, false);
  stream << "  },\n"
         << "  \"deadline_misses\": " << result.deadline_misses << ",\n"
         << "  \"deadline_miss_ratio\": " << result.deadline_miss_ratio << ",\n"
         << "  \"reference_check_passed\": "
         << (result.reference_check_passed ? "true" : "false") << ",\n"
         << "  \"deterministic_replay_passed\": "
         << (result.deterministic_replay_passed ? "true" : "false") << ",\n"
         << "  \"selected\": {\n"
         << "    \"collision\": " << (result.selected.collision ? "true" : "false")
         << ",\n"
         << "    \"worst_tier\": " << static_cast<int>(result.selected.worst_tier)
         << ",\n"
         << "    \"critical_exposure_m\": " << result.selected.critical_exposure_m
         << ",\n"
         << "    \"planning_exposure_m\": " << result.selected.planning_exposure_m
         << ",\n"
         << "    \"minimum_clearance_m\": " << result.selected.minimum_clearance_m
         << ",\n"
         << "    \"soft_cost\": " << result.selected.soft_cost << ",\n"
         << "    \"terminal_x\": " << result.selected.terminal_state.x << ",\n"
         << "    \"terminal_y\": " << result.selected.terminal_state.y << "\n"
         << "  }\n"
         << "}\n";
  return stream.str();
}

} // namespace drone_city_nav::mppi

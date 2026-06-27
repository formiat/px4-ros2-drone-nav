#include "drone_city_nav/final_trajectory_debug_io.hpp"

#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>

namespace drone_city_nav {

[[nodiscard]] std::vector<double> finalTrajectoryProfiledTimesFromStart(
    const std::span<const TrajectoryPointSample> samples,
    const TrajectorySpeedProfile& speed_profile) {
  std::vector<double> times(samples.size(), std::numeric_limits<double>::quiet_NaN());
  if (samples.empty()) {
    return times;
  }

  constexpr double kMinimumIntegrationSpeedMps = 0.1;
  times.front() = 0.0;
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    double ds = samples[i].s_m - samples[i - 1U].s_m;
    if (!std::isfinite(ds) || !(ds > 0.0)) {
      ds = distance(samples[i - 1U].point, samples[i].point);
    }
    if (!(ds > 1.0e-6) || !std::isfinite(ds) || !std::isfinite(times[i - 1U])) {
      continue;
    }
    const TrajectorySpeedSample start =
        speedProfileSampleAtS(speed_profile, samples[i - 1U].s_m);
    const TrajectorySpeedSample end =
        speedProfileSampleAtS(speed_profile, samples[i].s_m);
    const double average_speed =
        std::max(kMinimumIntegrationSpeedMps,
                 0.5 * (start.profiled_limit_mps + end.profiled_limit_mps));
    if (std::isfinite(average_speed)) {
      times[i] = times[i - 1U] + ds / average_speed;
    }
  }
  return times;
}

bool writeFinalTrajectorySamplesCsv(std::ostream& stream,
                                    const FinalTrajectorySamplesCsvInput& input) {
  if (!stream.good()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << input.source_label
         << " local_path_update_id=" << input.local_path_update_id
         << " planner_path_id=" << input.planner_path_id
         << " trajectory_valid=" << (input.trajectory_valid ? "true" : "false")
         << " trajectory_status="
         << trajectoryPlannerStatusName(input.trajectory_status) << "\n";
  stream << finalTrajectorySamplesCsvHeader() << "\n";
  const TrajectorySpeedProfile empty_profile;
  const TrajectorySpeedProfile& speed_profile =
      input.speed_profile != nullptr ? *input.speed_profile : empty_profile;
  const std::vector<double> times_from_start =
      finalTrajectoryProfiledTimesFromStart(input.samples, speed_profile);
  const double total_time_s = times_from_start.empty()
                                  ? std::numeric_limits<double>::quiet_NaN()
                                  : times_from_start.back();
  for (std::size_t i = 0U; i < input.samples.size(); ++i) {
    const TrajectoryPointSample& sample = input.samples[i];
    const TrajectorySpeedSample speed_sample =
        speed_profile.valid ? speedProfileSampleAtS(speed_profile, sample.s_m)
                            : TrajectorySpeedSample{};
    const double time_from_start_s = i < times_from_start.size()
                                         ? times_from_start[i]
                                         : std::numeric_limits<double>::quiet_NaN();
    const double time_to_finish_s =
        std::isfinite(total_time_s) && std::isfinite(time_from_start_s)
            ? std::max(0.0, total_time_s - time_from_start_s)
            : std::numeric_limits<double>::quiet_NaN();
    stream << finalTrajectorySamplesCsvRow(i, sample, speed_sample, time_from_start_s,
                                           time_to_finish_s)
           << "\n";
  }
  return stream.good();
}

bool writeFinalTrajectorySamplesCsvFile(const std::filesystem::path& path,
                                        const FinalTrajectorySamplesCsvInput& input) {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  return writeFinalTrajectorySamplesCsv(stream, input);
}

bool writeFinalTrajectorySummaryJson(
    std::ostream& stream, const TrajectoryPlannerStats& stats,
    const TrajectoryShapeDiagnostics& shape_diagnostics) {
  if (!stream.good()) {
    return false;
  }
  stream << finalTrajectoryDiagnosticsSummaryJson(stats, shape_diagnostics) << "\n";
  return stream.good();
}

bool writeFinalTrajectorySummaryJsonFile(
    const std::filesystem::path& path, const TrajectoryPlannerStats& stats,
    const TrajectoryShapeDiagnostics& shape_diagnostics) {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  return writeFinalTrajectorySummaryJson(stream, stats, shape_diagnostics);
}

} // namespace drone_city_nav

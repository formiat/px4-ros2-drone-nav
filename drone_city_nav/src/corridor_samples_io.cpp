#include "drone_city_nav/corridor_samples_io.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>

namespace drone_city_nav {

void writeCsvNumberOrEmpty(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
  }
}

[[nodiscard]] std::optional<std::size_t>
windowIdForS(const std::span<const RacingLineWindowMetadata> windows,
             const double s_m) {
  if (!std::isfinite(s_m)) {
    return std::nullopt;
  }
  constexpr double kWindowEpsilonM = 1.0e-6;
  for (const RacingLineWindowMetadata& window : windows) {
    if (s_m + kWindowEpsilonM >= window.begin_s_m &&
        s_m <= window.end_s_m + kWindowEpsilonM) {
      return window.id;
    }
  }
  return std::nullopt;
}

[[nodiscard]] double
selectedOffsetAtS(const std::span<const TrajectoryPointSample> samples,
                  const double s_m) {
  if (samples.empty() || !std::isfinite(s_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (samples.size() == 1U || s_m <= samples.front().s_m) {
    return samples.front().racing_offset_m;
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (s_m > samples[i].s_m) {
      continue;
    }
    const TrajectoryPointSample& previous = samples[i - 1U];
    const TrajectoryPointSample& next = samples[i];
    if (!std::isfinite(previous.racing_offset_m) ||
        !std::isfinite(next.racing_offset_m)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double span_m = next.s_m - previous.s_m;
    if (!(span_m > 1.0e-6)) {
      return next.racing_offset_m;
    }
    const double t = std::clamp((s_m - previous.s_m) / span_m, 0.0, 1.0);
    return previous.racing_offset_m +
           (next.racing_offset_m - previous.racing_offset_m) * t;
  }
  return samples.back().racing_offset_m;
}

bool writeCorridorSamplesCsv(std::ostream& stream,
                             const TrajectoryPlannerResult& result,
                             const std::string_view source_label,
                             const std::uint64_t candidate_path_id) {
  if (!stream.good()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "sample_index,s_m,route_x,route_y,center_x,center_y,tangent_x,"
            "tangent_y,normal_x,normal_y,left_bound_m,right_bound_m,width_m,"
            "clearance_m,center_recovery_m,window_id,active_window,"
            "selected_offset_m,distance_to_prohibited_m\n";
  for (std::size_t i = 0U; i < result.corridor_samples.size(); ++i) {
    const CorridorSample& sample = result.corridor_samples[i];
    const std::optional<std::size_t> window_id =
        windowIdForS(result.racing_windows, sample.s_m);
    const bool active_window = window_id.has_value();
    const double selected_offset_m = selectedOffsetAtS(result.samples, sample.s_m);
    stream << i << ",";
    writeCsvNumberOrEmpty(stream, sample.s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.route_center.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.route_center.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.tangent.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.tangent.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.normal.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.normal.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.left_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.right_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.left_bound_m + sample.right_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.clearance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center_recovery_m);
    stream << ",";
    if (window_id.has_value()) {
      stream << *window_id;
    }
    stream << "," << (active_window ? "true" : "false") << ",";
    writeCsvNumberOrEmpty(stream, selected_offset_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.clearance_m);
    stream << "\n";
  }
  return stream.good();
}

bool writeCorridorSamplesCsvFile(const std::filesystem::path& path,
                                 const TrajectoryPlannerResult& result,
                                 const std::string_view source_label,
                                 const std::uint64_t candidate_path_id) {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  return writeCorridorSamplesCsv(stream, result, source_label, candidate_path_id);
}

} // namespace drone_city_nav

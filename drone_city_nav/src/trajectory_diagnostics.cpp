#include "drone_city_nav/trajectory_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

} // namespace

TrajectoryShapeDiagnostics computeTrajectoryShapeDiagnostics(
    const std::span<const TrajectoryPointSample> samples) {
  TrajectoryShapeDiagnostics diagnostics{};
  if (samples.size() < 2U) {
    return diagnostics;
  }

  diagnostics.segment_count = samples.size() - 1U;
  diagnostics.min_segment_length_m = std::numeric_limits<double>::infinity();
  double segment_length_sum = 0.0;
  double previous_heading_rad = std::numeric_limits<double>::quiet_NaN();
  bool previous_heading_valid = false;

  for (std::size_t i = 1U; i < samples.size(); ++i) {
    const Point2 delta{samples[i].point.x - samples[i - 1U].point.x,
                       samples[i].point.y - samples[i - 1U].point.y};
    const double segment_length_m = std::hypot(delta.x, delta.y);
    diagnostics.min_segment_length_m =
        std::min(diagnostics.min_segment_length_m, segment_length_m);
    diagnostics.max_segment_length_m =
        std::max(diagnostics.max_segment_length_m, segment_length_m);
    segment_length_sum += segment_length_m;
    if (segment_length_m < 0.5) {
      ++diagnostics.segments_shorter_than_0_5m;
    }
    if (segment_length_m < 1.0) {
      ++diagnostics.segments_shorter_than_1m;
    }
    if (segment_length_m < 2.0) {
      ++diagnostics.segments_shorter_than_2m;
    }

    if (segment_length_m > kTinyDistanceM) {
      const double heading_rad = std::atan2(delta.y, delta.x);
      if (previous_heading_valid) {
        const double heading_delta_rad =
            std::abs(normalizeAngle(heading_rad - previous_heading_rad));
        if (heading_delta_rad > diagnostics.max_heading_delta_rad) {
          diagnostics.max_heading_delta_rad = heading_delta_rad;
          diagnostics.max_heading_delta_index = i;
          diagnostics.max_heading_delta_point = samples[i].point;
        }
      }
      previous_heading_rad = heading_rad;
      previous_heading_valid = true;
    }

    const double curvature_jump =
        std::abs(samples[i].curvature_1pm - samples[i - 1U].curvature_1pm);
    if (curvature_jump > diagnostics.max_curvature_jump_1pm) {
      diagnostics.max_curvature_jump_1pm = curvature_jump;
      diagnostics.max_curvature_jump_index = i;
      diagnostics.max_curvature_jump_point = samples[i].point;
    }

    if (std::isfinite(samples[i].lateral_offset_m) &&
        std::isfinite(samples[i - 1U].lateral_offset_m)) {
      const double offset_delta =
          std::abs(samples[i].lateral_offset_m - samples[i - 1U].lateral_offset_m);
      if (offset_delta > diagnostics.max_offset_delta_m) {
        diagnostics.max_offset_delta_m = offset_delta;
        diagnostics.max_offset_delta_index = i;
        diagnostics.max_offset_delta_point = samples[i].point;
      }
    }
  }

  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    if (!std::isfinite(samples[i - 1U].lateral_offset_m) ||
        !std::isfinite(samples[i].lateral_offset_m) ||
        !std::isfinite(samples[i + 1U].lateral_offset_m)) {
      continue;
    }
    const double offset_second_delta =
        std::abs(samples[i + 1U].lateral_offset_m - 2.0 * samples[i].lateral_offset_m +
                 samples[i - 1U].lateral_offset_m);
    if (offset_second_delta > diagnostics.max_offset_second_delta_m) {
      diagnostics.max_offset_second_delta_m = offset_second_delta;
      diagnostics.max_offset_second_delta_index = i;
      diagnostics.max_offset_second_delta_point = samples[i].point;
    }
  }

  diagnostics.mean_segment_length_m =
      segment_length_sum / static_cast<double>(diagnostics.segment_count);
  if (!std::isfinite(diagnostics.min_segment_length_m)) {
    diagnostics.min_segment_length_m = std::numeric_limits<double>::quiet_NaN();
  }
  return diagnostics;
}

} // namespace drone_city_nav

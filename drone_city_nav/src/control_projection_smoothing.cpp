#include "drone_city_nav/control_projection_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kTwoPi = 2.0 * std::numbers::pi;
constexpr double kCurvatureSignChangeThreshold1pm = 0.002;

struct ControlProjectionSmoothingWindow {
  Point2 tangent{};
  double mean_curvature_1pm{0.0};
  double min_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double max_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double heading_span_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double window_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double window_end_s_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double normalizedAngle(double angle_rad) noexcept {
  angle_rad = std::fmod(angle_rad, kTwoPi);
  if (angle_rad < 0.0) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

[[nodiscard]] double headingSpanRad(std::vector<double> angles_rad) {
  if (angles_rad.size() < 2U) {
    return 0.0;
  }
  for (double& angle_rad : angles_rad) {
    angle_rad = normalizedAngle(angle_rad);
  }
  std::ranges::sort(angles_rad);

  double max_gap = 0.0;
  for (std::size_t i = 0U; i + 1U < angles_rad.size(); ++i) {
    max_gap = std::max(max_gap, angles_rad[i + 1U] - angles_rad[i]);
  }
  max_gap = std::max(max_gap, angles_rad.front() + kTwoPi - angles_rad.back());
  return kTwoPi - max_gap;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double smoothstep(const double edge0, const double edge1,
                                const double value) noexcept {
  if (!(edge1 > edge0)) {
    return value >= edge1 ? 1.0 : 0.0;
  }
  const double t = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

[[nodiscard]] bool
curvatureSignChanges(const ControlProjectionSmoothingWindow& window) noexcept {
  return std::isfinite(window.min_curvature_1pm) &&
         std::isfinite(window.max_curvature_1pm) &&
         window.min_curvature_1pm < -kCurvatureSignChangeThreshold1pm &&
         window.max_curvature_1pm > kCurvatureSignChangeThreshold1pm;
}

[[nodiscard]] double
smoothedStraightishCurvature1pm(const ControlProjectionSmoothingWindow& window,
                                const double straight_max_heading_span_rad,
                                const double straight_max_abs_curvature_1pm) noexcept {
  const bool nearly_straight_heading =
      window.heading_span_rad <= 0.5 * straight_max_heading_span_rad;
  const bool nearly_zero_curvature =
      window.max_abs_curvature_1pm <= 0.5 * straight_max_abs_curvature_1pm;
  if (curvatureSignChanges(window) || nearly_straight_heading ||
      nearly_zero_curvature) {
    return 0.0;
  }
  return window.mean_curvature_1pm;
}

[[nodiscard]] double
curvatureFeedforwardContextScale(const ControlProjectionSmoothingWindow& window,
                                 const VelocityFollowerConfig& config) noexcept {
  if (!std::isfinite(window.heading_span_rad) ||
      !std::isfinite(window.max_abs_curvature_1pm) ||
      !(window.max_abs_curvature_1pm > kTinyDistanceM)) {
    return 1.0;
  }

  double scale = 1.0;
  const double straight_heading_span_rad =
      sanitizedPositive(config.control_tangent_smoothing_max_heading_span_rad,
                        12.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double straight_max_abs_curvature = sanitizedPositive(
      config.control_tangent_smoothing_max_abs_curvature_1pm, 0.015, 0.0, 1000.0);
  if (straight_heading_span_rad > kTinyDistanceM &&
      window.max_abs_curvature_1pm <= straight_max_abs_curvature &&
      window.heading_span_rad < straight_heading_span_rad) {
    scale *= smoothstep(0.5 * straight_heading_span_rad, straight_heading_span_rad,
                        window.heading_span_rad);
  }

  if (curvatureSignChanges(window)) {
    const double mean_to_peak_ratio =
        std::abs(window.mean_curvature_1pm) / window.max_abs_curvature_1pm;
    scale *= smoothstep(0.35, 0.85, mean_to_peak_ratio);
  }

  return std::clamp(scale, 0.0, 1.0);
}

[[nodiscard]] std::optional<TrajectoryPointSample>
sampleAtS(const std::span<const TrajectoryPointSample> samples, const double s_m) {
  if (!trajectorySamplesAreUsable(samples)) {
    return std::nullopt;
  }
  const double clamped_s = std::clamp(std::isfinite(s_m) ? s_m : 0.0,
                                      samples.front().s_m, samples.back().s_m);
  if (clamped_s <= samples.front().s_m) {
    return samples.front();
  }
  if (clamped_s >= samples.back().s_m) {
    return samples.back();
  }

  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (clamped_s < start.s_m || clamped_s > end.s_m) {
      continue;
    }
    const double station_delta_m = end.s_m - start.s_m;
    if (!(station_delta_m > kTinyDistanceM)) {
      continue;
    }
    const double t = std::clamp((clamped_s - start.s_m) / station_delta_m, 0.0, 1.0);
    TrajectoryPointSample sample{};
    sample.s_m = clamped_s;
    sample.point = start.point * (1.0 - t) + end.point * t;
    sample.tangent = normalized(start.tangent * (1.0 - t) + end.tangent * t);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      sample.tangent = normalized(end.point - start.point);
    }
    sample.curvature_1pm = start.curvature_1pm * (1.0 - t) + end.curvature_1pm * t;
    return sample;
  }

  return std::nullopt;
}

[[nodiscard]] std::optional<ControlProjectionSmoothingWindow>
buildControlProjectionSmoothingWindow(
    const std::span<const TrajectoryPointSample> samples, const double station_s_m,
    const double back_m, const double forward_m) {
  if (!(back_m + forward_m > kTinyDistanceM) || !trajectorySamplesAreUsable(samples)) {
    return std::nullopt;
  }

  ControlProjectionSmoothingWindow window{};
  window.window_start_s_m = std::max(samples.front().s_m, station_s_m - back_m);
  window.window_end_s_m = std::min(samples.back().s_m, station_s_m + forward_m);
  if (!(window.window_end_s_m - window.window_start_s_m > kTinyDistanceM)) {
    return std::nullopt;
  }

  const std::optional<TrajectoryPointSample> start_sample =
      sampleAtS(samples, window.window_start_s_m);
  const std::optional<TrajectoryPointSample> end_sample =
      sampleAtS(samples, window.window_end_s_m);
  if (!start_sample.has_value() || !end_sample.has_value()) {
    return std::nullopt;
  }

  window.tangent = normalized(end_sample->point - start_sample->point);
  if (!(norm(window.tangent) > kTinyDistanceM)) {
    return std::nullopt;
  }

  std::vector<double> headings_rad;
  headings_rad.reserve(samples.size() + 2U);
  double curvature_sum = 0.0;
  std::size_t curvature_count = 0U;
  auto add_sample = [&](const TrajectoryPointSample& sample) {
    if (norm(sample.tangent) > kTinyDistanceM && std::isfinite(sample.curvature_1pm)) {
      headings_rad.push_back(std::atan2(sample.tangent.y, sample.tangent.x));
      window.max_abs_curvature_1pm = std::max(
          std::isfinite(window.max_abs_curvature_1pm) ? window.max_abs_curvature_1pm
                                                      : 0.0,
          std::abs(sample.curvature_1pm));
      window.min_curvature_1pm =
          std::isfinite(window.min_curvature_1pm)
              ? std::min(window.min_curvature_1pm, sample.curvature_1pm)
              : sample.curvature_1pm;
      window.max_curvature_1pm =
          std::isfinite(window.max_curvature_1pm)
              ? std::max(window.max_curvature_1pm, sample.curvature_1pm)
              : sample.curvature_1pm;
      curvature_sum += sample.curvature_1pm;
      ++curvature_count;
    }
  };

  add_sample(*start_sample);
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m > window.window_start_s_m && sample.s_m < window.window_end_s_m) {
      add_sample(sample);
    }
  }
  add_sample(*end_sample);
  window.heading_span_rad = headingSpanRad(std::move(headings_rad));
  if (curvature_count > 0U) {
    window.mean_curvature_1pm = curvature_sum / static_cast<double>(curvature_count);
  }
  return window;
}

} // namespace

const char*
controlProjectionSmoothingModeName(const ControlProjectionSmoothingMode mode) noexcept {
  switch (mode) {
    case ControlProjectionSmoothingMode::kNone:
      return "none";
    case ControlProjectionSmoothingMode::kStraight:
      return "straight";
    case ControlProjectionSmoothingMode::kCurve:
      return "curve";
  }
  return "unknown";
}

SmoothedControlProjection
smoothControlProjection(const std::span<const TrajectoryPointSample> samples,
                        const TrajectoryProjection& raw_projection,
                        const VelocityFollowerConfig& config) {
  SmoothedControlProjection result{.projection = raw_projection};
  ControlProjectionSmoothingDiagnostics& diagnostics = result.diagnostics;
  TrajectoryProjection& control_projection = result.projection;
  diagnostics.raw_tangent = raw_projection.tangent;
  if (!control_projection.valid ||
      !(norm(control_projection.tangent) > kTinyDistanceM) ||
      !trajectorySamplesAreUsable(samples)) {
    return result;
  }

  const double straight_back_m =
      sanitizedPositive(config.control_tangent_smoothing_back_m, 8.0, 0.0, 1000.0);
  const double straight_forward_m =
      sanitizedPositive(config.control_tangent_smoothing_forward_m, 18.0, 0.0, 1000.0);
  const std::optional<ControlProjectionSmoothingWindow> straight_window =
      buildControlProjectionSmoothingWindow(samples, control_projection.s_m,
                                            straight_back_m, straight_forward_m);
  const double straight_max_heading_span_rad =
      sanitizedPositive(config.control_tangent_smoothing_max_heading_span_rad,
                        12.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double straight_max_abs_curvature = sanitizedPositive(
      config.control_tangent_smoothing_max_abs_curvature_1pm, 0.015, 0.0, 1000.0);
  if (straight_window.has_value()) {
    diagnostics.heading_span_rad = straight_window->heading_span_rad;
    diagnostics.max_abs_curvature_1pm = straight_window->max_abs_curvature_1pm;
    diagnostics.window_start_s_m = straight_window->window_start_s_m;
    diagnostics.window_end_s_m = straight_window->window_end_s_m;
    diagnostics.curvature_feedforward_context_scale =
        curvatureFeedforwardContextScale(*straight_window, config);
    if (straight_window->heading_span_rad <= straight_max_heading_span_rad &&
        straight_window->max_abs_curvature_1pm <= straight_max_abs_curvature &&
        dot(straight_window->tangent, control_projection.tangent) > 0.0) {
      control_projection.tangent = straight_window->tangent;
      control_projection.curvature_1pm = smoothedStraightishCurvature1pm(
          *straight_window, straight_max_heading_span_rad, straight_max_abs_curvature);
      if (!(std::abs(control_projection.curvature_1pm) > kTinyDistanceM)) {
        diagnostics.curvature_feedforward_context_scale = 0.0;
      }
      diagnostics.applied = true;
      diagnostics.mode = ControlProjectionSmoothingMode::kStraight;
      return result;
    }
  }

  const double curve_back_m =
      sanitizedPositive(config.control_curve_smoothing_back_m, 2.0, 0.0, 1000.0);
  const double curve_forward_m =
      sanitizedPositive(config.control_curve_smoothing_forward_m, 6.0, 0.0, 1000.0);
  const std::optional<ControlProjectionSmoothingWindow> curve_window =
      buildControlProjectionSmoothingWindow(samples, control_projection.s_m,
                                            curve_back_m, curve_forward_m);
  if (!curve_window.has_value()) {
    return result;
  }
  diagnostics.heading_span_rad = curve_window->heading_span_rad;
  diagnostics.max_abs_curvature_1pm = curve_window->max_abs_curvature_1pm;
  diagnostics.window_start_s_m = curve_window->window_start_s_m;
  diagnostics.window_end_s_m = curve_window->window_end_s_m;
  diagnostics.curvature_feedforward_context_scale =
      curvatureFeedforwardContextScale(*curve_window, config);
  const double curve_max_heading_span_rad =
      sanitizedPositive(config.control_curve_smoothing_max_heading_span_rad,
                        45.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  if (!(curve_window->heading_span_rad <= curve_max_heading_span_rad) ||
      dot(curve_window->tangent, control_projection.tangent) <= 0.0) {
    return result;
  }

  control_projection.tangent = curve_window->tangent;
  control_projection.curvature_1pm = curve_window->mean_curvature_1pm;
  diagnostics.applied = true;
  diagnostics.mode = ControlProjectionSmoothingMode::kCurve;
  return result;
}

} // namespace drone_city_nav

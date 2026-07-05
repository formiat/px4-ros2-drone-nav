#include "turn_smoothing_internal.hpp"

namespace drone_city_nav::turn_smoothing_detail {

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] Point2 leftNormal(const Point2 tangent) noexcept {
  return Point2{-tangent.y, tangent.x};
}

[[nodiscard]] double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

[[nodiscard]] double headingOf(const Point2 vector) noexcept {
  return std::atan2(vector.y, vector.x);
}

[[nodiscard]] double signedHeadingDelta(const Point2 previous,
                                        const Point2 next) noexcept {
  return normalizeAngle(headingOf(next) - headingOf(previous));
}

[[nodiscard]] double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double
curvatureSpeedLimitMps(const double radius_m,
                       const VelocityFollowerConfig& config) noexcept {
  if (!(radius_m > kTinyDistanceM) || !std::isfinite(radius_m)) {
    return sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  }
  const double cruise_speed =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double min_turn_speed = std::min(
      sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0), cruise_speed);
  const double max_lateral_accel =
      sanitizedPositive(config.turn_speed_lateral_accel_mps2, 5.0, 1.0e-6, 100.0);
  return std::clamp(std::sqrt(max_lateral_accel * radius_m), min_turn_speed,
                    cruise_speed);
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] std::int64_t quantizedCacheValue(const double value) noexcept {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<std::int64_t>(std::llround(value * 1.0e6));
}

[[nodiscard]] std::vector<double>
distanceFallbackCandidates(const double max_distance_m) {
  std::vector<double> candidates;
  if (!(max_distance_m > kTinyDistanceM)) {
    return candidates;
  }
  constexpr double kFallbackStepM = 5.0;
  constexpr double kMinFallbackDistanceM = 5.0;
  constexpr double kMaxFallbackDistanceM = 60.0;
  double candidate = kMaxFallbackDistanceM;
  while (candidate >= kMinFallbackDistanceM - kTinyDistanceM) {
    candidates.push_back(candidate);
    candidate -= kFallbackStepM;
  }
  return candidates;
}

[[nodiscard]] std::array<double, 6U> relaxedTangentAngleCandidatesRad() noexcept {
  return std::array<double, 6U>{
      5.0 * std::numbers::pi / 180.0,  10.0 * std::numbers::pi / 180.0,
      15.0 * std::numbers::pi / 180.0, 20.0 * std::numbers::pi / 180.0,
      25.0 * std::numbers::pi / 180.0, 30.0 * std::numbers::pi / 180.0};
}

[[nodiscard]] double discreteCurvature(const Point2 previous, const Point2 current,
                                       const Point2 next) {
  const double a = distance(previous, current);
  const double b = distance(current, next);
  const double c = distance(previous, next);
  if (!(a > kTinyDistanceM) || !(b > kTinyDistanceM) || !(c > kTinyDistanceM)) {
    return 0.0;
  }
  const double signed_double_area = cross(current - previous, next - previous);
  return 2.0 * signed_double_area / (a * b * c);
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[1U].point - samples[0U].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples,
                                const std::size_t start_index,
                                const std::size_t end_index) {
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = start_index + 1U; i <= end_index; ++i) {
    length += distance(samples[i - 1U].point, samples[i].point);
  }
  return length;
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return 0.0;
  }
  return pathLength(samples, 0U, samples.size() - 1U);
}

[[nodiscard]] std::optional<CorridorSample>
corridorSampleAtS(const std::span<const CorridorSample> corridor_samples,
                  const double s_m) {
  if (corridor_samples.empty()) {
    return std::nullopt;
  }
  if (s_m <= corridor_samples.front().s_m) {
    return corridor_samples.front();
  }
  for (std::size_t i = 1U; i < corridor_samples.size(); ++i) {
    if (s_m > corridor_samples[i].s_m && i + 1U < corridor_samples.size()) {
      continue;
    }
    const CorridorSample& previous = corridor_samples[i - 1U];
    const CorridorSample& next = corridor_samples[i];
    const double span = next.s_m - previous.s_m;
    const double t =
        span > kTinyDistanceM ? std::clamp((s_m - previous.s_m) / span, 0.0, 1.0) : 0.0;
    CorridorSample sample{};
    sample.s_m = previous.s_m + (next.s_m - previous.s_m) * t;
    sample.route_center =
        previous.route_center + (next.route_center - previous.route_center) * t;
    sample.center = previous.center + (next.center - previous.center) * t;
    sample.tangent =
        normalized(previous.tangent + (next.tangent - previous.tangent) * t);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      sample.tangent = previous.tangent;
    }
    sample.normal = leftNormal(sample.tangent);
    sample.left_bound_m =
        previous.left_bound_m + (next.left_bound_m - previous.left_bound_m) * t;
    sample.right_bound_m =
        previous.right_bound_m + (next.right_bound_m - previous.right_bound_m) * t;
    sample.clearance_m =
        previous.clearance_m + (next.clearance_m - previous.clearance_m) * t;
    sample.center_recovery_m =
        previous.center_recovery_m +
        (next.center_recovery_m - previous.center_recovery_m) * t;
    return sample;
  }
  return corridor_samples.back();
}

} // namespace drone_city_nav::turn_smoothing_detail

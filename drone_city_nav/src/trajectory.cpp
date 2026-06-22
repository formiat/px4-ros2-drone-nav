#include "drone_city_nav/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
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

[[nodiscard]] double positiveAngleDelta(double from_rad, double to_rad) noexcept {
  double delta = std::fmod(to_rad - from_rad, kTwoPi);
  if (delta < 0.0) {
    delta += kTwoPi;
  }
  return delta;
}

[[nodiscard]] double clampSegmentT(const double value) noexcept {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, 0.0, 1.0);
}

[[nodiscard]] double segmentEndS(const TrajectorySegment& segment) noexcept {
  return segment.s_start_m + std::max(0.0, segment.length_m);
}

[[nodiscard]] std::size_t
segmentIndexAtS(const std::span<const TrajectorySegment> trajectory,
                const double s_m) noexcept {
  if (trajectory.empty()) {
    return 0U;
  }
  const double clamped_s = std::clamp(s_m, 0.0, trajectoryLengthM(trajectory));
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    if (clamped_s <= segmentEndS(trajectory[i]) || i + 1U == trajectory.size()) {
      return i;
    }
  }
  return trajectory.size() - 1U;
}

[[nodiscard]] double segmentTAtS(const TrajectorySegment& segment,
                                 const double s_m) noexcept {
  if (!(segment.length_m > kTinyDistanceM)) {
    return 0.0;
  }
  return clampSegmentT((s_m - segment.s_start_m) / segment.length_m);
}

[[nodiscard]] Point2 pointOnSegment(const TrajectorySegment& segment,
                                    const double segment_t) noexcept {
  const double t = clampSegmentT(segment_t);
  if (segment.kind == TrajectorySegmentKind::kLine) {
    return Point2{segment.start.x + (segment.end.x - segment.start.x) * t,
                  segment.start.y + (segment.end.y - segment.start.y) * t};
  }

  const double angle = segment.start_angle_rad + segment.sweep_rad * t;
  return Point2{segment.center.x + std::cos(angle) * segment.radius_m,
                segment.center.y + std::sin(angle) * segment.radius_m};
}

[[nodiscard]] Point2 tangentOnSegment(const TrajectorySegment& segment,
                                      const double segment_t) noexcept {
  if (segment.kind == TrajectorySegmentKind::kLine) {
    return normalized(segment.end - segment.start);
  }

  const double angle =
      segment.start_angle_rad + segment.sweep_rad * clampSegmentT(segment_t);
  const double sign = segment.sweep_rad >= 0.0 ? 1.0 : -1.0;
  return Point2{-std::sin(angle) * sign, std::cos(angle) * sign};
}

[[nodiscard]] bool segmentIsFinite(const TrajectorySegment& segment) noexcept {
  if (!finite2D(segment.start) || !finite2D(segment.end) ||
      !std::isfinite(segment.length_m) || !std::isfinite(segment.s_start_m)) {
    return false;
  }
  if (segment.kind == TrajectorySegmentKind::kLine) {
    return true;
  }
  return finite2D(segment.center) && std::isfinite(segment.radius_m) &&
         segment.radius_m > kTinyDistanceM && std::isfinite(segment.start_angle_rad) &&
         std::isfinite(segment.sweep_rad);
}

} // namespace

const char* trajectorySegmentKindName(const TrajectorySegmentKind kind) noexcept {
  switch (kind) {
    case TrajectorySegmentKind::kLine:
      return "line";
    case TrajectorySegmentKind::kArc:
      return "arc";
  }
  return "unknown";
}

TrajectorySegment makeLineSegment(const Point2 start, const Point2 end) {
  TrajectorySegment segment{};
  segment.kind = TrajectorySegmentKind::kLine;
  segment.start = start;
  segment.end = end;
  segment.length_m = distance(start, end);
  return segment;
}

TrajectorySegment makeArcSegment(const Point2 start, const Point2 end,
                                 const Point2 center, const double sweep_rad) {
  TrajectorySegment segment{};
  segment.kind = TrajectorySegmentKind::kArc;
  segment.start = start;
  segment.end = end;
  segment.center = center;
  segment.radius_m = distance(center, start);
  segment.start_angle_rad = std::atan2(start.y - center.y, start.x - center.x);
  segment.sweep_rad = sweep_rad;
  if (segment.radius_m > kTinyDistanceM && std::isfinite(sweep_rad)) {
    segment.length_m = std::abs(sweep_rad) * segment.radius_m;
  }
  return segment;
}

void assignTrajectoryStationing(std::vector<TrajectorySegment>& trajectory) {
  double s_m = 0.0;
  for (TrajectorySegment& segment : trajectory) {
    segment.s_start_m = s_m;
    s_m += std::max(0.0, segment.length_m);
  }
}

std::vector<TrajectorySegment>
lineTrajectoryFromPoints(const std::span<const Point2> points) {
  std::vector<TrajectorySegment> trajectory;
  if (points.size() < 2U) {
    return trajectory;
  }
  trajectory.reserve(points.size() - 1U);
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    TrajectorySegment segment = makeLineSegment(points[i], points[i + 1U]);
    if (segment.length_m > kTinyDistanceM && segmentIsFinite(segment)) {
      trajectory.push_back(segment);
    }
  }
  assignTrajectoryStationing(trajectory);
  return trajectory;
}

bool trajectoryIsUsable(const std::span<const TrajectorySegment> trajectory) {
  if (trajectory.empty()) {
    return false;
  }
  return std::ranges::all_of(trajectory, [](const TrajectorySegment& segment) {
    return segment.length_m > kTinyDistanceM && segmentIsFinite(segment);
  });
}

TrajectoryMetrics
trajectoryMetrics(const std::span<const TrajectorySegment> trajectory) {
  TrajectoryMetrics metrics{};
  for (const TrajectorySegment& segment : trajectory) {
    metrics.length_m += std::max(0.0, segment.length_m);
    if (segment.kind == TrajectorySegmentKind::kArc) {
      ++metrics.arc_segments;
    } else {
      ++metrics.line_segments;
    }
  }
  return metrics;
}

double trajectoryLengthM(const std::span<const TrajectorySegment> trajectory) {
  if (trajectory.empty()) {
    return 0.0;
  }
  const TrajectorySegment& last = trajectory.back();
  return segmentEndS(last);
}

Point2 trajectoryPointAtS(const std::span<const TrajectorySegment> trajectory,
                          const double s_m) {
  if (trajectory.empty()) {
    return Point2{};
  }
  const std::size_t index = segmentIndexAtS(trajectory, s_m);
  const TrajectorySegment& segment = trajectory[index];
  return pointOnSegment(segment, segmentTAtS(segment, s_m));
}

Point2 trajectoryTangentAtS(const std::span<const TrajectorySegment> trajectory,
                            const double s_m) {
  if (trajectory.empty()) {
    return Point2{};
  }
  const std::size_t index = segmentIndexAtS(trajectory, s_m);
  const TrajectorySegment& segment = trajectory[index];
  return tangentOnSegment(segment, segmentTAtS(segment, s_m));
}

double trajectoryCurvatureAtS(const std::span<const TrajectorySegment> trajectory,
                              const double s_m) {
  if (trajectory.empty()) {
    return 0.0;
  }
  const std::size_t index = segmentIndexAtS(trajectory, s_m);
  const TrajectorySegment& segment = trajectory[index];
  if (segment.kind != TrajectorySegmentKind::kArc ||
      !(segment.radius_m > kTinyDistanceM)) {
    return 0.0;
  }
  return (segment.sweep_rad >= 0.0 ? 1.0 : -1.0) / segment.radius_m;
}

std::optional<TrajectoryProjection>
projectOnTrajectory(const std::span<const TrajectorySegment> trajectory,
                    const Point2 point, const double minimum_s_m) {
  if (!finite2D(point) || !trajectoryIsUsable(trajectory)) {
    return std::nullopt;
  }

  const double min_s = std::clamp(std::isfinite(minimum_s_m) ? minimum_s_m : 0.0, 0.0,
                                  trajectoryLengthM(trajectory));
  std::optional<TrajectoryProjection> best;
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    const TrajectorySegment& segment = trajectory[i];
    const double segment_min_t =
        segmentEndS(segment) <= min_s
            ? 1.0
            : std::max(0.0, (min_s - segment.s_start_m) / segment.length_m);

    double t = 0.0;
    if (segment.kind == TrajectorySegmentKind::kLine) {
      const Point2 line = segment.end - segment.start;
      const double length_sq = squaredDistance(segment.start, segment.end);
      if (!(length_sq > kTinyDistanceM * kTinyDistanceM)) {
        continue;
      }
      t = ((point.x - segment.start.x) * line.x +
           (point.y - segment.start.y) * line.y) /
          length_sq;
    } else {
      const double point_angle =
          std::atan2(point.y - segment.center.y, point.x - segment.center.x);
      if (segment.sweep_rad >= 0.0) {
        t = positiveAngleDelta(segment.start_angle_rad, point_angle) /
            std::max(segment.sweep_rad, kTinyDistanceM);
      } else {
        t = positiveAngleDelta(point_angle, segment.start_angle_rad) /
            std::max(std::abs(segment.sweep_rad), kTinyDistanceM);
      }
    }
    t = std::max(clampSegmentT(t), clampSegmentT(segment_min_t));

    const Point2 projected = pointOnSegment(segment, t);
    const double distance_sq = squaredDistance(point, projected);
    if (!best.has_value() || distance_sq < best->distance_sq) {
      TrajectoryProjection projection{};
      projection.valid = true;
      projection.segment_index = i;
      projection.segment_t = t;
      projection.s_m = segment.s_start_m + segment.length_m * t;
      projection.point = projected;
      projection.tangent = tangentOnSegment(segment, t);
      projection.curvature_1pm =
          segment.kind == TrajectorySegmentKind::kArc &&
                  segment.radius_m > kTinyDistanceM
              ? (segment.sweep_rad >= 0.0 ? 1.0 : -1.0) / segment.radius_m
              : 0.0;
      projection.distance_sq = distance_sq;
      best = projection;
    }
  }

  return best;
}

std::vector<Point2>
sampleTrajectory(const std::span<const TrajectorySegment> trajectory,
                 const double step_m) {
  std::vector<Point2> samples;
  if (!trajectoryIsUsable(trajectory)) {
    return samples;
  }

  const double length = trajectoryLengthM(trajectory);
  const double step = std::clamp(std::isfinite(step_m) ? step_m : 1.0, 0.05, 1000.0);
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(length / step)) + 1U;
  samples.reserve(sample_count + 1U);
  for (std::size_t i = 0U; i <= sample_count; ++i) {
    const double s = std::min(length, static_cast<double>(i) * step);
    samples.push_back(trajectoryPointAtS(trajectory, s));
    if (s >= length) {
      break;
    }
  }
  if (samples.empty() || squaredDistance(samples.back(), trajectory.back().end) >
                             kTinyDistanceM * kTinyDistanceM) {
    samples.push_back(trajectory.back().end);
  }
  return samples;
}

} // namespace drone_city_nav

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

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
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

[[nodiscard]] bool sampleIsFinite(const TrajectoryPointSample& sample) noexcept {
  return std::isfinite(sample.s_m) && finite2D(sample.point) &&
         finite2D(sample.tangent) && std::isfinite(sample.curvature_1pm) &&
         std::isfinite(sample.z_m);
}

[[nodiscard]] double signedCurvatureFromTriplet(const Point2 previous,
                                                const Point2 current,
                                                const Point2 next) noexcept {
  const Point2 a = current - previous;
  const Point2 b = next - current;
  const Point2 c = next - previous;
  const double ab = norm(a);
  const double bc = norm(b);
  const double ac = norm(c);
  const double denominator = ab * bc * ac;
  if (!(denominator > kTinyDistanceM)) {
    return 0.0;
  }
  return 2.0 * cross(a, b) / denominator;
}

[[nodiscard]] Point2 sampleTangentFromNeighbors(const std::span<const Point2> points,
                                                const std::size_t index) noexcept {
  if (points.size() < 2U || index >= points.size()) {
    return Point2{1.0, 0.0};
  }
  if (index == 0U) {
    return normalized(points[1U] - points[0U]);
  }
  if (index + 1U == points.size()) {
    return normalized(points[index] - points[index - 1U]);
  }

  const Point2 previous_direction = normalized(points[index] - points[index - 1U]);
  const Point2 next_direction = normalized(points[index + 1U] - points[index]);
  Point2 tangent = normalized(Point2{previous_direction.x + next_direction.x,
                                     previous_direction.y + next_direction.y});
  if (!(norm(tangent) > kTinyDistanceM)) {
    tangent = next_direction;
  }
  if (!(norm(tangent) > kTinyDistanceM)) {
    tangent = previous_direction;
  }
  return tangent;
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

std::vector<TrajectorySegment>
lineTrajectoryFromSamples(const std::span<const TrajectoryPointSample> samples) {
  std::vector<Point2> points;
  points.reserve(samples.size());
  for (const TrajectoryPointSample& sample : samples) {
    if (!sampleIsFinite(sample)) {
      return {};
    }
    points.push_back(sample.point);
  }
  return lineTrajectoryFromPoints(points);
}

bool trajectoryIsUsable(const std::span<const TrajectorySegment> trajectory) {
  if (trajectory.empty()) {
    return false;
  }
  return std::ranges::all_of(trajectory, [](const TrajectorySegment& segment) {
    return segment.length_m > kTinyDistanceM && segmentIsFinite(segment);
  });
}

bool trajectorySamplesAreUsable(const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return false;
  }
  double previous_s = -std::numeric_limits<double>::infinity();
  for (const TrajectoryPointSample& sample : samples) {
    if (!sampleIsFinite(sample) || sample.s_m + kTinyDistanceM < previous_s ||
        !(norm(sample.tangent) > kTinyDistanceM)) {
      return false;
    }
    previous_s = sample.s_m;
  }
  return true;
}

void populateTrajectorySampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    samples[i].curvature_1pm = 0.0;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = signedCurvatureFromTriplet(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

void assignTrajectorySampleAltitude(const std::span<TrajectoryPointSample> samples,
                                    const double altitude_m) {
  for (TrajectoryPointSample& sample : samples) {
    sample.z_m = altitude_m;
  }
}

double trajectorySampleAltitudeAtS(const std::span<const TrajectoryPointSample> samples,
                                   const double s_m) {
  if (!trajectorySamplesAreUsable(samples)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double clamped_s =
      std::clamp(std::isfinite(s_m) ? s_m : 0.0, 0.0, samples.back().s_m);
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (clamped_s > end.s_m && i + 2U < samples.size()) {
      continue;
    }
    const double station_delta_m = end.s_m - start.s_m;
    if (!(station_delta_m > kTinyDistanceM)) {
      return start.z_m;
    }
    const double t = std::clamp((clamped_s - start.s_m) / station_delta_m, 0.0, 1.0);
    return start.z_m * (1.0 - t) + end.z_m * t;
  }
  return samples.back().z_m;
}

TrajectoryVerticalTarget
trajectoryVerticalTargetAtS(const std::span<const TrajectoryPointSample> samples,
                            const double s_m) {
  TrajectoryVerticalTarget target{};
  if (!trajectorySamplesAreUsable(samples)) {
    return target;
  }

  const double clamped_s =
      std::clamp(std::isfinite(s_m) ? s_m : 0.0, 0.0, samples.back().s_m);
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (clamped_s > end.s_m && i + 2U < samples.size()) {
      continue;
    }

    const double station_delta_m = end.s_m - start.s_m;
    const double t =
        station_delta_m > kTinyDistanceM
            ? std::clamp((clamped_s - start.s_m) / station_delta_m, 0.0, 1.0)
            : 0.0;
    const TrajectoryPointSample& nearest = t <= 0.5 ? start : end;
    target.valid = true;
    target.s_m = clamped_s;
    target.z_m = start.z_m * (1.0 - t) + end.z_m * t;
    target.vertical_slope_dz_ds =
        start.vertical_slope_dz_ds * (1.0 - t) + end.vertical_slope_dz_ds * t;
    target.vertical_constraint_active =
        start.vertical_constraint_active || end.vertical_constraint_active;
    if (!nearest.vertical_profile_passage_id.empty()) {
      target.vertical_profile_passage_id = nearest.vertical_profile_passage_id;
    } else if (!start.vertical_profile_passage_id.empty()) {
      target.vertical_profile_passage_id = start.vertical_profile_passage_id;
    } else {
      target.vertical_profile_passage_id = end.vertical_profile_passage_id;
    }
    return target;
  }

  target.valid = true;
  target.s_m = samples.back().s_m;
  target.z_m = samples.back().z_m;
  target.vertical_slope_dz_ds = samples.back().vertical_slope_dz_ds;
  target.vertical_constraint_active = samples.back().vertical_constraint_active;
  target.vertical_profile_passage_id = samples.back().vertical_profile_passage_id;
  return target;
}

TrajectoryAltitudeStats
trajectoryAltitudeStats(const std::span<const TrajectoryPointSample> samples) {
  TrajectoryAltitudeStats stats{};
  if (samples.empty()) {
    return stats;
  }

  double sum_z_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const double z_m = samples[i].z_m;
    if (!std::isfinite(z_m)) {
      return TrajectoryAltitudeStats{};
    }
    if (i == 0U) {
      stats.min_z_m = z_m;
      stats.max_z_m = z_m;
    } else {
      stats.min_z_m = std::min(stats.min_z_m, z_m);
      stats.max_z_m = std::max(stats.max_z_m, z_m);
    }
    sum_z_m += z_m;
  }
  stats.valid = true;
  stats.mean_z_m = sum_z_m / static_cast<double>(samples.size());
  return stats;
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

std::optional<TrajectoryProjection>
projectOnTrajectorySamples(const std::span<const TrajectoryPointSample> samples,
                           const Point2 point, const double minimum_s_m) {
  if (!finite2D(point) || !trajectorySamplesAreUsable(samples)) {
    return std::nullopt;
  }

  const double max_s = std::max(0.0, samples.back().s_m);
  const double min_s =
      std::clamp(std::isfinite(minimum_s_m) ? minimum_s_m : 0.0, 0.0, max_s);
  std::optional<TrajectoryProjection> best;
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    const double station_delta_m = end.s_m - start.s_m;
    const Point2 line = end.point - start.point;
    const double length_sq = squaredDistance(start.point, end.point);
    if (!(station_delta_m > kTinyDistanceM) ||
        !(length_sq > kTinyDistanceM * kTinyDistanceM)) {
      continue;
    }

    const double segment_min_t =
        end.s_m <= min_s ? 1.0 : std::max(0.0, (min_s - start.s_m) / station_delta_m);
    double t =
        ((point.x - start.point.x) * line.x + (point.y - start.point.y) * line.y) /
        length_sq;
    t = std::max(clampSegmentT(t), clampSegmentT(segment_min_t));

    const Point2 projected{start.point.x + line.x * t, start.point.y + line.y * t};
    const double distance_sq = squaredDistance(point, projected);
    if (best.has_value() && distance_sq >= best->distance_sq) {
      continue;
    }

    Point2 tangent = normalized(start.tangent * (1.0 - t) + end.tangent * t);
    if (!(norm(tangent) > kTinyDistanceM)) {
      tangent = normalized(line);
    }
    if (!(norm(tangent) > kTinyDistanceM)) {
      continue;
    }

    TrajectoryProjection projection{};
    projection.valid = true;
    projection.segment_index = i;
    projection.segment_t = t;
    projection.s_m = start.s_m + station_delta_m * t;
    projection.point = projected;
    projection.tangent = tangent;
    projection.curvature_1pm = start.curvature_1pm * (1.0 - t) + end.curvature_1pm * t;
    projection.distance_sq = distance_sq;
    best = projection;
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

std::vector<TrajectoryPointSample>
sampleTrajectoryDetailed(const std::span<const TrajectorySegment> trajectory,
                         const double step_m) {
  std::vector<TrajectoryPointSample> samples;
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
    TrajectoryPointSample sample{};
    sample.s_m = s;
    sample.point = trajectoryPointAtS(trajectory, s);
    sample.tangent = trajectoryTangentAtS(trajectory, s);
    sample.curvature_1pm = trajectoryCurvatureAtS(trajectory, s);
    samples.push_back(sample);
    if (s >= length) {
      break;
    }
  }
  if (samples.empty() || squaredDistance(samples.back().point, trajectory.back().end) >
                             kTinyDistanceM * kTinyDistanceM) {
    TrajectoryPointSample sample{};
    sample.s_m = length;
    sample.point = trajectory.back().end;
    sample.tangent = trajectoryTangentAtS(trajectory, length);
    sample.curvature_1pm = trajectoryCurvatureAtS(trajectory, length);
    samples.push_back(sample);
  }
  return samples;
}

std::vector<TrajectoryPointSample>
trajectoryPointSamplesFromPoints(const std::span<const Point2> points) {
  std::vector<TrajectoryPointSample> samples;
  if (points.size() < 2U) {
    return samples;
  }

  samples.reserve(points.size());
  double s_m = 0.0;
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (!finite2D(points[i])) {
      return {};
    }
    if (i > 0U) {
      const double ds = distance(points[i - 1U], points[i]);
      if (!(ds > kTinyDistanceM) || !std::isfinite(ds)) {
        continue;
      }
      s_m += ds;
    }

    TrajectoryPointSample sample{};
    sample.s_m = s_m;
    sample.point = points[i];
    sample.tangent = sampleTangentFromNeighbors(points, i);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      return {};
    }
    if (i > 0U && i + 1U < points.size()) {
      sample.curvature_1pm =
          signedCurvatureFromTriplet(points[i - 1U], points[i], points[i + 1U]);
    }
    samples.push_back(sample);
  }

  if (samples.size() < 2U) {
    return {};
  }
  return samples;
}

} // namespace drone_city_nav

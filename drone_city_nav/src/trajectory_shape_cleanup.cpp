#include "drone_city_nav/trajectory_shape_cleanup.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kIsolatedCurvatureSpikeMin1pm = 0.05;
constexpr double kIsolatedCurvatureSpikeNeighborRatio = 0.45;
constexpr double kCurvatureSpikeGeometryBlend = 0.65;
constexpr std::size_t kCurvatureSpikeSmoothingPasses = 2U;

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = std::hypot(point.x, point.y);
  if (!(length > kTinyDistanceM)) {
    return Point2{1.0, 0.0};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double signedCurvatureFromTriplet(const Point2 previous,
                                                const Point2 current,
                                                const Point2 next) noexcept {
  const Point2 a = current - previous;
  const Point2 b = next - current;
  const double ab = distance(previous, current);
  const double bc = distance(current, next);
  const double ac = distance(previous, next);
  const double denominator = ab * bc * ac;
  if (!(denominator > kTinyDistanceM)) {
    return 0.0;
  }
  return 2.0 * cross(a, b) / denominator;
}

[[nodiscard]] bool
isIsolatedCurvatureSpike(const std::span<const TrajectoryPointSample> samples,
                         const std::size_t index) noexcept {
  if (index == 0U || index + 1U >= samples.size()) {
    return false;
  }
  const double current = std::abs(samples[index].curvature_1pm);
  if (!(current >= kIsolatedCurvatureSpikeMin1pm)) {
    return false;
  }
  const double previous = std::abs(samples[index - 1U].curvature_1pm);
  const double next = std::abs(samples[index + 1U].curvature_1pm);
  return previous <= current * kIsolatedCurvatureSpikeNeighborRatio &&
         next <= current * kIsolatedCurvatureSpikeNeighborRatio;
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
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

[[nodiscard]] bool segmentTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                      const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  return std::ranges::all_of(
      grid.cellsOnLine(*start_cell, *end_cell),
      [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
}

[[nodiscard]] bool localSpikeSmoothingTraversable(const OccupancyGrid2D& grid,
                                                  const Point2 previous,
                                                  const Point2 current,
                                                  const Point2 next) {
  return segmentTraversable(grid, previous, current) &&
         segmentTraversable(grid, current, next);
}

[[nodiscard]] bool
smoothSingleIsolatedCurvatureSpike(std::vector<TrajectoryPointSample>& samples,
                                   const OccupancyGrid2D& grid,
                                   const std::size_t index) {
  if (!isIsolatedCurvatureSpike(samples, index)) {
    return false;
  }

  const Point2 previous = samples[index - 1U].point;
  const Point2 current = samples[index].point;
  const Point2 next = samples[index + 1U].point;
  const Point2 midpoint{0.5 * (previous.x + next.x), 0.5 * (previous.y + next.y)};
  const Point2 candidate{
      current.x + (midpoint.x - current.x) * kCurvatureSpikeGeometryBlend,
      current.y + (midpoint.y - current.y) * kCurvatureSpikeGeometryBlend,
  };
  if (!localSpikeSmoothingTraversable(grid, previous, candidate, next)) {
    return false;
  }

  const double current_abs = std::abs(samples[index].curvature_1pm);
  const double candidate_abs =
      std::abs(signedCurvatureFromTriplet(previous, candidate, next));
  if (!(candidate_abs < current_abs * 0.75)) {
    return false;
  }

  samples[index].point = candidate;
  return true;
}

} // namespace

std::size_t countIsolatedCurvatureSpikes(
    const std::span<const TrajectoryPointSample> samples) noexcept {
  std::size_t count = 0U;
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    if (isIsolatedCurvatureSpike(samples, i)) {
      ++count;
    }
  }
  return count;
}

double maxIsolatedCurvatureSpike(
    const std::span<const TrajectoryPointSample> samples) noexcept {
  double max_spike = 0.0;
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    if (isIsolatedCurvatureSpike(samples, i)) {
      max_spike = std::max(max_spike, std::abs(samples[i].curvature_1pm));
    }
  }
  return max_spike;
}

std::size_t
smoothIsolatedCurvatureSpikeGeometry(std::vector<TrajectoryPointSample>& samples,
                                     const OccupancyGrid2D& grid) {
  std::size_t smoothed = 0U;
  for (std::size_t pass = 0U; pass < kCurvatureSpikeSmoothingPasses; ++pass) {
    bool changed = false;
    for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
      if (smoothSingleIsolatedCurvatureSpike(samples, grid, i)) {
        populateSampleGeometry(samples);
        ++smoothed;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }
  return smoothed;
}

} // namespace drone_city_nav

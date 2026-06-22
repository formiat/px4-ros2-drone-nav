#include "drone_city_nav/corner_rounding.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <ranges>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kTinyAngleRad = 1.0e-6;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

struct CornerArcCandidate {
  TrajectorySegment arc;
  Point2 tangent_start{};
  Point2 tangent_end{};
  double radius_m{0.0};
};

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

[[nodiscard]] Point2 leftNormal(const Point2 direction) noexcept {
  return Point2{-direction.y, direction.x};
}

[[nodiscard]] Point2 rightNormal(const Point2 direction) noexcept {
  return Point2{direction.y, -direction.x};
}

[[nodiscard]] double positiveAngleDelta(double from_rad, double to_rad) noexcept {
  double delta = std::fmod(to_rad - from_rad, kTwoPi);
  if (delta < 0.0) {
    delta += kTwoPi;
  }
  return delta;
}

[[nodiscard]] double turnAngleRad(const Point2 incoming_unit,
                                  const Point2 outgoing_unit) noexcept {
  return std::acos(std::clamp(dot(incoming_unit, outgoing_unit), -1.0, 1.0));
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] std::optional<CornerArcCandidate>
buildCandidate(const Point2 current_start, const Point2 corner, const Point2 next,
               const double radius_m, const double min_segment_remainder_m) {
  if (!finite2D(current_start) || !finite2D(corner) || !finite2D(next) ||
      !(radius_m > kTinyDistanceM)) {
    return std::nullopt;
  }

  const Point2 incoming = corner - current_start;
  const Point2 outgoing = next - corner;
  const double incoming_length = norm(incoming);
  const double outgoing_length = norm(outgoing);
  if (incoming_length <= min_segment_remainder_m + kTinyDistanceM ||
      outgoing_length <= min_segment_remainder_m + kTinyDistanceM) {
    return std::nullopt;
  }

  const Point2 incoming_unit = normalized(incoming);
  const Point2 outgoing_unit = normalized(outgoing);
  const double angle = turnAngleRad(incoming_unit, outgoing_unit);
  if (angle <= kTinyAngleRad || std::abs(std::numbers::pi - angle) <= kTinyAngleRad) {
    return std::nullopt;
  }

  const double tangent_distance = radius_m * std::tan(angle * 0.5);
  if (!(tangent_distance > kTinyDistanceM) ||
      tangent_distance > incoming_length - min_segment_remainder_m ||
      tangent_distance > outgoing_length - min_segment_remainder_m) {
    return std::nullopt;
  }

  const double turn_sign = cross(incoming_unit, outgoing_unit);
  if (std::abs(turn_sign) <= kTinyDistanceM) {
    return std::nullopt;
  }

  const Point2 tangent_start = corner - incoming_unit * tangent_distance;
  const Point2 tangent_end = corner + outgoing_unit * tangent_distance;
  const Point2 inward_normal =
      turn_sign > 0.0 ? leftNormal(incoming_unit) : rightNormal(incoming_unit);
  const Point2 center = tangent_start + inward_normal * radius_m;

  const double start_angle =
      std::atan2(tangent_start.y - center.y, tangent_start.x - center.x);
  const double end_angle =
      std::atan2(tangent_end.y - center.y, tangent_end.x - center.x);
  const double sweep = turn_sign > 0.0 ? positiveAngleDelta(start_angle, end_angle)
                                       : -positiveAngleDelta(end_angle, start_angle);
  if (!(std::abs(sweep) > kTinyAngleRad) ||
      std::abs(std::abs(sweep) - angle) > 1.0e-4) {
    return std::nullopt;
  }

  CornerArcCandidate candidate{};
  candidate.tangent_start = tangent_start;
  candidate.tangent_end = tangent_end;
  candidate.radius_m = radius_m;
  candidate.arc = makeArcSegment(tangent_start, tangent_end, center, sweep);
  return candidate;
}

[[nodiscard]] bool lineCellsAreTraversable(const OccupancyGrid2D& grid,
                                           const Point2 start, const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  const std::vector<GridIndex> cells =
      grid.cellsOnLine(start_cell.value(), end_cell.value());
  return std::ranges::all_of(
      cells, [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
}

[[nodiscard]] bool arcIsTraversable(const CornerArcCandidate& candidate,
                                    const OccupancyGrid2D* grid,
                                    const CornerRoundingConfig& config) {
  if (grid == nullptr) {
    return true;
  }
  const double sample_step =
      std::min(sanitizedPositive(config.collision_sample_step_m, 0.25, 0.05, 10.0),
               std::max(0.05, 0.5 * grid->resolution()));
  const std::size_t samples = static_cast<std::size_t>(
      std::ceil(std::max(candidate.arc.length_m, sample_step) / sample_step));

  Point2 previous = candidate.tangent_start;
  if (!lineCellsAreTraversable(*grid, previous, previous)) {
    return false;
  }
  for (std::size_t i = 1U; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double angle = candidate.arc.start_angle_rad + candidate.arc.sweep_rad * t;
    const Point2 current{
        candidate.arc.center.x + std::cos(angle) * candidate.arc.radius_m,
        candidate.arc.center.y + std::sin(angle) * candidate.arc.radius_m};
    if (!lineCellsAreTraversable(*grid, previous, current)) {
      return false;
    }
    previous = current;
  }
  return true;
}

void addLineIfUsable(std::vector<TrajectorySegment>& segments, const Point2 start,
                     const Point2 end) {
  TrajectorySegment line = makeLineSegment(start, end);
  if (line.length_m > kTinyDistanceM) {
    segments.push_back(line);
  }
}

void recordRadius(CornerRoundingStats& stats, const double radius_m) {
  if (stats.corners_rounded == 0U) {
    stats.min_radius_m = radius_m;
    stats.max_radius_m = radius_m;
    stats.mean_radius_m = radius_m;
    return;
  }
  stats.min_radius_m = std::min(stats.min_radius_m, radius_m);
  stats.max_radius_m = std::max(stats.max_radius_m, radius_m);
  stats.mean_radius_m =
      ((stats.mean_radius_m * static_cast<double>(stats.corners_rounded)) + radius_m) /
      static_cast<double>(stats.corners_rounded + 1U);
}

} // namespace

CornerRoundingResult roundCorners(const std::span<const Point2> path_points,
                                  const CornerRoundingConfig& config,
                                  const OccupancyGrid2D* const prohibited_grid) {
  CornerRoundingResult result{};
  result.stats.input_points = path_points.size();
  if (path_points.size() < 2U) {
    return result;
  }
  if (!config.enabled || path_points.size() < 3U) {
    result.segments = lineTrajectoryFromPoints(path_points);
    result.stats.output_segments = result.segments.size();
    return result;
  }

  const double min_radius = sanitizedPositive(config.min_radius_m, 3.0, 0.05, 1000.0);
  const double max_radius =
      std::max(min_radius, sanitizedPositive(config.max_radius_m, 30.0, 0.05, 1000.0));
  const double min_remainder =
      sanitizedPositive(config.min_segment_remainder_m, 1.0, 0.0, 1000.0);

  Point2 current_start = path_points.front();
  for (std::size_t corner_index = 1U; corner_index + 1U < path_points.size();
       ++corner_index) {
    const Point2 corner = path_points[corner_index];
    const Point2 next = path_points[corner_index + 1U];
    ++result.stats.corners_seen;

    const Point2 incoming = corner - current_start;
    const Point2 outgoing = next - corner;
    const double incoming_length = norm(incoming);
    const double outgoing_length = norm(outgoing);
    if (incoming_length <= kTinyDistanceM || outgoing_length <= kTinyDistanceM ||
        !finite2D(corner) || !finite2D(next)) {
      ++result.stats.skipped_degenerate;
      addLineIfUsable(result.segments, current_start, corner);
      current_start = corner;
      continue;
    }

    const double angle = turnAngleRad(normalized(incoming), normalized(outgoing));
    if (angle <= kTinyAngleRad) {
      ++result.stats.skipped_straight;
      continue;
    }

    const double tan_half = std::tan(angle * 0.5);
    if (!(tan_half > kTinyDistanceM)) {
      ++result.stats.skipped_degenerate;
      continue;
    }
    const double radius_by_lengths =
        (std::min(incoming_length, outgoing_length) - min_remainder) / tan_half;
    double radius = std::min(max_radius, radius_by_lengths);
    if (!(radius >= min_radius)) {
      ++result.stats.skipped_short_segments;
      addLineIfUsable(result.segments, current_start, corner);
      current_start = corner;
      continue;
    }

    std::optional<CornerArcCandidate> accepted;
    bool rejected_by_collision = false;
    for (int attempt = 0; attempt < 16 && radius >= min_radius; ++attempt) {
      const std::optional<CornerArcCandidate> candidate =
          buildCandidate(current_start, corner, next, radius, min_remainder);
      if (candidate.has_value()) {
        if (arcIsTraversable(*candidate, prohibited_grid, config)) {
          accepted = candidate;
          break;
        }
        rejected_by_collision = true;
      }
      radius *= 0.5;
    }

    if (!accepted.has_value()) {
      if (rejected_by_collision) {
        ++result.stats.skipped_collision;
      } else {
        ++result.stats.skipped_short_segments;
      }
      addLineIfUsable(result.segments, current_start, corner);
      current_start = corner;
      continue;
    }

    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): guarded above.
    const CornerArcCandidate accepted_candidate = accepted.value();
    addLineIfUsable(result.segments, current_start, accepted_candidate.tangent_start);
    result.segments.push_back(accepted_candidate.arc);
    recordRadius(result.stats, accepted_candidate.radius_m);
    ++result.stats.corners_rounded;
    current_start = accepted_candidate.tangent_end;
  }

  addLineIfUsable(result.segments, current_start, path_points.back());
  assignTrajectoryStationing(result.segments);
  result.stats.output_segments = result.segments.size();
  return result;
}

} // namespace drone_city_nav

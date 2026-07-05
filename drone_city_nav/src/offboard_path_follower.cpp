#include "drone_city_nav/offboard_path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

} // namespace

std::optional<OffboardPathProjection>
closestOffboardPathProjection(const std::span<const Point2> path,
                              const Point2 current_position,
                              const std::size_t minimum_segment_start_index) {
  if (path.empty() || !finite2D(current_position)) {
    return std::nullopt;
  }
  if (path.size() == 1U) {
    return OffboardPathProjection{
        0U, 0.0, squaredDistance(current_position, path.front()), path.front()};
  }

  OffboardPathProjection best{};
  const std::size_t first_segment_index =
      std::min(minimum_segment_start_index, path.size() - 2U);
  for (std::size_t i = first_segment_index; i + 1U < path.size(); ++i) {
    const Point2 segment_start = path[i];
    const Point2 segment_end = path[i + 1U];
    const Point2 segment{segment_end.x - segment_start.x,
                         segment_end.y - segment_start.y};
    const double segment_length_sq = squaredDistance(segment_start, segment_end);
    const double segment_t =
        segment_length_sq > kTinyDistanceM
            ? std::clamp(((current_position.x - segment_start.x) * segment.x +
                          (current_position.y - segment_start.y) * segment.y) /
                             segment_length_sq,
                         0.0, 1.0)
            : 0.0;
    const Point2 projected{segment_start.x + segment.x * segment_t,
                           segment_start.y + segment.y * segment_t};
    const double distance_sq = squaredDistance(current_position, projected);
    if (distance_sq < best.distance_sq) {
      best = OffboardPathProjection{i, segment_t, distance_sq, projected};
    }
  }

  return best;
}

std::size_t advanceWaypointIndex(const std::span<const Point2> path,
                                 const Point2 current_position,
                                 const std::size_t waypoint_index,
                                 const OffboardPathFollowerConfig& config) {
  if (path.empty() || !finite2D(current_position)) {
    return 0U;
  }

  std::size_t next_index = std::min(waypoint_index, path.size() - 1U);
  const std::size_t minimum_segment_index = next_index > 0U ? next_index - 1U : 0U;
  if (const auto projection =
          closestOffboardPathProjection(path, current_position, minimum_segment_index);
      projection.has_value()) {
    next_index = std::max(
        next_index, std::min(projection->segment_start_index + 1U, path.size() - 1U));
  }
  while (next_index + 1U < path.size()) {
    if (distance(current_position, path[next_index]) > config.acceptance_radius_m) {
      break;
    }
    ++next_index;
  }

  return next_index;
}

} // namespace drone_city_nav

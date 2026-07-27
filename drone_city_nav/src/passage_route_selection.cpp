#include "drone_city_nav/passage_route_selection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] double longitudinal(const Point2 point,
                                  const PassageOpening& opening) noexcept {
  return (point.x - opening.center.x) * opening.normal_xy.x +
         (point.y - opening.center.y) * opening.normal_xy.y;
}

[[nodiscard]] double lateral(const Point2 point,
                             const PassageOpening& opening) noexcept {
  return -(point.x - opening.center.x) * opening.normal_xy.y +
         (point.y - opening.center.y) * opening.normal_xy.x;
}

[[nodiscard]] std::size_t nearestGuideIndex(const mppi::State& state,
                                            const std::span<const Point2> guide) {
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < guide.size(); ++index) {
    const double candidate =
        std::hypot(guide[index].x - state.x, guide[index].y - state.y);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest_index = index;
    }
  }
  return nearest_index;
}

} // namespace

bool guideCrossesPassageAhead(const mppi::State& state,
                              const std::span<const Point2> guide,
                              const PassageOpening& opening,
                              const PassageRouteSelectionConfig& config) {
  if (guide.size() < 2U || !(config.activation_distance_m > 0.0) ||
      !(config.lateral_margin_m >= 0.0) || !(config.minimum_normal_alignment >= 0.0) ||
      !(config.minimum_normal_alignment <= 1.0)) {
    return false;
  }
  const Point2 position{state.x, state.y};
  if (distance(position, Point2{opening.center.x, opening.center.y}) >
      config.activation_distance_m) {
    return false;
  }
  const double usable_half_width = 0.5 * opening.width_m - config.lateral_margin_m;
  if (!(usable_half_width > 0.0)) {
    return false;
  }
  if (std::abs(longitudinal(position, opening)) <= 0.5 * opening.depth_m &&
      std::abs(lateral(position, opening)) <= usable_half_width) {
    return true;
  }

  const std::size_t nearest_index = nearestGuideIndex(state, guide);
  for (std::size_t index = nearest_index; index + 1U < guide.size(); ++index) {
    const Point2 first = guide[index];
    const Point2 second = guide[index + 1U];
    const double segment_x = second.x - first.x;
    const double segment_y = second.y - first.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length <= 1.0e-6) {
      continue;
    }
    const double normal_alignment =
        std::abs((segment_x * opening.normal_xy.x + segment_y * opening.normal_xy.y) /
                 segment_length);
    if (normal_alignment < config.minimum_normal_alignment) {
      continue;
    }
    const double first_longitudinal = longitudinal(first, opening);
    const double second_longitudinal = longitudinal(second, opening);
    if (first_longitudinal * second_longitudinal > 0.0 ||
        std::abs(first_longitudinal - second_longitudinal) <= 1.0e-9) {
      continue;
    }
    const double ratio = std::clamp(
        first_longitudinal / (first_longitudinal - second_longitudinal), 0.0, 1.0);
    const Point2 crossing{
        std::lerp(first.x, second.x, ratio),
        std::lerp(first.y, second.y, ratio),
    };
    if (std::abs(lateral(crossing, opening)) <= usable_half_width) {
      return true;
    }
  }
  return false;
}

} // namespace drone_city_nav

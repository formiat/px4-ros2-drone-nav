#include "drone_city_nav/raw_guide_validation.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {

RawGuideValidationResult validateGuideAgainstRawOccupancy(
    const std::span<const Point2> guide, const OccupancyGrid2D& occupancy,
    const double maximum_sample_step_m, const double start_station_m) {
  if (guide.size() < 2U || !(maximum_sample_step_m > 0.0) ||
      !std::isfinite(maximum_sample_step_m) || !(start_station_m >= 0.0) ||
      !std::isfinite(start_station_m)) {
    return {};
  }

  RawGuideValidationResult result{
      .accepted = true,
      .status = RawGuideValidationStatus::kAccepted,
  };
  double total_length_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    if (!std::isfinite(guide[index - 1U].x) || !std::isfinite(guide[index - 1U].y) ||
        !std::isfinite(guide[index].x) || !std::isfinite(guide[index].y)) {
      return {.status = RawGuideValidationStatus::kInvalidGuide,
              .failure_point = guide[index - 1U]};
    }
    total_length_m += distance(guide[index - 1U], guide[index]);
  }
  if (start_station_m > total_length_m + 1.0e-9) {
    return {.status = RawGuideValidationStatus::kInvalidGuide,
            .failure_point = guide.back()};
  }
  const double sample_step_m =
      std::min(maximum_sample_step_m, 0.5 * occupancy.resolution());
  double cumulative_station_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    Point2 from = guide[index - 1U];
    const Point2 to = guide[index];
    if (!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(to.x) ||
        !std::isfinite(to.y)) {
      return {.status = RawGuideValidationStatus::kInvalidGuide,
              .checked_samples = result.checked_samples,
              .failure_point = from};
    }
    const double segment_length_m = distance(from, to);
    if (cumulative_station_m + segment_length_m < start_station_m) {
      cumulative_station_m += segment_length_m;
      continue;
    }
    if (segment_length_m > 0.0 && start_station_m > cumulative_station_m) {
      const double ratio = std::clamp(
          (start_station_m - cumulative_station_m) / segment_length_m, 0.0, 1.0);
      from = Point2{std::lerp(from.x, to.x, ratio), std::lerp(from.y, to.y, ratio)};
    }
    cumulative_station_m += segment_length_m;
    const double remaining_segment_length_m = distance(from, to);
    const std::size_t sample_count =
        std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(
                                      remaining_segment_length_m / sample_step_m)));
    for (std::size_t sample_index = 0U; sample_index <= sample_count; ++sample_index) {
      const double ratio =
          static_cast<double>(sample_index) / static_cast<double>(sample_count);
      const Point2 point{std::lerp(from.x, to.x, ratio),
                         std::lerp(from.y, to.y, ratio)};
      ++result.checked_samples;
      const std::optional<GridIndex> cell = occupancy.worldToCell(point);
      if (!cell.has_value()) {
        return {.status = RawGuideValidationStatus::kOutsideGrid,
                .checked_samples = result.checked_samples,
                .failure_point = point};
      }
      if (occupancy.isOccupied(*cell)) {
        return {.status = RawGuideValidationStatus::kRawCollision,
                .checked_samples = result.checked_samples,
                .failure_point = point};
      }
    }
  }
  return result;
}

const char*
rawGuideValidationStatusName(const RawGuideValidationStatus status) noexcept {
  switch (status) {
    case RawGuideValidationStatus::kAccepted:
      return "accepted";
    case RawGuideValidationStatus::kInvalidGuide:
      return "invalid_guide";
    case RawGuideValidationStatus::kOutsideGrid:
      return "outside_grid";
    case RawGuideValidationStatus::kRawCollision:
      return "raw_collision";
  }
  return "unknown";
}

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav {

enum class RawGuideValidationStatus {
  kAccepted,
  kInvalidGuide,
  kOutsideGrid,
  kRawCollision,
};

struct RawGuideValidationResult {
  bool accepted{false};
  RawGuideValidationStatus status{RawGuideValidationStatus::kInvalidGuide};
  std::size_t checked_samples{0U};
  Point2 failure_point{};
};

[[nodiscard]] RawGuideValidationResult validateGuideAgainstRawOccupancy(
    std::span<const Point2> guide, const OccupancyGrid2D& occupancy,
    double maximum_sample_step_m, double start_station_m = 0.0);

[[nodiscard]] const char*
rawGuideValidationStatusName(RawGuideValidationStatus status) noexcept;

} // namespace drone_city_nav

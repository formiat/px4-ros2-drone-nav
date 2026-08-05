#include "drone_city_nav/tracking_objective.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Point3 interpolate(const Point3& first, const Point3& second,
                                 const double fraction) noexcept {
  return Point3{first.x + fraction * (second.x - first.x),
                first.y + fraction * (second.y - first.y),
                first.z + fraction * (second.z - first.z)};
}

using RawOccupiedQuery = std::function<bool(const Point3&)>;

[[nodiscard]] TrackingObjectiveResolution
resolve(const Point3& observed_position, const Point3& predicted_position,
        const double segment_length_m, const double grid_resolution_m,
        const double maximum_sample_spacing_m, const RawOccupiedQuery& raw_occupied) {
  if (!finite(observed_position) || !finite(predicted_position) ||
      !(grid_resolution_m > 0.0) || !(maximum_sample_spacing_m > 0.0)) {
    return {};
  }
  if (segment_length_m <= 1.0e-9) {
    return TrackingObjectiveResolution{
        .resolved_position = predicted_position,
        .status = TrackingObjectiveResolutionStatus::kUnchanged,
        .resolved_fraction = 1.0,
    };
  }

  const double sample_spacing_m =
      std::min(maximum_sample_spacing_m, grid_resolution_m * 0.5);
  const auto sample_count = static_cast<std::size_t>(
      std::max(1.0, std::ceil(segment_length_m / sample_spacing_m)));
  Point3 last_free = observed_position;
  double last_free_fraction = 0.0;
  for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
    const double fraction =
        static_cast<double>(sample) / static_cast<double>(sample_count);
    const Point3 point = interpolate(observed_position, predicted_position, fraction);
    if (raw_occupied(point)) {
      return TrackingObjectiveResolution{
          .resolved_position = last_free,
          .status = last_free_fraction > 0.0
                        ? TrackingObjectiveResolutionStatus::kClippedRawOccupied
                        : TrackingObjectiveResolutionStatus::kFallbackObserved,
          .resolved_fraction = last_free_fraction,
      };
    }
    last_free = point;
    last_free_fraction = fraction;
  }

  return TrackingObjectiveResolution{
      .resolved_position = predicted_position,
      .status = TrackingObjectiveResolutionStatus::kUnchanged,
      .resolved_fraction = 1.0,
  };
}

} // namespace

TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid2D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, const double maximum_sample_spacing_m) {
  const double segment_length_m =
      std::hypot(predicted_position.x - observed_position.x,
                 predicted_position.y - observed_position.y);
  return resolve(observed_position, predicted_position, segment_length_m,
                 raw_occupancy.resolution(), maximum_sample_spacing_m,
                 [&raw_occupancy](const Point3& point) {
                   const auto cell =
                       raw_occupancy.worldToCell(Point2{point.x, point.y});
                   return cell.has_value() && raw_occupancy.isOccupied(*cell);
                 });
}

TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid3D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, const double maximum_sample_spacing_m) {
  return resolve(observed_position, predicted_position,
                 distance3D(observed_position, predicted_position),
                 raw_occupancy.bounds().resolution_m, maximum_sample_spacing_m,
                 [&raw_occupancy](const Point3& point) {
                   const auto cell = raw_occupancy.worldToCell(point);
                   return cell.has_value() && raw_occupancy.isOccupied(*cell);
                 });
}

const char* trackingObjectiveResolutionStatusName(
    const TrackingObjectiveResolutionStatus status) noexcept {
  switch (status) {
    case TrackingObjectiveResolutionStatus::kUnchanged:
      return "unchanged";
    case TrackingObjectiveResolutionStatus::kClippedRawOccupied:
      return "clipped_raw_occupied";
    case TrackingObjectiveResolutionStatus::kFallbackObserved:
      return "fallback_observed";
    case TrackingObjectiveResolutionStatus::kWorldUnavailable:
      return "world_unavailable";
    case TrackingObjectiveResolutionStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav

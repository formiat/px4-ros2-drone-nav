#include "drone_city_nav/executed_horizon_clearance_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 position(const MotionState3D& state) noexcept {
  return Point3{static_cast<double>(state.x), static_cast<double>(state.y),
                static_cast<double>(state.z)};
}

[[nodiscard]] double segmentLength(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(second.x - first.x, second.y - first.y),
                    second.z - first.z);
}

// The physical body alone, for the clearance a constrained sample's body keeps.
[[nodiscard]] SweptFootprintConfig
bodyFootprint(const SweptFootprintConfig& footprint) {
  SweptFootprintConfig body = footprint;
  body.radius_m = footprint.body_radius_m;
  body.lower_extent_m = footprint.body_lower_extent_m;
  body.upper_extent_m = footprint.body_upper_extent_m;
  return body;
}

[[nodiscard]] double bodyClearanceM(const EsdfGrid3D& grid,
                                    const std::span<const float> esdf_m,
                                    const Point3& first, const Point3& second,
                                    const SweptFootprintConfig& body) {
  const DerivedFootprintClearance3D clearance =
      querySweptFootprintClearance3D(grid, esdf_m, first, second, body);
  return clearance.evidence.known_clearance_observed
             ? clearance.evidence.minimum_known_clearance_m
             : std::numeric_limits<double>::infinity();
}

} // namespace

ExecutedHorizonClearance3D measureExecutedHorizonClearance3D(
    const FiniteMotionHorizon3D& horizon, const std::size_t first_remaining_state_index,
    const EsdfGrid3D& grid, const std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint, const double constraint_clearance_m) {
  ExecutedHorizonClearance3D result;
  if (horizon.states.size() < 2U ||
      first_remaining_state_index + 1U >= horizon.states.size() || esdf_m.empty() ||
      !(constraint_clearance_m > 0.0)) {
    return result;
  }
  result.available = true;
  const SweptFootprintConfig body = bodyFootprint(footprint);
  double travelled_m{0.0};
  for (std::size_t index = first_remaining_state_index;
       index + 1U < horizon.states.size(); ++index) {
    const Point3 first = position(horizon.states[index]);
    const Point3 second = position(horizon.states[index + 1U]);
    const DerivedFootprintClearance3D clearance =
        querySweptFootprintClearance3D(grid, esdf_m, first, second, footprint);
    if (clearance.evidence.unknown_exposure && !result.unobserved_distance_m) {
      result.unobserved_distance_m = travelled_m;
    }
    if (clearance.evidence.known_clearance_observed) {
      const double clearance_m = clearance.evidence.minimum_known_clearance_m;
      result.minimum_clearance_m = std::min(result.minimum_clearance_m, clearance_m);
      if (clearance_m < constraint_clearance_m) {
        result.constrained_samples.push_back(ConstrainedHorizonSample3D{
            .distance_m = travelled_m,
            .clearance_m = clearance_m,
            .body_clearance_m = bodyClearanceM(grid, esdf_m, first, second, body)});
      }
    }
    travelled_m += segmentLength(first, second);
  }
  return result;
}

ExecutedHorizonClearance3D
measureRouteClearance3D(const std::span<const RouteSample3D> route,
                        const double from_station_m, const double lookahead_m,
                        const EsdfGrid3D& grid, const std::span<const float> esdf_m,
                        const SweptFootprintConfig& footprint,
                        const double constraint_clearance_m) {
  ExecutedHorizonClearance3D result;
  if (route.size() < 2U || !std::isfinite(from_station_m) ||
      !std::isfinite(lookahead_m) || !(lookahead_m > 0.0) || esdf_m.empty() ||
      !(constraint_clearance_m > 0.0)) {
    return result;
  }
  result.available = true;
  const SweptFootprintConfig body = bodyFootprint(footprint);
  const double end_station_m = from_station_m + lookahead_m;
  Point3 previous = sampleRoute3DAtStation(route, from_station_m).position;
  double previous_station_m = from_station_m;
  for (const RouteSample3D& sample : route) {
    if (sample.station_m <= from_station_m) {
      continue;
    }
    const DerivedFootprintClearance3D clearance = querySweptFootprintClearance3D(
        grid, esdf_m, previous, sample.position, footprint);
    if (clearance.evidence.known_clearance_observed) {
      const double clearance_m = clearance.evidence.minimum_known_clearance_m;
      result.minimum_clearance_m = std::min(result.minimum_clearance_m, clearance_m);
      if (clearance_m < constraint_clearance_m) {
        result.constrained_samples.push_back(ConstrainedHorizonSample3D{
            .distance_m = std::max(0.0, previous_station_m - from_station_m),
            .clearance_m = clearance_m,
            .body_clearance_m =
                bodyClearanceM(grid, esdf_m, previous, sample.position, body)});
      }
    }
    if (sample.station_m >= end_station_m) {
      break;
    }
    previous = sample.position;
    previous_station_m = sample.station_m;
  }
  return result;
}

std::optional<double>
measureRouteObservedRange3D(const std::span<const RouteSample3D> route,
                            const double from_station_m, const double lookahead_m,
                            const EsdfGrid3D& grid, const std::span<const float> esdf_m,
                            const SweptFootprintConfig& footprint) {
  if (route.size() < 2U || !std::isfinite(from_station_m) ||
      !std::isfinite(lookahead_m) || !(lookahead_m > 0.0) || esdf_m.empty()) {
    return std::nullopt;
  }
  const double end_station_m = from_station_m + lookahead_m;
  Point3 previous = sampleRoute3DAtStation(route, from_station_m).position;
  double previous_station_m = from_station_m;
  for (const RouteSample3D& sample : route) {
    if (sample.station_m <= from_station_m) {
      continue;
    }
    const DerivedFootprintClearance3D clearance = querySweptFootprintClearance3D(
        grid, esdf_m, previous, sample.position, footprint);
    if (clearance.evidence.unknown_exposure) {
      return std::max(0.0, previous_station_m - from_station_m);
    }
    if (sample.station_m >= end_station_m) {
      break;
    }
    previous = sample.position;
    previous_station_m = sample.station_m;
  }
  return std::nullopt;
}

double measureObservedRangeAlong3D(const ObservedOccupancyGrid3D& occupancy,
                                   const Point3& origin, const Vec3& direction,
                                   const double body_radius_m,
                                   const double maximum_range_m) {
  const double length = std::hypot(std::hypot(direction.x, direction.y), direction.z);
  const double step_m = 0.5 * occupancy.bounds().resolution_m;
  if (!(length > 1.0e-9) || !(step_m > 0.0) || !(body_radius_m >= 0.0) ||
      !std::isfinite(maximum_range_m)) {
    return 0.0;
  }
  const Vec3 along{direction.x / length, direction.y / length, direction.z / length};
  // Two axes across the motion: the body's radius is probed on both.
  const Vec3 reference =
      std::abs(along.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
  Vec3 across{along.y * reference.z - along.z * reference.y,
              along.z * reference.x - along.x * reference.z,
              along.x * reference.y - along.y * reference.x};
  const double across_length = std::hypot(std::hypot(across.x, across.y), across.z);
  across = Vec3{across.x / across_length, across.y / across_length,
                across.z / across_length};
  const Vec3 other{along.y * across.z - along.z * across.y,
                   along.z * across.x - along.x * across.z,
                   along.x * across.y - along.y * across.x};
  const std::array<Vec3, 5U> offsets{
      Vec3{},
      Vec3{across.x * body_radius_m, across.y * body_radius_m,
           across.z * body_radius_m},
      Vec3{-across.x * body_radius_m, -across.y * body_radius_m,
           -across.z * body_radius_m},
      Vec3{other.x * body_radius_m, other.y * body_radius_m, other.z * body_radius_m},
      Vec3{-other.x * body_radius_m, -other.y * body_radius_m,
           -other.z * body_radius_m},
  };
  double observed_m = body_radius_m;
  for (double range_m = body_radius_m + step_m; range_m <= maximum_range_m;
       range_m += step_m) {
    for (const Vec3& offset : offsets) {
      const std::optional<GridIndex3D> cell =
          occupancy.worldToCell(Point3{origin.x + along.x * range_m + offset.x,
                                       origin.y + along.y * range_m + offset.y,
                                       origin.z + along.z * range_m + offset.z});
      if (!cell.has_value() || !occupancy.isKnown(*cell)) {
        return observed_m;
      }
    }
    observed_m = range_m;
  }
  return observed_m;
}

} // namespace drone_city_nav

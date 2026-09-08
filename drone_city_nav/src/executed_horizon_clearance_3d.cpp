#include "drone_city_nav/executed_horizon_clearance_3d.hpp"

#include <algorithm>
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
            .distance_m = travelled_m, .clearance_m = clearance_m});
      }
    }
    travelled_m += segmentLength(first, second);
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

} // namespace drone_city_nav

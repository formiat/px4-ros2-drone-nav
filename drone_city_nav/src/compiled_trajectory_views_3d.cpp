#include "drone_city_nav/compiled_trajectory_views_3d.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

std::vector<Point2>
projectCompiledTrajectoryTo2D(const CompiledTrajectory3D& trajectory) {
  std::vector<Point2> result;
  if (trajectory.route == nullptr || trajectory.compiled_trajectory_revision == 0U ||
      compiledTrajectoryRevision3D(trajectory) !=
          trajectory.compiled_trajectory_revision) {
    return result;
  }
  result.reserve(trajectory.route->size());
  for (const RouteSample3D& sample : *trajectory.route) {
    result.push_back(Point2{sample.position.x, sample.position.y});
  }
  return result;
}

std::optional<double>
remainingCompiledTrajectoryTime3D(const CompiledTrajectory3D& trajectory,
                                  const double station_m) noexcept {
  constexpr double kStationToleranceM{1.0e-6};
  if (!std::isfinite(station_m) || trajectory.route == nullptr ||
      !compiledTrajectoryResourcesValid3D(trajectory) ||
      trajectory.compiled_trajectory_revision == 0U ||
      compiledTrajectoryRevision3D(trajectory) !=
          trajectory.compiled_trajectory_revision) {
    return std::nullopt;
  }
  const std::vector<RouteSample3D>& route = *trajectory.route;
  const CompiledTrajectoryTimeProfile3D& profile = trajectory.time_profile;
  const double begin_station_m = route.front().station_m;
  const double end_station_m = route.back().station_m;
  if (station_m < begin_station_m - kStationToleranceM ||
      station_m > end_station_m + kStationToleranceM) {
    return std::nullopt;
  }
  const double clamped_station_m =
      std::clamp(station_m, begin_station_m, end_station_m);
  if (clamped_station_m >= end_station_m - kStationToleranceM) {
    return 0.0;
  }

  const auto upper =
      std::upper_bound(route.begin(), route.end(), clamped_station_m,
                       [](const double value, const RouteSample3D& sample) {
                         return value < sample.station_m;
                       });
  if (upper == route.begin() || upper == route.end()) {
    return std::nullopt;
  }
  const std::size_t segment =
      static_cast<std::size_t>(std::distance(route.begin(), upper) - 1);
  const double segment_begin_station_m = route[segment].station_m;
  const double segment_end_station_m = route[segment + 1U].station_m;
  const double segment_length_m = segment_end_station_m - segment_begin_station_m;
  if (!(segment_length_m > 0.0)) {
    return std::nullopt;
  }

  double elapsed_time_s = profile.arrival_times_s[segment];
  if (clamped_station_m > segment_begin_station_m + kStationToleranceM) {
    const double fraction =
        (clamped_station_m - segment_begin_station_m) / segment_length_m;
    elapsed_time_s = profile.departure_times_s[segment] +
                     fraction * (profile.arrival_times_s[segment + 1U] -
                                 profile.departure_times_s[segment]);
  }
  if (!std::isfinite(elapsed_time_s) || elapsed_time_s < 0.0 ||
      elapsed_time_s > profile.travel_time_s + kStationToleranceM) {
    return std::nullopt;
  }
  return std::max(0.0, profile.travel_time_s - elapsed_time_s);
}

} // namespace drone_city_nav

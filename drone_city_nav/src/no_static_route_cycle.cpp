#include "drone_city_nav/no_static_route_cycle.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace drone_city_nav {

NoStaticRouteCycleDetector::NoStaticRouteCycleDetector(
    const NoStaticRouteCycleConfig& config)
    : config_{config} {
}

NoStaticRouteCycleResult
NoStaticRouteCycleDetector::observe(const NoStaticRouteCycleObservation& observation) {
  if (observation.guide_generation == 0U ||
      observation.guide_generation == last_generation_) {
    return {};
  }
  last_generation_ = observation.guide_generation;
  const std::int64_t window_ns =
      static_cast<std::int64_t>(std::max(0.0, config_.observation_window_s) * 1.0e9);
  while (!observations_.empty() &&
         observation.stamp_ns - observations_.front().stamp_ns > window_ns) {
    observations_.pop_front();
  }
  observations_.push_back(observation);
  if (observations_.size() < config_.minimum_generation_changes) {
    return {.generation_changes = observations_.size()};
  }

  const NoStaticRouteCycleObservation& oldest = observations_.front();
  const double vehicle_displacement =
      distance(oldest.vehicle_position, observation.vehicle_position);
  const double mission_progress =
      oldest.mission_distance_m - observation.mission_distance_m;
  const auto repeated = std::find_if(
      observations_.begin(), std::prev(observations_.end()),
      [&](const NoStaticRouteCycleObservation& previous) {
        return distance(previous.guide_endpoint, observation.guide_endpoint) <=
               config_.repeated_endpoint_radius_m;
      });
  const bool cooldown_elapsed =
      last_detection_stamp_ns_ == 0 ||
      observation.stamp_ns - last_detection_stamp_ns_ > window_ns / 2;
  if (repeated == std::prev(observations_.end()) ||
      vehicle_displacement > config_.maximum_vehicle_displacement_m ||
      mission_progress > config_.maximum_mission_progress_m || !cooldown_elapsed) {
    return {.generation_changes = observations_.size()};
  }
  last_detection_stamp_ns_ = observation.stamp_ns;
  return {.cycle_detected = true,
          .generation_changes = observations_.size(),
          .repeated_endpoint = observation.guide_endpoint,
          .approach_heading_rad = observation.approach_heading_rad};
}

void NoStaticRouteCycleDetector::reset() noexcept {
  observations_.clear();
  last_generation_ = 0U;
  last_detection_stamp_ns_ = 0;
}

std::vector<NoStaticDirectedTabuSample>
sampleNoStaticDirectedTabu(const std::span<const Point2> guide,
                           const double sample_spacing_m) {
  std::vector<NoStaticDirectedTabuSample> samples;
  const double spacing_m = std::max(0.1, sample_spacing_m);
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    const Point2& from = guide[index - 1U];
    const Point2& to = guide[index];
    const double segment_length_m = distance(from, to);
    if (!(segment_length_m > 1.0e-9)) {
      continue;
    }
    const std::size_t sample_count = static_cast<std::size_t>(
        std::max(1.0, std::ceil(segment_length_m / spacing_m)));
    const double heading_rad = std::atan2(to.y - from.y, to.x - from.x);
    for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(sample_count);
      samples.push_back(NoStaticDirectedTabuSample{
          .point =
              Point2{std::lerp(from.x, to.x, ratio), std::lerp(from.y, to.y, ratio)},
          .approach_heading_rad = heading_rad,
      });
    }
  }
  return samples;
}

} // namespace drone_city_nav

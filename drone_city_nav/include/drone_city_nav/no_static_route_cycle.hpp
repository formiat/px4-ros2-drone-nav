#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace drone_city_nav {

struct NoStaticRouteCycleConfig {
  double observation_window_s{20.0};
  std::size_t minimum_generation_changes{6U};
  double repeated_endpoint_radius_m{6.0};
  double maximum_vehicle_displacement_m{12.0};
  double maximum_mission_progress_m{4.0};
};

struct NoStaticRouteCycleObservation {
  std::uint64_t guide_generation{0U};
  std::int64_t stamp_ns{0};
  Point2 vehicle_position{};
  Point2 guide_endpoint{};
  double approach_heading_rad{0.0};
  double mission_distance_m{0.0};
};

struct NoStaticRouteCycleResult {
  bool cycle_detected{false};
  std::size_t generation_changes{0U};
  Point2 repeated_endpoint{};
  double approach_heading_rad{0.0};
};

struct NoStaticDirectedTabuSample {
  Point2 point{};
  double approach_heading_rad{0.0};
};

class NoStaticRouteCycleDetector {
public:
  explicit NoStaticRouteCycleDetector(const NoStaticRouteCycleConfig& config);

  [[nodiscard]] NoStaticRouteCycleResult
  observe(const NoStaticRouteCycleObservation& observation);
  void reset() noexcept;

private:
  NoStaticRouteCycleConfig config_{};
  std::deque<NoStaticRouteCycleObservation> observations_;
  std::uint64_t last_generation_{0U};
  std::int64_t last_detection_stamp_ns_{0};
};

[[nodiscard]] std::vector<NoStaticDirectedTabuSample>
sampleNoStaticDirectedTabu(std::span<const Point2> guide, double sample_spacing_m);

} // namespace drone_city_nav

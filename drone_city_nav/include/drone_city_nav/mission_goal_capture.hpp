#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

struct MissionGoalCaptureConfig {
  double capture_radius_m{2.0};
};

struct MissionGoalCaptureObservation {
  Point3 mission_goal{};
  mppi::State state{};
  bool terminal_route_available{false};
};

struct MissionGoalCaptureResult {
  bool latched{false};
  bool newly_latched{false};
  double horizontal_distance_m{0.0};
};

class MissionGoalCaptureLatch final {
public:
  explicit MissionGoalCaptureLatch(const MissionGoalCaptureConfig& config = {});

  [[nodiscard]] MissionGoalCaptureResult
  update(const MissionGoalCaptureObservation& observation);

private:
  MissionGoalCaptureConfig config_{};
  Point3 mission_goal_{};
  bool mission_initialized_{false};
  bool latched_{false};
};

} // namespace drone_city_nav

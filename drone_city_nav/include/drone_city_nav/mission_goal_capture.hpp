#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

// The latch fires when the vehicle rests inside the capture radius: the
// capture radius is the mission's acceptance of the goal, and the goal hold
// then pins the vehicle where it came to rest rather than at the exact goal
// coordinate. Latching while still moving would hand the goal to a stationary
// hold that cannot be certified, with no controller left to stop the vehicle.
struct MissionGoalCaptureConfig {
  double capture_radius_m{2.0};
  double stationary_speed_tolerance_mps{kStationaryExecutionHoldSpeedToleranceMps};
};

struct MissionGoalCaptureObservation {
  Point3 mission_goal{};
  mppi::State state{};
  bool terminal_route_available{false};
};

struct MissionGoalCaptureResult {
  bool latched{false};
  bool newly_latched{false};
  double distance_m{0.0};
  double speed_mps{0.0};
};

class MissionGoalCaptureLatch final {
public:
  explicit MissionGoalCaptureLatch(const MissionGoalCaptureConfig& config = {});

  [[nodiscard]] MissionGoalCaptureResult
  update(const MissionGoalCaptureObservation& observation);
  [[nodiscard]] bool latchedFor(const Point3& mission_goal) const noexcept;

private:
  MissionGoalCaptureConfig config_{};
  Point3 mission_goal_{};
  bool mission_initialized_{false};
  bool latched_{false};
};

} // namespace drone_city_nav

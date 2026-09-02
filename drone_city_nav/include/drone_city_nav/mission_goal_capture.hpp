#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

// The latch fires when the vehicle can be held at the goal: it is inside the
// capture radius and, more tightly, inside the stationary hold tolerance at a
// stationary speed. Latching earlier would hand the goal to a stationary hold
// that cannot be certified until the vehicle actually arrives, with no
// controller left to bring it there.
struct MissionGoalCaptureConfig {
  double capture_radius_m{2.0};
  double stationary_position_tolerance_m{kStationaryExecutionHoldPositionToleranceM};
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

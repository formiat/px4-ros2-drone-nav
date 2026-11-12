#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

// The latch fires when the vehicle rests inside the capture radius at the end
// of its approach: the capture radius is the mission's acceptance of the
// goal, and the goal hold then pins the vehicle where it came to rest rather
// than at the exact goal coordinate. The approach has ended where the goal
// limiter brings the vehicle to rest, near the goal, or where it rests
// without getting closer: a vehicle the clearance laws slow to a crawl
// inside the radius is still approaching, and was latched 1.9 m short with
// the estimate 2.8 m off, 4.3 m from the goal in truth (r772). Latching while still
// moving would hand the goal to a stationary hold that cannot be certified, with no
// controller left to stop the vehicle. The latch releases again when the vehicle leaves
// the capture radius: the capture itself is only acknowledged while the vehicle stays
// inside it, and a latched planner flies no route, so a vehicle that drifted out would
// otherwise stay out with nothing left to bring it back.
struct MissionGoalCaptureConfig {
  double capture_radius_m{2.0};
  double stationary_speed_tolerance_mps{kStationaryExecutionHoldSpeedToleranceMps};
};

struct MissionGoalCaptureObservation {
  Point3 mission_goal{};
  mppi::State state{};
  bool terminal_route_available{false};
  std::int64_t stamp_ns{0};
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
  // The closest the vehicle has rested to the goal inside the radius, and
  // when it got there; unset while it moves or is outside.
  std::optional<double> resting_closest_m_;
  std::int64_t resting_closest_stamp_ns_{0};
};

} // namespace drone_city_nav

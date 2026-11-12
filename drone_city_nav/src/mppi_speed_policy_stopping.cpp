#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace drone_city_nav {

double stoppingLimitedSpeed(const double available_distance_m,
                            const double terminal_speed_mps,
                            const StoppingCapability& capability,
                            const SensorBrakingContract3D& contract,
                            const double forward_acceleration_mps2) noexcept {
  const double braking_acceleration_mps2 =
      capability.maximum_commanded_horizontal_deceleration_mps2;
  const double reaction_latency_s = capability.reaction_latency_s;
  if (!(available_distance_m > 0.0) || !(braking_acceleration_mps2 > 0.0) ||
      !(terminal_speed_mps >= 0.0) || !(reaction_latency_s >= 0.0)) {
    return 0.0;
  }
  // The law answers for the vehicle as it flies, not for one that already
  // decelerates: the jerk ramp from its forward acceleration to the full
  // deceleration is flown before the speed falls at all. A law of
  // instantaneous deceleration fell along its own curve at 4 m/s^2 once the
  // vehicle rode it, and the vehicle under it was accelerating: in the urban
  // flight r440 the clearance limit read 7.9, 7.0, 6.6, 6.1, 5.1, 4.6 and
  // 3.8 m/s over its last 1.3 s while the vehicle climbed from 3.8 to 5.6 m/s
  // at 3.7 m/s^2 beneath it, crossed it 1.05 s before the wall, and needed
  // 0.64 s and 3.5 m to turn that acceleration round. The flights r430 to
  // r452 carry fifteen to twenty-three such crossings each, the vehicle 2 to
  // 4.7 m/s above the reference, and three to twelve revocations at speed.
  // The sensor-braking contract reserves the ramp from the largest
  // acceleration, because it bounds a speed held for the whole flight; here
  // the largest acceleration would deny a vehicle creeping to its goal any
  // speed inside the last 0.6 m, so the measured one is charged.
  const JerkLimitedAxisStoppingConfig slowdown{
      .guaranteed_deceleration_mps2 = braking_acceleration_mps2,
      .maximum_acceleration_mps2 = contract.maximum_horizontal_acceleration_mps2,
      .maximum_jerk_mps3 = contract.maximum_control_jerk_mps3,
      .reaction_latency_s = reaction_latency_s,
  };
  // The instantaneous law bounds the answer from above.
  const double latency_velocity = braking_acceleration_mps2 * reaction_latency_s;
  double lower_mps = terminal_speed_mps;
  double upper_mps = std::sqrt(latency_velocity * latency_velocity +
                               terminal_speed_mps * terminal_speed_mps +
                               2.0 * braking_acceleration_mps2 * available_distance_m) -
                     latency_velocity;
  if (!(upper_mps > lower_mps)) {
    return std::max(0.0, upper_mps);
  }
  constexpr int kBisectionIterations{40};
  for (int iteration = 0; iteration < kBisectionIterations; ++iteration) {
    const double candidate_mps = std::midpoint(lower_mps, upper_mps);
    if (jerkLimitedAxisSlowdownDistanceM(candidate_mps, terminal_speed_mps,
                                         forward_acceleration_mps2,
                                         slowdown) <= available_distance_m) {
      lower_mps = candidate_mps;
    } else {
      upper_mps = candidate_mps;
    }
  }
  return lower_mps;
}

} // namespace drone_city_nav

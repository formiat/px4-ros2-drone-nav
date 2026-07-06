#include <cmath>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {

double Px4OffboardNode::finalTrajectoryGoalAltitudeM() const {
  if (trajectorySamplesAreUsable(final_trajectory_samples_) &&
      std::isfinite(final_trajectory_samples_.back().z_m)) {
    return final_trajectory_samples_.back().z_m;
  }
  return cruise_altitude_m_;
}

VerticalSetpointPlan Px4OffboardNode::planVerticalSetpointForCurrentTrajectory(
    const VelocitySetpointPlan& velocity_plan, const double dt_s) const {
  const double scalar_speed_mps = std::isfinite(velocity_plan.accel_limited_speed_mps)
                                      ? velocity_plan.accel_limited_speed_mps
                                      : velocity_plan.final_command_speed_mps;
  return planVerticalSetpoint(final_trajectory_samples_, velocity_plan.trajectory_s_m,
                              scalar_speed_mps, current_altitude_m_, altitude_valid_,
                              dt_s, vertical_follower_state_,
                              vertical_follower_config_);
}

} // namespace drone_city_nav

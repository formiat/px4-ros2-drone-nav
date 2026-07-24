#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::schedulePlanningCycle(const PlanningWakeReason reason) {
  {
    const std::scoped_lock lock{planning_request_mutex_};
    planning_request_state_.schedule(reason);
  }
  planning_request_cv_.notify_one();
}

void PlannerNode::invalidateAndSchedulePlanningCycle(
    const PlanningInvalidationReason reason) {
  {
    const std::scoped_lock lock{planning_request_mutex_};
    planning_request_state_.invalidate(reason);
  }
  planning_request_cv_.notify_one();
}

std::uint64_t PlannerNode::latestPlanningInvalidationGeneration() const {
  const std::scoped_lock lock{planning_request_mutex_};
  return planning_request_state_.latestInvalidationGeneration();
}

PlanningInvalidationReason PlannerNode::latestPlanningInvalidationReason() const {
  const std::scoped_lock lock{planning_request_mutex_};
  return planning_request_state_.latestInvalidationReason();
}

void PlannerNode::planningWorkerLoop(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    PlanningJobIdentity identity{};
    {
      std::unique_lock lock{planning_request_mutex_};
      planning_request_cv_.wait(lock, stop_token,
                                [this]() { return planning_request_state_.pending(); });
      if (stop_token.stop_requested()) {
        return;
      }
      identity = planning_request_state_.beginCycle();
    }

    runPlanningCycle(identity);
    {
      const std::scoped_lock lock{planning_request_mutex_};
      planning_request_state_.finishCycle();
    }
  }
}

PlannerNode::NavigationStateSnapshot PlannerNode::navigationStateSnapshot() const {
  const std::scoped_lock lock{navigation_state_mutex_};
  return live_navigation_state_;
}

void PlannerNode::applyNavigationStateSnapshot(
    const NavigationStateSnapshot& snapshot) {
  current_pose_ = snapshot.pose;
  current_velocity_ = snapshot.velocity;
  current_attitude_ = snapshot.attitude;
  current_altitude_m_ = snapshot.altitude_m;
  current_speed_mps_ = snapshot.speed_mps;
  last_pose_update_ns_ = snapshot.stamp_ns;
  pose_valid_ = snapshot.pose_valid;
  altitude_valid_ = snapshot.altitude_valid;
  current_velocity_valid_ = snapshot.velocity_valid;
  attitude_valid_ = snapshot.attitude_valid;
}

void PlannerNode::applyLatestLidarInputSnapshot() {
  const std::scoped_lock lock{lidar_input_mutex_};
  last_scan_ = live_lidar_input_.scan;
  last_scan_projection_pose_ = live_lidar_input_.projection_pose;
  last_scan_projection_poses_ = live_lidar_input_.beam_projection_poses;
  last_scan_projection_pose_source_ = live_lidar_input_.projection_pose_source;
  last_scan_motion_shift_ = live_lidar_input_.motion_shift;
  last_scan_pose_lag_s_ = live_lidar_input_.pose_lag_s;
  last_scan_pose_latency_s_ = live_lidar_input_.pose_latency_s;
  last_scan_motion_shift_m_ = live_lidar_input_.motion_shift_m;
  last_scan_update_ns_ = live_lidar_input_.update_ns;
  scan_seen_ = live_lidar_input_.seen;
  last_scan_projection_pose_valid_ = live_lidar_input_.projection_pose_valid;
}

} // namespace drone_city_nav

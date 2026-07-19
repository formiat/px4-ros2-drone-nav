#include <algorithm>
#include <cmath>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

constexpr double kMinimumPredictionHorizonS = 0.20;
constexpr double kMaximumPredictionHorizonS = 2.50;

[[nodiscard]] Point2
interpolateTrajectoryPoint(const std::span<const TrajectoryPointSample> samples,
                           const double station_m) {
  if (!trajectorySamplesAreUsable(samples)) {
    return {};
  }
  const double bounded_station_m = std::clamp(station_m, 0.0, samples.back().s_m);
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const TrajectoryPointSample& start = samples[index];
    const TrajectoryPointSample& end = samples[index + 1U];
    if (bounded_station_m > end.s_m && index + 2U < samples.size()) {
      continue;
    }
    const double length_m = end.s_m - start.s_m;
    const double ratio =
        length_m > 1.0e-6
            ? std::clamp((bounded_station_m - start.s_m) / length_m, 0.0, 1.0)
            : 0.0;
    return Point2{start.point.x * (1.0 - ratio) + end.point.x * ratio,
                  start.point.y * (1.0 - ratio) + end.point.y * ratio};
  }
  return samples.back().point;
}

} // namespace

void PlannerNode::requestPlanningCycle() {
  {
    const std::scoped_lock lock{planning_request_mutex_};
    ++latest_planning_request_generation_;
    planning_request_pending_ = true;
  }
  planning_request_cv_.notify_one();
}

void PlannerNode::planningWorkerLoop(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::uint64_t generation = 0U;
    {
      std::unique_lock lock{planning_request_mutex_};
      planning_request_cv_.wait(lock, stop_token,
                                [this]() { return planning_request_pending_; });
      if (stop_token.stop_requested()) {
        return;
      }
      generation = latest_planning_request_generation_;
      planning_request_pending_ = false;
    }

    runPlanningCycle(generation);
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

Point2 PlannerNode::predictedPlanningStart(const NavigationStateSnapshot& navigation,
                                           const double horizon_s) const {
  if (!navigation.pose_valid || !finite2D(navigation.pose.position)) {
    return navigation.pose.position;
  }
  const double bounded_horizon_s =
      std::clamp(horizon_s, kMinimumPredictionHorizonS, kMaximumPredictionHorizonS);
  if (trajectorySamplesAreUsable(last_valid_trajectory_samples_)) {
    const std::optional<TrajectoryProjection> projection = projectOnTrajectorySamples(
        last_valid_trajectory_samples_, navigation.pose.position);
    if (projection.has_value()) {
      const double speed_mps =
          navigation.velocity_valid && std::isfinite(navigation.speed_mps)
              ? std::max(0.0, navigation.speed_mps)
              : 0.0;
      return interpolateTrajectoryPoint(last_valid_trajectory_samples_,
                                        projection->s_m +
                                            speed_mps * bounded_horizon_s);
    }
  }
  if (navigation.velocity_valid) {
    return Point2{
        navigation.pose.position.x + navigation.velocity.x * bounded_horizon_s,
        navigation.pose.position.y + navigation.velocity.y * bounded_horizon_s};
  }
  return navigation.pose.position;
}

} // namespace drone_city_nav

#include "mission_monitor_node.hpp"
#include "mission_monitor_node_internal.hpp"

namespace drone_city_nav {

using namespace mission_monitor_node_internal;

// The return home (roadmap item 19): the memory the reachability proof reads, the
// proof's verdict, the home estimate, the goal given up for the start and the objective
// it publishes, the route's remaining length (moved whole on 2026-11-02, the source
// size contract).

void MissionMonitorNode::onMemorySnapshot(const msg::RawObstacleSnapshot3D& snapshot) {
  const GridBounds3D bounds{
      .origin_x = snapshot.origin_x_m,
      .origin_y = snapshot.origin_y_m,
      .origin_z = snapshot.origin_z_m,
      .resolution_m = snapshot.resolution_m,
      .width_cells = static_cast<int>(snapshot.width_cells),
      .height_cells = static_cast<int>(snapshot.height_cells),
      .depth_cells = static_cast<int>(snapshot.depth_cells),
  };
  if (!std::isfinite(bounds.resolution_m) || bounds.resolution_m <= 0.0 ||
      bounds.width_cells <= 0 || bounds.height_cells <= 0 || bounds.depth_cells <= 0 ||
      static_cast<int>(snapshot.chunk_size_cells) !=
          ObservedOccupancyGrid3D::kChunkSize) {
    return;
  }
  auto grid = std::make_shared<ObservedOccupancyGrid3D>(bounds);
  applyMemoryChunks(*grid, snapshot.chunks);
  memory_ = std::move(grid);
  memory_producer_instance_id_ = snapshot.producer_instance_id;
  memory_base_revision_ = snapshot.obstacle_snapshot_revision;
}

void MissionMonitorNode::onMemoryDelta(const msg::RawObstacleDelta3D& delta) {
  if (memory_ == nullptr ||
      delta.producer_instance_id != memory_producer_instance_id_ ||
      delta.base_snapshot_revision != memory_base_revision_) {
    return;
  }
  // The proof reads a grid of its own on another thread: a delta writes a
  // fresh grid object, whose untouched chunks share the old storage.
  auto grid = std::make_shared<ObservedOccupancyGrid3D>(*memory_);
  applyMemoryChunks(*grid, delta.chunks);
  memory_ = std::move(grid);
}

void MissionMonitorNode::judgeReturnHome() {
  if (result_reported_ || !navigation_mission_ready_ ||
      active_waypoint_index_ >= waypoints_.size()) {
    return;
  }
  const std::int64_t now_ns = now().nanoseconds();
  if (proof_future_.valid() &&
      proof_future_.wait_for(std::chrono::seconds{0}) == std::future_status::ready) {
    const GoalReachabilityProof3D proof = proof_future_.get();
    const Point3 goal = waypoints_[active_waypoint_index_];
    // A proof started for the goal the return gave up says nothing of the
    // start.
    if (proof_goal_.x == goal.x && proof_goal_.y == goal.y && proof_goal_.z == goal.z) {
      latest_proof_ = proof;
      if (!latest_proof_.provenUnreachable()) {
        proven_since_ns_ = 0;
      } else if (proven_since_ns_ == 0) {
        proven_since_ns_ = now_ns;
      }
      const bool proof_held =
          proven_since_ns_ > 0 &&
          static_cast<double>(now_ns - proven_since_ns_) * 1.0e-9 >= proof_hold_s_;
      RCLCPP_INFO(get_logger(),
                  "GOAL_REACHABILITY_PROOF verdict=%s component_voxels=%zu "
                  "goal_inside=%s budget_exhausted=%s goal_region_voxels=%zu "
                  "held=%s returning=%s",
                  goalReachabilityProofVerdict(latest_proof_),
                  latest_proof_.component_voxels,
                  latest_proof_.goal_inside ? "true" : "false",
                  latest_proof_.budget_exhausted ? "true" : "false",
                  latest_proof_.goal_region_voxels, proof_held ? "true" : "false",
                  goal_substituted_ ? "true" : "false");
      // The survey's goals are the survey's business (roadmap item 34): one
      // proven unreachable is its next frontier's turn, not the way home.
      if (!goal_substituted_ && proof_held && !survey_ &&
          substituteGoalWithStart("topological", now_ns)) {
        proven_since_ns_ = 0;
        return;
      }
      if (goal_substituted_ && proof_held != start_unreachable_) {
        start_unreachable_ = proof_held;
        RCLCPP_WARN(
            get_logger(),
            "%s position=(%.3f,%.3f,%.3f) start=(%.3f,%.3f,%.3f) "
            "component_voxels=%zu",
            start_unreachable_ ? "START_UNREACHABLE_HELD" : "START_REACHABLE_AGAIN",
            latest_map_position_.x, latest_map_position_.y, latest_map_position_.z,
            goal.x, goal.y, goal.z, latest_proof_.component_voxels);
      }
    }
  }
  if (!proof_future_.valid() && memory_ != nullptr && latest_position_valid_) {
    const std::shared_ptr<const ObservedOccupancyGrid3D> grid = memory_;
    const Point3 vehicle = latest_map_position_;
    const Point3 goal = waypoints_[active_waypoint_index_];
    const FlightEnvelopeConfig envelope = flight_envelope_;
    const double capture_radius_m = goal_capture_radius_m_;
    const std::size_t budget = proof_voxel_budget_;
    proof_goal_ = goal;
    proof_future_ = std::async(std::launch::async, [grid, vehicle, goal, envelope,
                                                    capture_radius_m, budget] {
      return proveGoalUnreachable3D(*grid, vehicle, goal, envelope, capture_radius_m,
                                    SweptFootprintConfig{}.body_radius_m, budget);
    });
  }
}

[[nodiscard]] double MissionMonitorNode::homeEstimateS() const noexcept {
  const double mean = meanSpeed();
  return kReturnEstimateMargin * flown_path_m_ /
             std::max(std::isfinite(mean) ? mean : 0.0, 0.5) +
         kReturnReserveS;
}

void MissionMonitorNode::onNavigationProgress(const msg::NavigationProgress& progress) {
  if (result_reported_ || goal_substituted_ || !navigation_mission_ready_ ||
      progress.header.frame_id != frame_id_ ||
      active_waypoint_index_ >= waypoints_.size()) {
    return;
  }
  const std::int64_t now_ns = now().nanoseconds();
  // The share of its configured range the frame alone grants (K12), not
  // the measured range over the guaranteed: the range has a floor at the
  // margin, 2.001 m, which is 0.385 of the 5.2 m of K7, over the blind
  // line; by the ratio no frame was blind from 2026-10-14 (r1533, r1536).
  light_judgment_.observe(
      static_cast<double>(now_ns) * 1.0e-9,
      progress.sensor_guaranteed_range_m > 0.0 ? progress.sensor_frame_share : -1.0,
      light_reading_, unlit_surface_share_);
  if (std::isfinite(progress.route_remaining_m)) {
    route_remaining_m_ = progress.route_remaining_m;
  }
  if (light_judgment_.unreliable()) {
    static_cast<void>(substituteGoalWithStart("unreliable_light", now_ns));
    return;
  }
  // The way to B is not known until it is flown, the way home is: the
  // vehicle flies on while its charge covers the way home with a margin,
  // and turns when it no longer does. A flight with no battery (no charge
  // published) never turns for it.
  if (std::isfinite(light_charge_s_) &&
      light_charge_s_ < kReturnChargeMargin * homeEstimateS()) {
    static_cast<void>(substituteGoalWithStart("battery", now_ns));
  }
  // The vehicle's own battery by the same rule (specification K32).
  if (main_battery_.timeLeftS() < kReturnChargeMargin * homeEstimateS()) {
    static_cast<void>(substituteGoalWithStart("main_battery", now_ns));
  }
}

[[nodiscard]] std::uint64_t MissionMonitorNode::nextObjectiveEpoch() noexcept {
  last_objective_epoch_ =
      std::max(navigation_mission_epoch_, last_objective_epoch_) + 1U;
  return last_objective_epoch_;
}

[[nodiscard]] bool
MissionMonitorNode::substituteGoalWithStart(const char* trigger,
                                            const std::int64_t now_ns) {
  if (survey_) {
    // The survey's return is a new one-waypoint mission after the goals it
    // reached: the admission of acknowledgements starts over (its progress
    // rule took the start's acknowledgements for stale after a terminal one
    // accepted before, r2805: 352 rejected, no result, no landing).
    waypoint_acknowledgement_admission_ =
        MissionWaypointAcknowledgementAdmissionState{};
  }
  if (!returnHomePositionSourceFresh(latest_position_valid_, latest_position_stamp_ns_,
                                     now_ns)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "GOAL_UNREACHABLE_HELD trigger=%s reason=no_position_source",
                         trigger);
    return false;
  }
  const Point3 original_goal = waypoints_[active_waypoint_index_];
  const Point3 start{start_.x, start_.y,
                     std::isfinite(mission_ready_altitude_z_m_)
                         ? mission_ready_altitude_z_m_
                         : waypoints_.front().z};
  waypoints_ = {start};
  goal_ = start_;
  active_waypoint_index_ = 0U;
  completed_waypoint_count_ = 0U;
  minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
  goal_substituted_ = true;
  publishMissionEvent(std::string{"GOAL_UNREACHABLE trigger="} + trigger);
  // The return is one mission epoch (K20), opened here; its objective then
  // moves along the trail a sample at a time.
  return_epoch_ = nextObjectiveEpoch();
  return_sample_ = 0U;
  return_target_ = std::numeric_limits<std::size_t>::max();
  publishReturnObjective();
  const double elapsed_s =
      mission_start_ns_ > 0 ? static_cast<double>(now_ns - mission_start_ns_) * 1.0e-9
                            : 0.0;
  RCLCPP_WARN(get_logger(),
              "GOAL_UNREACHABLE trigger=%s goal=(%.3f,%.3f,%.3f) "
              "substituted_goal=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64
              " elapsed_s=%.1f flown_path_m=%.1f home_estimate_s=%.1f "
              "light_charge_s=%.1f route_remaining_m=%.1f "
              "light_outage_s=%.1f light_outage_share=%.3f proof=%s "
              "component_voxels=%zu",
              trigger, original_goal.x, original_goal.y, original_goal.z, start.x,
              start.y, start.z, return_epoch_, elapsed_s, flown_path_m_,
              homeEstimateS(), light_charge_s_, route_remaining_m_,
              light_judgment_.currentOutageS(), light_judgment_.outageShare(),
              goalReachabilityProofVerdict(latest_proof_),
              latest_proof_.component_voxels);
  return true;
}

void MissionMonitorNode::publishReturnObjective() {
  const std::size_t target = return_trail_.next(latest_map_position_);
  if (target == return_target_) {
    return;
  }
  return_target_ = target;
  const Point3 position =
      target == 0U ? waypoints_.front() : return_trail_.points()[target];
  // Each point is a sample of the return's one objective (K20): the planner
  // routes to the moved target afresh and its lifecycle replaces the route
  // to the old one (r908 stood 540 s at the first point with the objective
  // 20 m on, when it kept that route as equivalent).
  publishObjective(position, return_epoch_, ++return_sample_, false);
  RCLCPP_INFO(get_logger(),
              "RETURN_TRAIL objective=(%.3f,%.3f,%.3f) index=%zu of %zu "
              "mission_epoch=%" PRIu64 " sample=%" PRIu64,
              position.x, position.y, position.z, target, return_trail_.points().size(),
              return_epoch_, return_sample_);
}

} // namespace drone_city_nav

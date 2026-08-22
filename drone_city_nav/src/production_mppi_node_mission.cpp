#include <cmath>
#include <memory>
#include <optional>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

MissionWaypointUpdate ProductionMppiNode::updateMissionWaypoint(
    const std::shared_ptr<const ProductionNavigationObjective>& objective,
    const ProductionMppiNavigation& navigation,
    const MissionGoalCaptureResult& goal_capture, const std::int64_t now_ns) {
  if (!mission_waypoint_sequence_ || !objective || objective->tracking.has_value() ||
      objective->immediate_hold) {
    return {};
  }
  const MissionWaypointUpdate update =
      mission_waypoint_sequence_->update(MissionWaypointObservation{
          .stamp_ns = now_ns,
          .goal_captured = goal_capture.latched,
          .horizontal_speed_mps = std::hypot(static_cast<double>(navigation.state.vx),
                                             static_cast<double>(navigation.state.vy)),
      });
  if (!update.advanced) {
    return update;
  }

  mission_goal_ = mission_waypoint_sequence_->activeGoal();
  navigation_objective_.store(std::make_shared<const ProductionNavigationObjective>(
                                  ProductionNavigationObjective{
                                      .goal = mission_goal_,
                                      .tracking = std::nullopt,
                                      .mission_epoch = objective->mission_epoch + 1U,
                                      .sample_sequence = 0U,
                                      .assignment_generation = 0U,
                                      .target_detection_id = 0U,
                                      .target_track_id = 0U,
                                      .stamp_ns = now_ns,
                                      .continuous_tracking = false,
                                      .immediate_hold = false,
                                  }),
                              std::memory_order_release);
  {
    const std::scoped_lock lock{objective_replan_mutex_};
    objective_replan_anchor_ = mission_goal_;
    objective_replan_stamp_ns_ = now_ns;
  }
  if (topological_navigation_3d_) {
    topological_navigation_3d_->beginMissionLeg();
  }
  requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
  RCLCPP_INFO(get_logger(),
              "MISSION_WAYPOINT_ADVANCED completed_index=%zu waypoint_count=%zu "
              "next_goal=(%.2f,%.2f,%.2f)",
              update.completed_index, mission_waypoint_sequence_->waypointCount(),
              mission_goal_.x, mission_goal_.y, mission_goal_.z);
  return update;
}

} // namespace drone_city_nav

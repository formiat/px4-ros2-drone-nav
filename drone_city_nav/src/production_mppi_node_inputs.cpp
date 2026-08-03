#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <numbers>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double pointDistance(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y),
                    first.z - second.z);
}

} // namespace

void ProductionMppiNode::onLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition& message) {
  ProductionMppiNavigation navigation;
  navigation.receive_stamp_ns = get_clock()->now().nanoseconds();
  navigation.valid = message.xy_valid && message.z_valid && message.v_xy_valid &&
                     std::isfinite(message.x) && std::isfinite(message.y) &&
                     std::isfinite(message.z) && std::isfinite(message.vx) &&
                     std::isfinite(message.vy) && std::isfinite(message.vz);
  if (navigation.valid) {
    navigation.state.x = static_cast<float>(message.x + px4_local_origin_.x);
    navigation.state.y = static_cast<float>(message.y + px4_local_origin_.y);
    navigation.state.z = -message.z;
    navigation.state.vx = message.vx;
    navigation.state.vy = message.vy;
    navigation.state.vz = -message.vz;
    if (message.heading_good_for_control && std::isfinite(message.heading)) {
      navigation.state.yaw = message.heading;
    }
  }
  {
    const std::scoped_lock lock{input_mutex_};
    navigation.revision = navigation_.revision + 1U;
    navigation_ = navigation;
    latest_prediction_error_ = {};
    if (previous_predicted_next_state_.has_value() &&
        previous_prediction_stamp_ns_ > 0 && navigation.valid) {
      const mppi::State& predicted = *previous_predicted_next_state_;
      latest_prediction_error_.position_m =
          std::hypot(std::hypot(static_cast<double>(predicted.x - navigation.state.x),
                                static_cast<double>(predicted.y - navigation.state.y)),
                     static_cast<double>(predicted.z - navigation.state.z));
      latest_prediction_error_.velocity_mps = std::hypot(
          std::hypot(static_cast<double>(predicted.vx - navigation.state.vx),
                     static_cast<double>(predicted.vy - navigation.state.vy)),
          static_cast<double>(predicted.vz - navigation.state.vz));
      latest_prediction_error_.yaw_rad = std::abs(
          std::remainder(static_cast<double>(predicted.yaw - navigation.state.yaw),
                         2.0 * std::numbers::pi));
      latest_prediction_error_.valid = true;
    }
  }
}

void ProductionMppiNode::onRawObstacleSnapshot(
    msg::RawObstacleSnapshot::ConstSharedPtr message) {
  std::scoped_lock lock{raw_queue_mutex_};
  if (pending_raw_snapshot_) {
    ++dropped_raw_snapshots_;
  }
  pending_raw_snapshot_ = std::move(message);
  raw_queue_condition_.notify_all();
}

void ProductionMppiNode::onMemorySnapshot(const msg::ObstacleMemorySnapshot& message) {
  const std::scoped_lock lock{input_mutex_};
  memory_sequence_ = message.sequence;
  memory_receive_stamp_ns_ = get_clock()->now().nanoseconds();
}

void ProductionMppiNode::onAppliedControl(const msg::MppiControlFeedback& message) {
  ProductionMppiAppliedControl feedback;
  feedback.receive_stamp_ns = get_clock()->now().nanoseconds();
  feedback.horizon_sequence = message.horizon_sequence;
  feedback.emergency_braking = message.emergency_braking;
  feedback.valid =
      message.header.frame_id == frame_id_ && std::isfinite(message.acceleration.x) &&
      std::isfinite(message.acceleration.y) && std::isfinite(message.acceleration.z);
  if (feedback.valid) {
    feedback.control.ax = static_cast<float>(message.acceleration.x);
    feedback.control.ay = static_cast<float>(message.acceleration.y);
    feedback.control.az = static_cast<float>(message.acceleration.z);
  }
  const std::scoped_lock lock{input_mutex_};
  applied_control_ = feedback;
}

std::shared_ptr<const ProductionNavigationObjective>
ProductionMppiNode::navigationObjective() const {
  return navigation_objective_.load(std::memory_order_acquire);
}

void ProductionMppiNode::onNavigationObjective(
    const msg::NavigationObjective& message) {
  if (!finitePoint(message.position) ||
      message.terminal_policy >
          msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING) {
    RCLCPP_WARN(get_logger(),
                "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                " sample=%" PRIu64 " reason=invalid_payload",
                message.mission_epoch, message.sample_sequence);
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> previous =
      navigationObjective();
  if (previous && message.mission_epoch < previous->mission_epoch) {
    return;
  }
  if (previous && message.mission_epoch == previous->mission_epoch &&
      message.sample_sequence <= previous->sample_sequence) {
    return;
  }

  const Point3 goal{message.position.x, message.position.y, message.position.z};
  const auto objective = std::make_shared<const ProductionNavigationObjective>(
      ProductionNavigationObjective{
          .goal = goal,
          .mission_epoch = message.mission_epoch,
          .sample_sequence = message.sample_sequence,
          .stamp_ns = timeNanoseconds(message.stamp),
          .continuous_tracking =
              message.terminal_policy ==
              msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING,
      });
  navigation_objective_.store(objective, std::memory_order_release);

  bool request_replan = false;
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  {
    const std::scoped_lock lock{objective_replan_mutex_};
    const bool epoch_changed =
        !previous || previous->mission_epoch != message.mission_epoch;
    const bool moved = pointDistance(goal, objective_replan_anchor_) >=
                       dynamic_objective_replan_distance_m_;
    const bool period_elapsed =
        objective_replan_stamp_ns_ <= 0 ||
        static_cast<double>(now_ns - objective_replan_stamp_ns_) * 1.0e-9 >=
            dynamic_objective_replan_period_s_;
    request_replan = epoch_changed || (moved && period_elapsed);
    if (request_replan) {
      objective_replan_anchor_ = goal;
      objective_replan_stamp_ns_ = now_ns;
    }
  }
  if (request_replan) {
    requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "NAVIGATION_OBJECTIVE accepted mission_epoch=%" PRIu64 " sample=%" PRIu64
      " goal=(%.2f,%.2f,%.2f) policy=%s replan=%s",
      message.mission_epoch, message.sample_sequence, goal.x, goal.y, goal.z,
      objective->continuous_tracking ? "continuous_tracking" : "position_hold",
      request_replan ? "true" : "false");
}

} // namespace drone_city_nav

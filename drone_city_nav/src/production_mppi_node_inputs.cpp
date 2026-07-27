#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

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

} // namespace drone_city_nav

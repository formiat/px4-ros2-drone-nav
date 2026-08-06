#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

namespace drone_city_nav {
namespace {

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

} // namespace

class InterceptSpectatorNode final : public rclcpp::Node {
public:
  InterceptSpectatorNode()
      : Node{"intercept_spectator_node"},
        tf_broadcaster_{std::make_unique<tf2_ros::TransformBroadcaster>(*this)} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    parent_frame_ = declare_parameter<std::string>("parent_frame", "gazebo_map");
    follow_frame_ = declare_parameter<std::string>("follow_frame", "drone_follow");
    ids_ = declare_parameter<std::vector<std::string>>(
        "interceptor_ids", {"interceptor_0", "interceptor_1", "interceptor_2"});
    if (ids_.empty()) {
      throw std::invalid_argument{"at least one spectator target is required"};
    }
    const std::vector<std::string> state_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_state_topics", defaultValues("/state", "/vehicles/"));
    const std::vector<std::string> destroyed_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_destroyed_topics",
            defaultValues("/vehicle_destroyed", "/vehicles/"));
    models_ = declare_parameter<std::vector<std::string>>(
        "gazebo_models", {"x500_lidar_2d_0", "x500_lidar_2d_1", "x500_lidar_2d_2"});
    requireCount(state_topics, ids_.size(), "interceptor_state_topics");
    requireCount(destroyed_topics, ids_.size(), "interceptor_destroyed_topics");
    requireCount(models_, ids_.size(), "gazebo_models");

    states_.resize(ids_.size());
    destroyed_.assign(ids_.size(), false);
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
    for (std::size_t index = 0; index < ids_.size(); ++index) {
      state_subs_.push_back(create_subscription<msg::VehicleNavigationState>(
          state_topics[index], state_qos,
          [this, index](const msg::VehicleNavigationState::SharedPtr state) {
            states_[index] = *state;
          }));
      destroyed_subs_.push_back(create_subscription<msg::VehicleDestroyed>(
          destroyed_topics[index], latched_qos,
          [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
            if (destroyed->vehicle_role != msg::VehicleDestroyed::ROLE_INTERCEPTOR ||
                destroyed->vehicle_id != ids_[index] ||
                (destroyed->mission_epoch != 0U &&
                 destroyed->mission_epoch != mission_epoch_)) {
              return;
            }
            destroyed_[index] = true;
            if (selected_index_ == index) {
              selectFirstLivingTarget();
            }
          }));
    }
    target_pub_ = create_publisher<msg::SpectatorTarget>(
        declare_parameter<std::string>("spectator_target_topic",
                                       "/drone_city_nav/spectator_target"),
        latched_qos);
    selected_index_ = 0U;
    publishSelection();
    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { publishTf(); });
  }

private:
  [[nodiscard]] std::vector<std::string>
  defaultValues(const std::string& suffix, const std::string& prefix) const {
    std::vector<std::string> values;
    values.reserve(ids_.size());
    for (const std::string& id : ids_) {
      std::string value{prefix};
      value.append(id);
      value.append(suffix);
      values.push_back(std::move(value));
    }
    return values;
  }

  void selectFirstLivingTarget() {
    for (std::size_t index = 0; index < destroyed_.size(); ++index) {
      if (!destroyed_[index]) {
        selected_index_ = index;
        publishSelection();
        return;
      }
    }
    RCLCPP_WARN(get_logger(),
                "SPECTATOR_TARGET retained_last=true reason=no_living_interceptor "
                "mission_epoch=%" PRIu64,
                mission_epoch_);
  }

  void publishSelection() {
    if (!selected_index_.has_value()) {
      return;
    }
    const std::size_t selected_index = selected_index_.value();
    msg::SpectatorTarget target;
    target.stamp = now();
    target.mission_epoch = mission_epoch_;
    target.vehicle_id = ids_[selected_index];
    target.gazebo_model = models_[selected_index];
    target_pub_->publish(target);
    RCLCPP_INFO(get_logger(),
                "SPECTATOR_TARGET vehicle_id='%s' gazebo_model='%s' "
                "mission_epoch=%" PRIu64,
                target.vehicle_id.c_str(), target.gazebo_model.c_str(), mission_epoch_);
  }

  void publishTf() {
    if (!selected_index_.has_value()) {
      return;
    }
    const std::size_t selected_index = selected_index_.value();
    const std::optional<msg::VehicleNavigationState> state = states_[selected_index];
    if (!state.has_value()) {
      return;
    }
    if (!state->position_valid) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = follow_frame_;
    transform.transform.translation.x = state->position.y;
    transform.transform.translation.y = state->position.x;
    transform.transform.translation.z = state->position.z;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);
  }

  std::vector<std::string> ids_;
  std::vector<std::string> models_;
  std::vector<std::optional<msg::VehicleNavigationState>> states_;
  std::vector<bool> destroyed_;
  std::optional<std::size_t> selected_index_;
  std::string parent_frame_;
  std::string follow_frame_;
  std::uint64_t mission_epoch_{1U};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr> state_subs_;
  std::vector<rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr> destroyed_subs_;
  rclcpp::Publisher<msg::SpectatorTarget>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptSpectatorNode>());
  rclcpp::shutdown();
  return 0;
}

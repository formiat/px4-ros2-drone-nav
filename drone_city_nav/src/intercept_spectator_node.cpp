#include "drone_city_nav/intercept_spectator_node.hpp"

#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/spectator_selection.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

namespace drone_city_nav {
namespace {

template<typename T>
void requireCount(const std::vector<T>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

} // namespace

class InterceptSpectatorNode final : public rclcpp::Node {
public:
  explicit InterceptSpectatorNode(const rclcpp::NodeOptions& options)
      : Node{"intercept_spectator_node", options},
        tf_broadcaster_{std::make_unique<tf2_ros::TransformBroadcaster>(*this)} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    parent_frame_ = declare_parameter<std::string>("parent_frame", "gazebo_map");
    follow_frame_ = declare_parameter<std::string>("follow_frame", "drone_follow");
    ids_ = declare_parameter<std::vector<std::string>>(
        "vehicle_ids", {"interceptor_0", "interceptor_1", "interceptor_2"});
    if (ids_.empty()) {
      throw std::invalid_argument{"at least one spectator target is required"};
    }
    const std::vector<std::string> state_topics =
        declare_parameter<std::vector<std::string>>(
            "vehicle_state_topics", defaultValues("/state", "/vehicles/"));
    const std::vector<std::string> destroyed_topics =
        declare_parameter<std::vector<std::string>>(
            "vehicle_destroyed_topics",
            defaultValues("/vehicle_destroyed", "/vehicles/"));
    const std::vector<std::int64_t> role_values =
        declare_parameter<std::vector<std::int64_t>>(
            "vehicle_roles", std::vector<std::int64_t>(
                                 ids_.size(), msg::VehicleDestroyed::ROLE_INTERCEPTOR));
    models_ = declare_parameter<std::vector<std::string>>(
        "gazebo_models", {"x500_lidar_2d_0", "x500_lidar_2d_1", "x500_lidar_2d_2"});
    requireCount(state_topics, ids_.size(), "vehicle_state_topics");
    requireCount(destroyed_topics, ids_.size(), "vehicle_destroyed_topics");
    requireCount(role_values, ids_.size(), "vehicle_roles");
    requireCount(models_, ids_.size(), "gazebo_models");
    expected_roles_.reserve(role_values.size());
    for (const std::int64_t role : role_values) {
      if (role != msg::VehicleDestroyed::ROLE_INTERCEPTOR &&
          role != msg::VehicleDestroyed::ROLE_EVADER) {
        throw std::invalid_argument{"vehicle_roles contains an unsupported role"};
      }
      if (role < 0 || role > std::numeric_limits<std::uint8_t>::max()) {
        throw std::invalid_argument{"vehicle_roles contains an out-of-range role"};
      }
      expected_roles_.push_back(static_cast<std::uint8_t>(role));
    }

    const std::string initial_vehicle_id =
        declare_parameter<std::string>("initial_vehicle_id", ids_.front());
    const auto initial_iterator =
        std::find(ids_.begin(), ids_.end(), initial_vehicle_id);
    if (initial_iterator == ids_.end()) {
      throw std::invalid_argument{"initial_vehicle_id is not present in vehicle_ids"};
    }
    const std::string policy_name =
        declare_parameter<std::string>("reselection_policy", "first_living");
    const std::optional<SpectatorReselectionPolicy> policy =
        parseSpectatorReselectionPolicy(policy_name);
    if (!policy.has_value()) {
      throw std::invalid_argument{
          "reselection_policy must be first_living or next_living"};
    }
    reselection_policy_ = policy.value();
    const auto initial_index =
        static_cast<std::size_t>(std::distance(ids_.begin(), initial_iterator));
    selection_.emplace(ids_.size(), initial_index, reselection_policy_);

    states_.resize(ids_.size());
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
            if (destroyed->vehicle_role != expected_roles_[index] ||
                destroyed->vehicle_id != ids_[index] ||
                (destroyed->mission_epoch != 0U &&
                 destroyed->mission_epoch != mission_epoch_)) {
              return;
            }
            const bool selected = selection_->currentIndex() == index;
            const std::optional<std::size_t> replacement =
                selection_->markDestroyed(index);
            if (selected && replacement.has_value()) {
              publishSelection();
            } else if (selected) {
              RCLCPP_WARN(
                  get_logger(),
                  "SPECTATOR_TARGET retained_last=true reason=no_living_vehicle "
                  "vehicle_id='%s' mission_epoch=%" PRIu64,
                  ids_[index].c_str(), mission_epoch_);
            }
          }));
    }
    target_pub_ = create_publisher<msg::SpectatorTarget>(
        declare_parameter<std::string>("spectator_target_topic",
                                       "/drone_city_nav/spectator_target"),
        latched_qos);
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

  void publishSelection() {
    if (!selection_.has_value()) {
      return;
    }
    const std::size_t selected_index = selection_->currentIndex();
    msg::SpectatorTarget target;
    target.stamp = now();
    target.mission_epoch = mission_epoch_;
    target.vehicle_id = ids_[selected_index];
    target.gazebo_model = models_[selected_index];
    target_pub_->publish(target);
    RCLCPP_INFO(get_logger(),
                "SPECTATOR_TARGET vehicle_id='%s' gazebo_model='%s' "
                "mission_epoch=%" PRIu64 " reselection_policy='%s'",
                target.vehicle_id.c_str(), target.gazebo_model.c_str(), mission_epoch_,
                spectatorReselectionPolicyName(reselection_policy_).data());
  }

  void publishTf() {
    if (!selection_.has_value()) {
      return;
    }
    const std::size_t selected_index = selection_->currentIndex();
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
  std::vector<std::uint8_t> expected_roles_;
  std::vector<std::optional<msg::VehicleNavigationState>> states_;
  std::optional<SpectatorSelection> selection_;
  SpectatorReselectionPolicy reselection_policy_{
      SpectatorReselectionPolicy::kFirstLiving};
  std::string parent_frame_;
  std::string follow_frame_;
  std::uint64_t mission_epoch_{1U};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr> state_subs_;
  std::vector<rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr> destroyed_subs_;
  rclcpp::Publisher<msg::SpectatorTarget>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node>
makeInterceptSpectatorNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<InterceptSpectatorNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::InterceptSpectatorNode)

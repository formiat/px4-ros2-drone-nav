#pragma once

#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace drone_city_nav {

// Topics of the autopilot's own messages; an empty topic is not subscribed.
struct AutopilotStateTopics {
  std::string local_state;
  std::string attitude;
  std::string clock_sync;
  std::string status;
  std::string ground_contact;
};

struct AutopilotStateCallbacks {
  std::function<void(const AutopilotLocalState&)> local_state;
  std::function<void(const AutopilotAttitude&)> attitude;
  std::function<void(const AutopilotClockSync&)> clock_sync;
  std::function<void(const AutopilotStatus&)> status;
  std::function<void(const AutopilotGroundContact&)> ground_contact;
};

// Subscribes a node to the autopilot's messages and delivers them as the
// stack's contract. The one implementation is the PX4 adapter
// (px4_autopilot_adapter.cpp): it owns the frame, the altitude sign and the
// heading convention, and no other source includes the autopilot's messages.
class AutopilotStateSource final {
public:
  AutopilotStateSource(rclcpp::Node& node, const Px4MapFrameTransform& transform,
                       const AutopilotStateTopics& topics, const rclcpp::QoS& qos,
                       AutopilotStateCallbacks callbacks,
                       const rclcpp::SubscriptionOptions& state_options = {},
                       const rclcpp::SubscriptionOptions& status_options = {});

private:
  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
};

} // namespace drone_city_nav

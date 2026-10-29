// The step acknowledgement of roadmap item 36 (specification K53): under a
// simulator that waits for the onboard loop, each onboard node tells the step
// controller (a simulation process in scripts/) when it has
// processed everything the step brought it, and the world does not move on
// before every node has. The acknowledgement follows the callbacks of the
// group it rides in (the step's message is read in the same mutually
// exclusive group as the node's heavy work, so it runs after that work) and
// waits for the node's worker threads through a predicate. Off by default
// (`step_ack_enabled`): on the vehicle the topic is silent and nothing here
// runs.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace drone_city_nav {

class StepAcknowledger {
public:
  // name: the leaf of the acknowledgement topic; group: the callback group the
  // acknowledgement follows (nullptr: the node's default); idle: polled until
  // true before the acknowledgement (the worker threads' outputs are in).
  StepAcknowledger(rclcpp::Node& node, const std::string& name,
                   rclcpp::CallbackGroup::SharedPtr group, std::function<bool()> idle)
      : node_{node},
        idle_{std::move(idle)} {
    if (!node.declare_parameter<bool>("step_ack_enabled", false)) {
      return;
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = std::move(group);
    acknowledgement_ = node.create_publisher<std_msgs::msg::UInt64>(
        "/drone_city_nav/step_ack/" + name, rclcpp::QoS{10}.reliable());
    step_ = node.create_subscription<std_msgs::msg::UInt64>(
        "/drone_city_nav/step", rclcpp::QoS{10}.reliable(),
        [this](const std_msgs::msg::UInt64& step) { acknowledge(step); }, options);
  }

  [[nodiscard]] bool enabled() const noexcept {
    return step_ != nullptr;
  }

private:
  void acknowledge(const std_msgs::msg::UInt64& step) {
    // The worker threads' outputs of the step are in once the predicate holds;
    // a node stuck past five seconds acknowledges anyway, and the controller's
    // own timeout names it.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (!idle_() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::microseconds{200});
    }
    acknowledgement_->publish(step);
  }

  rclcpp::Node& node_;
  std::function<bool()> idle_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr acknowledgement_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr step_;
};

} // namespace drone_city_nav

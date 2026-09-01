#include "drone_city_nav/cooperative_traffic_ros.hpp"

#include <cinttypes>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::createCooperativeTrafficInterfaces(
    const rclcpp::SubscriptionOptions& subscription_options) {
  if (!config_.planning.cooperative_traffic_enabled) {
    return;
  }
  const auto command_qos = rclcpp::QoS{4}.reliable();
  cooperative_command_sub_ = create_subscription<msg::CooperativeManeuverCommand>(
      config_.planning.topics.cooperative_maneuver_command, command_qos,
      [this](const msg::CooperativeManeuverCommand::SharedPtr message) {
        onCooperativeManeuverCommand(*message);
      },
      subscription_options);
  cooperative_passage_state_pub_ = create_publisher<msg::CooperativePassageIntent>(
      config_.planning.topics.cooperative_passage_state, command_qos);
}

void ProductionMppiNode::onCooperativeManeuverCommand(
    const msg::CooperativeManeuverCommand& message) {
  const CooperativeManeuverCommandData command =
      cooperativeManeuverCommandData(message);
  if (message.header.frame_id != config_.world.frame_id ||
      command.vehicle_id != config_.planning.vehicle_id ||
      command.command_generation == 0U || command.stamp_ns <= 0 ||
      command.valid_until_ns < command.stamp_ns) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "COOPERATIVE_COMMAND_REJECTED vehicle_id='%s' source_vehicle_id='%s' "
        "generation=%" PRIu64 " reason=invalid_contract",
        config_.planning.vehicle_id.c_str(), command.vehicle_id.c_str(),
        command.command_generation);
    return;
  }
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const std::scoped_lock lock{input_mutex_};
  if (cooperative_command_.has_value()) {
    const CooperativeManeuverCommandData& previous = cooperative_command_->data;
    if (command.stamp_ns < previous.stamp_ns ||
        (command.stamp_ns == previous.stamp_ns &&
         command.command_generation <= previous.command_generation)) {
      return;
    }
  }
  cooperative_command_ = ProductionMppiCooperativeCommand{
      .data = command,
      .receive_stamp_ns = receive_stamp_ns,
  };
}

} // namespace drone_city_nav

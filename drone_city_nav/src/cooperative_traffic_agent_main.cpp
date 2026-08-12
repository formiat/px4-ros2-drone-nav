#include "drone_city_nav/cooperative_traffic_agent_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeCooperativeTrafficAgentNode());
  rclcpp::shutdown();
  return 0;
}

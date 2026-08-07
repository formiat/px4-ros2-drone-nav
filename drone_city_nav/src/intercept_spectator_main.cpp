#include "drone_city_nav/intercept_spectator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeInterceptSpectatorNode());
  rclcpp::shutdown();
  return 0;
}

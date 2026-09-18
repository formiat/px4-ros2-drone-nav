#include "drone_city_nav/multi_vehicle_spectator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeMultiVehicleSpectatorNode());
  rclcpp::shutdown();
  return 0;
}

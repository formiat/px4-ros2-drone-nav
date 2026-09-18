#include "drone_city_nav/multi_vehicle_diagnostics_mux_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeMultiVehicleDiagnosticsMuxNode());
  rclcpp::shutdown();
  return 0;
}

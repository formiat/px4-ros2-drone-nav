#include "drone_city_nav/world_visualization_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeWorldVisualizationNode());
  rclcpp::shutdown();
  return 0;
}

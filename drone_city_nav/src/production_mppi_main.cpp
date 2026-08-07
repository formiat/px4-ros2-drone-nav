#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "production_mppi_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<drone_city_nav::ProductionMppiNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "cooperative_traffic_referee_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::CooperativeTrafficRefereeNode>());
  rclcpp::shutdown();
  return 0;
}

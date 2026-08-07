#include "drone_city_nav/interceptor_guidance_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeInterceptorGuidanceNode());
  rclcpp::shutdown();
  return 0;
}

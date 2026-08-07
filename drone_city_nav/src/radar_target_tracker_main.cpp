#include "drone_city_nav/radar_target_tracker_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(drone_city_nav::makeRadarTargetTrackerNode());
  rclcpp::shutdown();
  return 0;
}

#include "planner_node.hpp"

#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_city_nav::PlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 3U};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

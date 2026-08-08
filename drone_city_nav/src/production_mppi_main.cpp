#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "production_mppi_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<drone_city_nav::ProductionMppiNode>(rclcpp::NodeOptions{});
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2U};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

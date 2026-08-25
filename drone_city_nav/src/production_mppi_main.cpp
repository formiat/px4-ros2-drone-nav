#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

#include "production_mppi_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node =
        std::make_shared<drone_city_nav::ProductionMppiNode>(rclcpp::NodeOptions{});
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 3U};
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("production_mppi_node"),
                 "PLANNER_STARTUP_FAILURE category=initialization detail='%s'",
                 error.what());
    rclcpp::shutdown();
    return 2;
  } catch (...) {
    RCLCPP_FATAL(
        rclcpp::get_logger("production_mppi_node"),
        "PLANNER_STARTUP_FAILURE category=unknown detail='non-standard exception'");
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

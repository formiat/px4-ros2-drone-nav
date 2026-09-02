#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <exception>
#include <memory>

#include "production_mppi_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node =
        std::make_shared<drone_city_nav::ProductionMppiNode>(rclcpp::NodeOptions{});
    // Planning, world reconstruction, high-rate vehicle input, lidar evidence,
    // and the default group each need a thread of their own so a long planning
    // tick or raw reconstruction can never starve control feedback.
    constexpr std::size_t kCallbackGroupCount{5U};
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{},
                                                      kCallbackGroupCount + 1U};
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

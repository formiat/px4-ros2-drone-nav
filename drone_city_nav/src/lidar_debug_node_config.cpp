#include "drone_city_nav/lidar_debug_node_config.hpp"

namespace drone_city_nav {

void sanitizeLidarDebugNodeConfig(LidarDebugNodeConfig& config) {
  config.snapshot_period_s = std::max(0.1, config.snapshot_period_s);
  config.view_radius_m = std::max(5.0, config.view_radius_m);
  config.max_lidar_range_m = std::max(1.0, config.max_lidar_range_m);
  config.range_hit_epsilon_m = std::max(0.0, config.range_hit_epsilon_m);
  config.hit_memory_resolution_m = std::max(0.05, config.hit_memory_resolution_m);
  config.beam_csv_stride = std::clamp<std::size_t>(config.beam_csv_stride, 1U, 100000U);
  config.max_logged_hit_points =
      std::min<std::size_t>(config.max_logged_hit_points, 100000U);
  config.max_remembered_hit_points =
      std::clamp<std::size_t>(config.max_remembered_hit_points, 1U, 1000000U);
}

} // namespace drone_city_nav

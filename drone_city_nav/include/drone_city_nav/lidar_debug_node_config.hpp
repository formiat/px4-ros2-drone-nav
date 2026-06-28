#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace drone_city_nav {

struct LidarDebugNodeConfig {
  std::string output_dir{"log/lidar_debug"};
  double snapshot_period_s{1.0};
  double view_radius_m{45.0};
  double max_lidar_range_m{35.0};
  double range_hit_epsilon_m{0.05};
  double hit_memory_resolution_m{0.25};
  std::size_t beam_csv_stride{1U};
  std::size_t max_logged_hit_points{256U};
  std::size_t max_remembered_hit_points{50000U};
  std::uint64_t max_snapshots{0U};
};

void sanitizeLidarDebugNodeConfig(LidarDebugNodeConfig& config);

} // namespace drone_city_nav

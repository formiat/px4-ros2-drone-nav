#pragma once

#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct LidarBeamSample3D {
  Vec3 direction_lidar_flu{};
  double range_m{0.0};
  bool hit{false};
  bool valid{false};
};

struct OrganizedLidarScan3DConfig {
  std::size_t horizontal_samples{240U};
  std::size_t vertical_samples{19U};
  double horizontal_min_angle_rad{-3.14159265358979323846};
  double horizontal_max_angle_rad{3.14159265358979323846};
  double vertical_min_angle_rad{-1.5707963267948966192};
  double vertical_max_angle_rad{1.5707963267948966192};
  double minimum_range_m{0.2};
  double maximum_range_m{35.0};
  double hit_epsilon_m{0.05};
};

struct OrganizedLidarScan3DResult {
  std::vector<LidarBeamSample3D> beams;
  std::size_t hit_beams{0U};
  std::size_t miss_beams{0U};
  std::size_t invalid_beams{0U};
  bool organized_dimensions_match{false};
};

[[nodiscard]] bool
organizedLidarScan3DConfigIsValid(const OrganizedLidarScan3DConfig& config) noexcept;

[[nodiscard]] OrganizedLidarScan3DResult
decodeOrganizedLidarScan3D(std::span<const Point3> returns_lidar_flu,
                           const OrganizedLidarScan3DConfig& config);

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace drone_city_nav {

enum class StaticMapSourceStatus {
  kDisabled,
  kLoaded,
  kLoadFailed,
};

struct StaticMapSourceConfig {
  bool enabled{true};
  std::filesystem::path configured_path{"worlds/generated_city.map2d"};
  std::filesystem::path package_share_directory;
  std::string expected_frame_id{"map"};
  double min_blocking_height_m{0.0};
};

struct StaticMapSourceResult {
  StaticMapSourceStatus status{StaticMapSourceStatus::kDisabled};
  std::filesystem::path resolved_path;
  std::optional<OccupancyGrid2D> grid;
  std::string map_frame_id;
  std::size_t rectangles{0U};
  std::size_t occupied_cells{0U};
  bool frame_matches{true};
  std::string error_message;
};

[[nodiscard]] std::filesystem::path
resolveStaticMapPath(const std::filesystem::path& configured_path,
                     const std::filesystem::path& package_share_directory);

[[nodiscard]] StaticMapSourceResult
loadStaticMapSource(const StaticMapSourceConfig& config);

} // namespace drone_city_nav

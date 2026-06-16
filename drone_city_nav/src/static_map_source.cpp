#include "drone_city_nav/static_map_source.hpp"

#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/static_city_map.hpp"

#include <exception>

namespace drone_city_nav {

const char* staticMapSourceStatusName(const StaticMapSourceStatus status) noexcept {
  switch (status) {
    case StaticMapSourceStatus::kDisabled:
      return "disabled";
    case StaticMapSourceStatus::kLoaded:
      return "loaded";
    case StaticMapSourceStatus::kLoadFailed:
      return "load_failed";
  }
  return "unknown";
}

std::filesystem::path
resolveStaticMapPath(const std::filesystem::path& configured_path,
                     const std::filesystem::path& package_share_directory) {
  std::filesystem::path path =
      configured_path.empty() ? std::filesystem::path{"worlds/generated_city.map2d"}
                              : configured_path;
  if (path.is_absolute()) {
    return path;
  }
  if (std::filesystem::exists(path)) {
    return std::filesystem::absolute(path);
  }
  if (package_share_directory.empty()) {
    return path;
  }

  std::filesystem::path package_candidate = package_share_directory / path;
  if (std::filesystem::exists(package_candidate)) {
    return package_candidate;
  }
  std::filesystem::path worlds_candidate =
      package_share_directory / "worlds" / path.filename();
  if (std::filesystem::exists(worlds_candidate)) {
    return worlds_candidate;
  }

  return path;
}

StaticMapSourceResult loadStaticMapSource(const StaticMapSourceConfig& config) {
  StaticMapSourceResult result{};
  result.resolved_path =
      resolveStaticMapPath(config.configured_path, config.package_share_directory);
  if (!config.enabled) {
    result.status = StaticMapSourceStatus::kDisabled;
    return result;
  }

  try {
    const StaticCityMap static_map = loadStaticCityMap(result.resolved_path);
    result.map_frame_id = static_map.frame_id;
    result.frame_matches = static_map.frame_id == config.expected_frame_id;
    result.rectangles = static_map.rectangles.size();
    result.grid = rasterizeStaticCityMap(static_map, config.min_blocking_height_m);
    const GridStats stats = collectGridStats(*result.grid);
    result.occupied_cells = stats.occupied_cells;
    result.status = StaticMapSourceStatus::kLoaded;
  } catch (const std::exception& error) {
    result.status = StaticMapSourceStatus::kLoadFailed;
    result.error_message = error.what();
    result.grid.reset();
    result.rectangles = 0U;
    result.occupied_cells = 0U;
  }

  return result;
}

} // namespace drone_city_nav

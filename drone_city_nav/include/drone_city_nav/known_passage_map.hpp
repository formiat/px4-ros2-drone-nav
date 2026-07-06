#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct PassageOpening {
  std::string id;
  std::string structure_id;
  Point3 center{};
  Point2 normal_xy{1.0, 0.0};
  double width_m{0.0};
  double height_m{0.0};
  double depth_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double approach_distance_m{0.0};
  double exit_distance_m{0.0};
};

struct PassageStructure {
  std::string id;
  Point2 center{};
  double size_x_m{0.0};
  double size_y_m{0.0};
  double z_min_m{0.0};
  double z_max_m{0.0};
  std::vector<PassageOpening> openings;
};

struct KnownPassageMap {
  std::string frame_id;
  std::vector<PassageStructure> structures;
};

enum class KnownPassageSourceStatus {
  kDisabled,
  kLoaded,
  kLoadFailed,
};

struct KnownPassageSourceConfig {
  bool enabled{true};
  std::filesystem::path configured_path{"worlds/known_passages.passages3d"};
  std::filesystem::path package_share_directory;
  std::string expected_frame_id{"map"};
};

struct KnownPassageSourceResult {
  KnownPassageSourceStatus status{KnownPassageSourceStatus::kDisabled};
  std::filesystem::path resolved_path;
  std::optional<KnownPassageMap> map;
  bool frame_matches{false};
  std::size_t structures{0U};
  std::size_t openings{0U};
  std::string error_message;
};

[[nodiscard]] const char*
knownPassageSourceStatusName(KnownPassageSourceStatus status) noexcept;

[[nodiscard]] KnownPassageMap loadKnownPassageMap(const std::filesystem::path& path);

[[nodiscard]] std::filesystem::path
resolveKnownPassageMapPath(const std::filesystem::path& configured_path,
                           const std::filesystem::path& package_share_directory);

[[nodiscard]] KnownPassageSourceResult
loadKnownPassageMapSource(const KnownPassageSourceConfig& config);

} // namespace drone_city_nav

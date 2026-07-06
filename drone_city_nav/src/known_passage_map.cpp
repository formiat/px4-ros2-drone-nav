#include "drone_city_nav/known_passage_map.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr const char* kKnownPassageMapVersion = "drone_city_nav_known_passages_v1";
constexpr double kTinyDistanceM = 1.0e-9;

[[nodiscard]] bool finiteNonNegative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] std::string stripComment(const std::string& line) {
  const std::size_t comment_position = line.find('#');
  if (comment_position == std::string::npos) {
    return line;
  }
  return line.substr(0, comment_position);
}

[[nodiscard]] double parseDouble(std::istringstream& stream, const char* field_name,
                                 const int line_number) {
  double value = 0.0;
  if (!(stream >> value) || !std::isfinite(value)) {
    throw std::runtime_error{"Invalid numeric field '" + std::string{field_name} +
                             "' at known passage map line " +
                             std::to_string(line_number)};
  }
  return value;
}

void requireNoTrailingTokens(std::istringstream& stream, const int line_number) {
  std::string trailing;
  if (stream >> trailing) {
    throw std::runtime_error{"Unexpected trailing token '" + trailing +
                             "' at known passage map line " +
                             std::to_string(line_number)};
  }
}

[[nodiscard]] bool centerInsideStructureFootprint(const Point2 center,
                                                  const PassageStructure& structure) {
  const double half_x = structure.size_x_m / 2.0;
  const double half_y = structure.size_y_m / 2.0;
  return center.x >= structure.center.x - half_x &&
         center.x <= structure.center.x + half_x &&
         center.y >= structure.center.y - half_y &&
         center.y <= structure.center.y + half_y;
}

[[nodiscard]] PassageStructure* findStructure(KnownPassageMap& map,
                                              const std::string& id) {
  const auto iter = std::find_if(
      map.structures.begin(), map.structures.end(),
      [&id](const PassageStructure& structure) { return structure.id == id; });
  if (iter == map.structures.end()) {
    return nullptr;
  }
  return &*iter;
}

[[nodiscard]] bool containsOpeningId(const PassageStructure& structure,
                                     const std::string& opening_id) {
  return std::any_of(structure.openings.begin(), structure.openings.end(),
                     [&opening_id](const PassageOpening& opening) {
                       return opening.id == opening_id;
                     });
}

[[nodiscard]] Point2 normalizedNormal(Point2 normal, const int line_number) {
  const double length = std::hypot(normal.x, normal.y);
  if (!(length > kTinyDistanceM) || !std::isfinite(length)) {
    throw std::runtime_error{"Known passage opening normal is invalid at line " +
                             std::to_string(line_number)};
  }
  normal.x /= length;
  normal.y /= length;
  return normal;
}

void validateStructure(const PassageStructure& structure, const int line_number) {
  if (!finitePositive(structure.size_x_m) || !finitePositive(structure.size_y_m) ||
      !finiteNonNegative(structure.z_min_m) || !std::isfinite(structure.z_max_m) ||
      structure.z_max_m <= structure.z_min_m) {
    throw std::runtime_error{"Known passage structure '" + structure.id +
                             "' has invalid dimensions at line " +
                             std::to_string(line_number)};
  }
}

void validateOpening(const PassageOpening& opening, const PassageStructure& structure,
                     const int line_number) {
  if (!finitePositive(opening.width_m) || !finitePositive(opening.height_m) ||
      !finitePositive(opening.depth_m) ||
      !finitePositive(opening.approach_distance_m) ||
      !finitePositive(opening.exit_distance_m) || !finiteNonNegative(opening.min_z_m) ||
      !std::isfinite(opening.max_z_m) || opening.max_z_m <= opening.min_z_m) {
    throw std::runtime_error{"Known passage opening '" + opening.id +
                             "' has invalid dimensions at line " +
                             std::to_string(line_number)};
  }

  if (opening.min_z_m < structure.z_min_m || opening.max_z_m > structure.z_max_m) {
    throw std::runtime_error{"Known passage opening '" + opening.id +
                             "' z range is outside structure '" + structure.id +
                             "' at line " + std::to_string(line_number)};
  }
  if (opening.center.z < opening.min_z_m || opening.center.z > opening.max_z_m) {
    throw std::runtime_error{"Known passage opening '" + opening.id +
                             "' center z is outside opening z range at line " +
                             std::to_string(line_number)};
  }
  if (!centerInsideStructureFootprint(Point2{opening.center.x, opening.center.y},
                                      structure)) {
    throw std::runtime_error{"Known passage opening '" + opening.id +
                             "' center is outside structure footprint at line " +
                             std::to_string(line_number)};
  }
}

[[nodiscard]] std::size_t countOpenings(const KnownPassageMap& map) noexcept {
  std::size_t openings = 0U;
  for (const PassageStructure& structure : map.structures) {
    openings += structure.openings.size();
  }
  return openings;
}

} // namespace

const char*
knownPassageSourceStatusName(const KnownPassageSourceStatus status) noexcept {
  switch (status) {
    case KnownPassageSourceStatus::kDisabled:
      return "disabled";
    case KnownPassageSourceStatus::kLoaded:
      return "loaded";
    case KnownPassageSourceStatus::kLoadFailed:
      return "load_failed";
  }
  return "unknown";
}

KnownPassageMap loadKnownPassageMap(const std::filesystem::path& path) {
  std::ifstream input{path};
  if (!input) {
    throw std::runtime_error{"Unable to open known passage map: " + path.string()};
  }

  KnownPassageMap map{};
  std::unordered_set<std::string> structure_ids;
  bool version_seen = false;

  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream stream{stripComment(line)};
    std::string keyword;
    if (!(stream >> keyword)) {
      continue;
    }

    if (!version_seen) {
      if (keyword != kKnownPassageMapVersion) {
        throw std::runtime_error{"Invalid known passage map version at line " +
                                 std::to_string(line_number)};
      }
      version_seen = true;
      requireNoTrailingTokens(stream, line_number);
      continue;
    }

    if (keyword == "frame_id") {
      if (!map.frame_id.empty()) {
        throw std::runtime_error{"Duplicate frame_id at known passage map line " +
                                 std::to_string(line_number)};
      }
      if (!(stream >> map.frame_id) || map.frame_id.empty()) {
        throw std::runtime_error{"Missing frame_id value at known passage map line " +
                                 std::to_string(line_number)};
      }
      requireNoTrailingTokens(stream, line_number);
      continue;
    }

    if (keyword == "structure") {
      PassageStructure structure{};
      if (!(stream >> structure.id) || structure.id.empty()) {
        throw std::runtime_error{"Missing structure id at known passage map line " +
                                 std::to_string(line_number)};
      }
      if (!structure_ids.insert(structure.id).second) {
        throw std::runtime_error{"Duplicate known passage structure id '" +
                                 structure.id + "' at line " +
                                 std::to_string(line_number)};
      }
      structure.center.x = parseDouble(stream, "center_x_m", line_number);
      structure.center.y = parseDouble(stream, "center_y_m", line_number);
      structure.size_x_m = parseDouble(stream, "size_x_m", line_number);
      structure.size_y_m = parseDouble(stream, "size_y_m", line_number);
      structure.z_min_m = parseDouble(stream, "z_min_m", line_number);
      structure.z_max_m = parseDouble(stream, "z_max_m", line_number);
      requireNoTrailingTokens(stream, line_number);
      validateStructure(structure, line_number);
      map.structures.push_back(std::move(structure));
      continue;
    }

    if (keyword == "opening") {
      std::string structure_id;
      std::string opening_id;
      if (!(stream >> structure_id) || structure_id.empty() ||
          !(stream >> opening_id) || opening_id.empty()) {
        throw std::runtime_error{
            "Missing opening structure/id at known passage map line " +
            std::to_string(line_number)};
      }
      PassageStructure* structure = findStructure(map, structure_id);
      if (structure == nullptr) {
        throw std::runtime_error{"Known passage opening references unknown "
                                 "structure '" +
                                 structure_id + "' at line " +
                                 std::to_string(line_number)};
      }
      if (containsOpeningId(*structure, opening_id)) {
        std::ostringstream message;
        message << "Duplicate known passage opening id '" << opening_id
                << "' in structure '" << structure_id << "' at line " << line_number;
        throw std::runtime_error{message.str()};
      }

      PassageOpening opening{};
      opening.structure_id = structure_id;
      opening.id = opening_id;
      opening.center.x = parseDouble(stream, "center_x_m", line_number);
      opening.center.y = parseDouble(stream, "center_y_m", line_number);
      opening.center.z = parseDouble(stream, "center_z_m", line_number);
      opening.normal_xy.x = parseDouble(stream, "normal_x", line_number);
      opening.normal_xy.y = parseDouble(stream, "normal_y", line_number);
      opening.normal_xy = normalizedNormal(opening.normal_xy, line_number);
      opening.width_m = parseDouble(stream, "width_m", line_number);
      opening.height_m = parseDouble(stream, "height_m", line_number);
      opening.depth_m = parseDouble(stream, "depth_m", line_number);
      opening.min_z_m = parseDouble(stream, "min_z_m", line_number);
      opening.max_z_m = parseDouble(stream, "max_z_m", line_number);
      opening.approach_distance_m =
          parseDouble(stream, "approach_distance_m", line_number);
      opening.exit_distance_m = parseDouble(stream, "exit_distance_m", line_number);
      requireNoTrailingTokens(stream, line_number);
      validateOpening(opening, *structure, line_number);
      structure->openings.push_back(std::move(opening));
      continue;
    }

    throw std::runtime_error{"Unknown known passage map keyword '" + keyword +
                             "' at line " + std::to_string(line_number)};
  }

  if (!version_seen) {
    throw std::runtime_error{"Known passage map is empty or missing version"};
  }
  if (map.frame_id.empty()) {
    throw std::runtime_error{"Known passage map is missing frame_id"};
  }
  return map;
}

std::filesystem::path
resolveKnownPassageMapPath(const std::filesystem::path& configured_path,
                           const std::filesystem::path& package_share_directory) {
  std::filesystem::path path =
      configured_path.empty()
          ? std::filesystem::path{"worlds/known_passages.passages3d"}
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

KnownPassageSourceResult
loadKnownPassageMapSource(const KnownPassageSourceConfig& config) {
  KnownPassageSourceResult result{};
  result.resolved_path = resolveKnownPassageMapPath(config.configured_path,
                                                    config.package_share_directory);
  if (!config.enabled) {
    result.status = KnownPassageSourceStatus::kDisabled;
    return result;
  }

  try {
    result.map = loadKnownPassageMap(result.resolved_path);
    result.frame_matches = result.map->frame_id == config.expected_frame_id;
    result.structures = result.map->structures.size();
    result.openings = countOpenings(*result.map);
    result.status = KnownPassageSourceStatus::kLoaded;
  } catch (const std::exception& error) {
    result.status = KnownPassageSourceStatus::kLoadFailed;
    result.error_message = error.what();
    result.map.reset();
    result.frame_matches = false;
    result.structures = 0U;
    result.openings = 0U;
  }

  return result;
}

} // namespace drone_city_nav

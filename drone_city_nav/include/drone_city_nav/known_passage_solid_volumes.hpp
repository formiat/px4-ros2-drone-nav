#pragma once

#include "drone_city_nav/known_passage_map.hpp"

#include <string>
#include <vector>

namespace drone_city_nav {

struct KnownPassageSolidVolume {
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  Point2 center{};
  Point2 normal_xy{1.0, 0.0};
  Point2 lateral_xy{0.0, 1.0};
  double depth_m{0.0};
  double width_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
};

[[nodiscard]] std::vector<KnownPassageSolidVolume>
knownPassageSolidVolumes(const PassageStructure& structure);

[[nodiscard]] std::vector<KnownPassageSolidVolume>
knownPassageSolidVolumes(const KnownPassageMap& map);

} // namespace drone_city_nav

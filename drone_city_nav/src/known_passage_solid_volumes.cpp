#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include "drone_city_nav/known_passage_geometry.hpp"

#include <algorithm>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point2 add(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 scale(const Point2 point, const double factor) noexcept {
  return Point2{point.x * factor, point.y * factor};
}

[[nodiscard]] double
structureLateralWidthM(const PassageStructure& structure) noexcept {
  // The passage map stores the opening depth explicitly. The wider footprint
  // dimension is the architectural span across the opening, matching the SDF
  // building-with-passage generator and avoiding a separate orientation field.
  return std::max(structure.size_x_m, structure.size_y_m);
}

void appendVolume(std::vector<KnownPassageSolidVolume>& volumes,
                  const PassageStructure& structure, const PassageOpening& opening,
                  const KnownPassageOpeningFrame& frame, const char* part_id,
                  const KnownPassageSolidPartKind part_kind, const Point2 center,
                  const double depth_m, const double width_m, const double min_z_m,
                  const double max_z_m) {
  if (!(depth_m > 0.0) || !(width_m > 0.0) || !(max_z_m > min_z_m)) {
    return;
  }

  volumes.push_back(KnownPassageSolidVolume{
      .structure_id = structure.id,
      .opening_id = opening.id,
      .part_id = part_id,
      .part_kind = part_kind,
      .center = center,
      .normal_xy = frame.normal,
      .lateral_xy = frame.lateral,
      .depth_m = depth_m,
      .width_m = width_m,
      .min_z_m = min_z_m,
      .max_z_m = max_z_m,
  });
}

void appendOpeningVolumes(std::vector<KnownPassageSolidVolume>& volumes,
                          const PassageStructure& structure,
                          const PassageOpening& opening) {
  const std::optional<KnownPassageOpeningFrame> frame =
      knownPassageOpeningFrame(opening);
  if (!frame.has_value()) {
    return;
  }

  const double lateral_width_m = structureLateralWidthM(structure);
  const double side_width_m = (lateral_width_m - opening.width_m) / 2.0;
  const double side_center_offset_m = (opening.width_m / 2.0) + (side_width_m / 2.0);
  appendVolume(volumes, structure, opening, *frame, "left_mass",
               KnownPassageSolidPartKind::kLeft,
               add(frame->center, scale(frame->lateral, -side_center_offset_m)),
               opening.depth_m, side_width_m, structure.z_min_m, structure.z_max_m);
  appendVolume(volumes, structure, opening, *frame, "right_mass",
               KnownPassageSolidPartKind::kRight,
               add(frame->center, scale(frame->lateral, side_center_offset_m)),
               opening.depth_m, side_width_m, structure.z_min_m, structure.z_max_m);
  appendVolume(volumes, structure, opening, *frame, "lower_mass",
               KnownPassageSolidPartKind::kLower, frame->center, opening.depth_m,
               opening.width_m, structure.z_min_m, opening.min_z_m);
  appendVolume(volumes, structure, opening, *frame, "upper_mass",
               KnownPassageSolidPartKind::kUpper, frame->center, opening.depth_m,
               opening.width_m, opening.max_z_m, structure.z_max_m);
}

} // namespace

const char*
knownPassageSolidPartKindName(const KnownPassageSolidPartKind kind) noexcept {
  switch (kind) {
    case KnownPassageSolidPartKind::kLeft:
      return "left";
    case KnownPassageSolidPartKind::kRight:
      return "right";
    case KnownPassageSolidPartKind::kLower:
      return "lower";
    case KnownPassageSolidPartKind::kUpper:
      return "upper";
  }
  return "unknown";
}

std::vector<KnownPassageSolidVolume>
knownPassageSolidVolumes(const PassageStructure& structure) {
  std::vector<KnownPassageSolidVolume> volumes;
  volumes.reserve(structure.openings.size() * 4U);
  for (const PassageOpening& opening : structure.openings) {
    appendOpeningVolumes(volumes, structure, opening);
  }
  return volumes;
}

std::vector<KnownPassageSolidVolume>
knownPassageSolidVolumes(const KnownPassageMap& map) {
  std::vector<KnownPassageSolidVolume> volumes;
  for (const PassageStructure& structure : map.structures) {
    std::vector<KnownPassageSolidVolume> structure_volumes =
        knownPassageSolidVolumes(structure);
    volumes.insert(volumes.end(), structure_volumes.begin(), structure_volumes.end());
  }
  return volumes;
}

} // namespace drone_city_nav

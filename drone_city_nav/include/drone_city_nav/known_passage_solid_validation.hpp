#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_passage_solid_volumes.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <span>
#include <string>

namespace drone_city_nav {

enum class KnownPassageSolidValidationReason {
  kNoMap,
  kInvalidTrajectory,
  kClear,
  kIntersection,
};

struct KnownPassageSolidIntersection {
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  KnownPassageSolidPartKind part_kind{KnownPassageSolidPartKind::kLeft};
  std::size_t segment_index{0U};
  double segment_t{0.0};
  double s_m{0.0};
  Point3 point{};
};

struct KnownPassageSolidValidationSummary {
  bool valid{true};
  KnownPassageSolidValidationReason reason{KnownPassageSolidValidationReason::kNoMap};
  std::size_t volumes_checked{0U};
  std::size_t segments_checked{0U};
  std::size_t intersections{0U};
  bool has_first_intersection{false};
  KnownPassageSolidIntersection first_intersection{};
};

[[nodiscard]] const char* knownPassageSolidValidationReasonName(
    KnownPassageSolidValidationReason reason) noexcept;

[[nodiscard]] KnownPassageSolidValidationSummary
validateTrajectoryAgainstKnownPassageSolids(
    std::span<const TrajectoryPointSample> samples,
    const KnownPassageMap* known_passage_map);

} // namespace drone_city_nav

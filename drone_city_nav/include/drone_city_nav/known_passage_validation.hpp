#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class KnownPassageValidationReason {
  kDisabled,
  kNoMap,
  kInvalidTrajectory,
  kNoStructureIntersection,
  kMatchedOpening,
  kPartialFromInside,
  kStructureWithoutOpening,
  kOpeningVolumeMiss,
};

struct KnownPassageValidationConfig {
  bool enabled{true};
  double min_opening_overlap_m{0.5};
  double min_opening_depth_fraction{0.0};
  double clearance_margin_m{0.0};
  std::size_t max_diagnostics{8U};
};

struct KnownPassageValidationSpan {
  std::string structure_id;
  std::string opening_id;
  double entry_s_m{0.0};
  double exit_s_m{0.0};
  double overlap_m{0.0};
  double clearance_m{0.0};
  KnownPassageValidationReason reason{
      KnownPassageValidationReason::kNoStructureIntersection};
  bool starts_inside_opening{false};
  bool valid{false};
};

struct KnownPassageValidationSummary {
  bool enabled{false};
  bool valid{true};
  std::size_t structures_checked{0U};
  std::size_t structures_intersected{0U};
  std::size_t opening_matches{0U};
  std::size_t violations{0U};
  KnownPassageValidationReason worst_reason{
      KnownPassageValidationReason::kNoStructureIntersection};
  std::vector<KnownPassageValidationSpan> diagnostics;
};

[[nodiscard]] const char*
knownPassageValidationReasonName(KnownPassageValidationReason reason) noexcept;

[[nodiscard]] KnownPassageValidationSummary
validateKnownPassageTraversal(std::span<const TrajectoryPointSample> samples,
                              const KnownPassageMap* map,
                              const KnownPassageValidationConfig& config);

} // namespace drone_city_nav

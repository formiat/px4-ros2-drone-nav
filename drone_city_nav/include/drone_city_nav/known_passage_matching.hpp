#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class KnownPassageValidationReason;
struct KnownPassageValidationConfig;

struct KnownPassageTraversalMatch {
  std::string structure_id;
  std::string opening_id;
  PassageOpening opening;
  double entry_s_m{0.0};
  double exit_s_m{0.0};
  double overlap_m{0.0};
  double clearance_m{0.0};
  KnownPassageValidationReason reason;
  bool valid{false};
};

[[nodiscard]] std::vector<KnownPassageTraversalMatch> findKnownPassageTraversalMatches(
    std::span<const TrajectoryPointSample> samples, const KnownPassageMap& map,
    const KnownPassageValidationConfig& config, bool ignore_altitude = false);

} // namespace drone_city_nav

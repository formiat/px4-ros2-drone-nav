#include "drone_city_nav/known_passage_validation.hpp"

#include "drone_city_nav/known_passage_matching.hpp"

#include <algorithm>

namespace drone_city_nav {
namespace {

void appendDiagnostic(KnownPassageValidationSummary& summary,
                      const KnownPassageValidationConfig& config,
                      const KnownPassageTraversalMatch& match) {
  if (summary.diagnostics.size() >= config.max_diagnostics) {
    return;
  }
  summary.diagnostics.push_back(KnownPassageValidationSpan{
      .structure_id = match.structure_id,
      .opening_id = match.opening_id,
      .entry_s_m = match.entry_s_m,
      .exit_s_m = match.exit_s_m,
      .overlap_m = match.overlap_m,
      .clearance_m = match.clearance_m,
      .reason = match.reason,
      .valid = match.valid,
  });
}

void setWorstReason(KnownPassageValidationSummary& summary,
                    const KnownPassageValidationReason reason) noexcept {
  if (summary.valid) {
    summary.worst_reason = reason;
    return;
  }
  if (summary.worst_reason == KnownPassageValidationReason::kMatchedOpening ||
      summary.worst_reason == KnownPassageValidationReason::kNoStructureIntersection) {
    summary.worst_reason = reason;
  }
}

[[nodiscard]] std::size_t
countIntersectedStructures(const std::vector<KnownPassageTraversalMatch>& matches) {
  std::vector<std::string> structure_ids;
  structure_ids.reserve(matches.size());
  for (const KnownPassageTraversalMatch& match : matches) {
    if (std::ranges::find(structure_ids, match.structure_id) == structure_ids.end()) {
      structure_ids.push_back(match.structure_id);
    }
  }
  return structure_ids.size();
}

} // namespace

const char*
knownPassageValidationReasonName(const KnownPassageValidationReason reason) noexcept {
  switch (reason) {
    case KnownPassageValidationReason::kDisabled:
      return "disabled";
    case KnownPassageValidationReason::kNoMap:
      return "no_map";
    case KnownPassageValidationReason::kInvalidTrajectory:
      return "invalid_trajectory";
    case KnownPassageValidationReason::kNoStructureIntersection:
      return "no_structure_intersection";
    case KnownPassageValidationReason::kMatchedOpening:
      return "matched_opening";
    case KnownPassageValidationReason::kStructureWithoutOpening:
      return "structure_without_opening";
    case KnownPassageValidationReason::kOpeningVolumeMiss:
      return "opening_volume_miss";
  }
  return "unknown";
}

KnownPassageValidationSummary
validateKnownPassageTraversal(const std::span<const TrajectoryPointSample> samples,
                              const KnownPassageMap* const map,
                              const KnownPassageValidationConfig& config) {
  KnownPassageValidationSummary summary{};
  summary.enabled = config.enabled;

  if (!config.enabled) {
    summary.worst_reason = KnownPassageValidationReason::kDisabled;
    return summary;
  }
  if (map == nullptr) {
    summary.worst_reason = KnownPassageValidationReason::kNoMap;
    return summary;
  }
  summary.structures_checked = map->structures.size();
  if (!trajectorySamplesAreUsable(samples)) {
    summary.valid = false;
    summary.violations = 1U;
    summary.worst_reason = KnownPassageValidationReason::kInvalidTrajectory;
    return summary;
  }

  const std::vector<KnownPassageTraversalMatch> matches =
      findKnownPassageTraversalMatches(samples, *map, config);
  summary.structures_intersected = countIntersectedStructures(matches);

  bool had_match = false;
  for (const KnownPassageTraversalMatch& match : matches) {
    appendDiagnostic(summary, config, match);
    if (match.valid) {
      had_match = true;
      ++summary.opening_matches;
      continue;
    }
    summary.valid = false;
    ++summary.violations;
    setWorstReason(summary, match.reason);
  }

  if (summary.valid) {
    summary.worst_reason = had_match
                               ? KnownPassageValidationReason::kMatchedOpening
                               : KnownPassageValidationReason::kNoStructureIntersection;
  } else if (matches.empty()) {
    summary.worst_reason = KnownPassageValidationReason::kNoStructureIntersection;
  }
  return summary;
}

} // namespace drone_city_nav

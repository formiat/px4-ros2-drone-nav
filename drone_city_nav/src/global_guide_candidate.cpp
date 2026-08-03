#include "drone_city_nav/global_guide_candidate.hpp"

#include <tuple>

namespace drone_city_nav {
namespace {

[[nodiscard]] int statusRank(const LatticePlanStatus status) noexcept {
  switch (status) {
    case LatticePlanStatus::kReachedPlanningGoal:
      return 0;
    case LatticePlanStatus::kViableFrontier:
      return 1;
    case LatticePlanStatus::kRawSafeDetourPrefix:
      return 2;
    case LatticePlanStatus::kSearchIncomplete:
    case LatticePlanStatus::kMotionGraphExhausted:
    case LatticePlanStatus::kInvalidInput:
      return 3;
  }
  return 3;
}

} // namespace

bool betterGlobalGuideCandidate(const GlobalGuideCandidate& candidate,
                                const GlobalGuideCandidate& current) noexcept {
  return std::tuple{statusRank(candidate.status),
                    -candidate.endpoint_displacement_m,
                    -candidate.reachable_depth_m,
                    candidate.guide_length_m,
                    candidate.remaining_goal_distance_m,
                    candidate.cost,
                    candidate.fingerprint} <
         std::tuple{statusRank(current.status),
                    -current.endpoint_displacement_m,
                    -current.reachable_depth_m,
                    current.guide_length_m,
                    current.remaining_goal_distance_m,
                    current.cost,
                    current.fingerprint};
}

bool globalGuideCandidateReadyForActivation(
    const GlobalGuideCandidate& candidate, const bool allow_raw_safe_prefix,
    const double minimum_raw_prefix_length_m,
    const double minimum_raw_prefix_endpoint_displacement_m) noexcept {
  if (candidate.status == LatticePlanStatus::kReachedPlanningGoal ||
      (candidate.status == LatticePlanStatus::kViableFrontier &&
       candidate.guide_length_m >= minimum_raw_prefix_length_m &&
       candidate.endpoint_displacement_m >=
           minimum_raw_prefix_endpoint_displacement_m)) {
    return true;
  }
  return allow_raw_safe_prefix &&
         candidate.status == LatticePlanStatus::kRawSafeDetourPrefix &&
         candidate.guide_length_m >= minimum_raw_prefix_length_m &&
         candidate.endpoint_displacement_m >=
             minimum_raw_prefix_endpoint_displacement_m;
}

} // namespace drone_city_nav

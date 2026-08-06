#pragma once

#include "drone_city_nav/risk_aware_lattice.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct GlobalGuideCandidate {
  std::shared_ptr<const std::vector<Point2>> guide;
  std::uint64_t base_generation{0U};
  std::uint64_t search_revision{0U};
  Point3 objective_goal{};
  std::uint64_t objective_mission_epoch{0U};
  std::uint64_t objective_sample_sequence{0U};
  bool objective_continuous_tracking{false};
  bool objective_available{false};
  bool reaches_mission_goal{false};
  LatticePlanStatus status{LatticePlanStatus::kInvalidInput};
  std::size_t expansions{0U};
  double guide_length_m{0.0};
  double endpoint_displacement_m{0.0};
  double reachable_depth_m{0.0};
  double remaining_goal_distance_m{0.0};
  double cost{0.0};
  std::uint64_t fingerprint{0U};
};

[[nodiscard]] bool
betterGlobalGuideCandidate(const GlobalGuideCandidate& candidate,
                           const GlobalGuideCandidate& current) noexcept;

[[nodiscard]] bool globalGuideCandidateReadyForActivation(
    const GlobalGuideCandidate& candidate, bool allow_raw_safe_prefix,
    double minimum_raw_prefix_length_m,
    double minimum_raw_prefix_endpoint_displacement_m) noexcept;

} // namespace drone_city_nav

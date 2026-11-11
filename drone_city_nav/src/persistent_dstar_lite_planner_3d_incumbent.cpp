#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

// The incumbent route carried over to the start of a newer update.

namespace drone_city_nav::detail {

std::optional<std::vector<Point3>>
PersistentDStarLitePlanner3DImpl::rebaseIncumbent(const std::vector<Point3>& incumbent,
                                                  const Point3& start,
                                                  const Point3& goal) const {
  if (incumbent.size() < 2U ||
      distance3D(incumbent.back(), goal) > config_.goal_tolerance_m) {
    return std::nullopt;
  }
  std::vector<std::size_t> candidates(incumbent.size());
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    candidates[index] = index;
  }
  std::ranges::sort(candidates, [&](const std::size_t first, const std::size_t second) {
    return distance3D(start, incumbent[first]) < distance3D(start, incumbent[second]);
  });
  for (const std::size_t candidate : candidates) {
    if (!lattice_.departureSegmentValid(start, incumbent[candidate])) {
      continue;
    }
    std::vector<Point3> rebased;
    rebased.reserve(incumbent.size() - candidate + 1U);
    rebased.push_back(start);
    if (distance3D(start, incumbent[candidate]) > 1.0e-9) {
      rebased.push_back(incumbent[candidate]);
    }
    rebased.insert(
        rebased.end(),
        std::next(incumbent.begin(), static_cast<std::ptrdiff_t>(candidate + 1U)),
        incumbent.end());
    if (rebased.size() >= 2U && lattice_.pathTraversable(rebased)) {
      return rebased;
    }
  }
  return std::nullopt;
}

} // namespace drone_city_nav::detail

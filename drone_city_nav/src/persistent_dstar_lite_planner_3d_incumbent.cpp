#include <algorithm>
#include <cstddef>
#include <optional>
#include <utility>
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
  // The joins are the vertices and the nearest point of every segment, each
  // followed by the vertices after it. The open sky's leg has four vertices
  // hundreds of metres apart: the nearest of them lay 88 m behind a vehicle on
  // the leg's level, and the route rebased there flew back to it and along the
  // same line forward again (r3280, r3288; 164 m in r3287).
  std::vector<std::pair<Point3, std::size_t>> joins;
  joins.reserve(2U * incumbent.size());
  for (std::size_t index = 0U; index < incumbent.size(); ++index) {
    joins.emplace_back(incumbent[index], index + 1U);
    if (index + 1U == incumbent.size()) {
      continue;
    }
    const Point3& from = incumbent[index];
    const Point3& to = incumbent[index + 1U];
    const Point3 along{to.x - from.x, to.y - from.y, to.z - from.z};
    const double length_sq = along.x * along.x + along.y * along.y + along.z * along.z;
    if (length_sq > 1.0e-12) {
      const double fraction =
          std::clamp(((start.x - from.x) * along.x + (start.y - from.y) * along.y +
                      (start.z - from.z) * along.z) /
                         length_sq,
                     0.0, 1.0);
      joins.emplace_back(Point3{from.x + fraction * along.x,
                                from.y + fraction * along.y,
                                from.z + fraction * along.z},
                         index + 1U);
    }
  }
  std::ranges::stable_sort(
      joins, {}, [&start](const auto& join) { return distance3D(start, join.first); });
  for (const auto& [join, next] : joins) {
    if (!lattice_.departureSegmentValid(start, join)) {
      continue;
    }
    std::vector<Point3> rebased{start};
    if (distance3D(start, join) > 1.0e-9) {
      rebased.push_back(join);
    }
    for (std::size_t index = next; index < incumbent.size(); ++index) {
      if (distance3D(rebased.back(), incumbent[index]) > 1.0e-9) {
        rebased.push_back(incumbent[index]);
      }
    }
    if (rebased.size() >= 2U && lattice_.pathTraversable(rebased)) {
      return rebased;
    }
  }
  return std::nullopt;
}

} // namespace drone_city_nav::detail

#include "risk_aware_lattice_3d_passage_index.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <tuple>

namespace drone_city_nav::detail {

PassageEntrySpatialIndex::PassageEntrySpatialIndex(
    const std::span<const PassageTraversalEdge> passages, const double bucket_size_m)
    : bucket_size_m_{bucket_size_m} {
  if (!std::isfinite(bucket_size_m_) || !(bucket_size_m_ > 0.0)) {
    throw std::invalid_argument{"passage entry bucket size must be positive"};
  }
  for (std::size_t index = 0U; index < passages.size(); ++index) {
    entries_[bucketFor(passages[index].entry)].push_back(
        OrientedPassageCandidate{.passage_index = index, .reversed = false});
    entries_[bucketFor(passages[index].exit)].push_back(
        OrientedPassageCandidate{.passage_index = index, .reversed = true});
  }
}

std::vector<OrientedPassageCandidate>
PassageEntrySpatialIndex::near(const Point3& point) const {
  const Bucket center = bucketFor(point);
  std::vector<OrientedPassageCandidate> result;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const auto found = entries_.find(
            Bucket{.x = center.x + dx, .y = center.y + dy, .z = center.z + dz});
        if (found != entries_.end()) {
          result.insert(result.end(), found->second.begin(), found->second.end());
        }
      }
    }
  }
  std::ranges::sort(result, [](const OrientedPassageCandidate& first,
                               const OrientedPassageCandidate& second) {
    return std::tie(first.passage_index, first.reversed) <
           std::tie(second.passage_index, second.reversed);
  });
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

PassageEntrySpatialIndex::Bucket
PassageEntrySpatialIndex::bucketFor(const Point3& point) const noexcept {
  return Bucket{
      .x = static_cast<int>(std::floor(point.x / bucket_size_m_)),
      .y = static_cast<int>(std::floor(point.y / bucket_size_m_)),
      .z = static_cast<int>(std::floor(point.z / bucket_size_m_)),
  };
}

} // namespace drone_city_nav::detail

#pragma once

#include "drone_city_nav/portal_graph.hpp"

#include <cstddef>
#include <set>
#include <vector>

namespace drone_city_nav::topology_extractor_detail {

struct NavigablePassageGraph {
  std::vector<PassageSegment> segments;
  std::set<PassagePortalId> attached_portal_ids;
  std::size_t pruned_segment_count{0U};
};

[[nodiscard]] NavigablePassageGraph
retainNavigableSegmentComponents(std::vector<PassageSegment> segments);

} // namespace drone_city_nav::topology_extractor_detail

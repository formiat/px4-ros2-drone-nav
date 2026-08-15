#include "free_space_topology_extractor_3d_graph.hpp"

#include <map>
#include <stdexcept>
#include <utility>

namespace drone_city_nav::topology_extractor_detail {

NavigablePassageGraph
retainNavigableSegmentComponents(std::vector<PassageSegment> segments) {
  std::map<PassageSegmentId, std::size_t> index_by_id;
  for (std::size_t index = 0U; index < segments.size(); ++index) {
    if (!index_by_id.emplace(segments[index].id, index).second) {
      throw std::logic_error{"duplicate extracted passage segment id"};
    }
  }

  std::vector<bool> visited(segments.size(), false);
  std::vector<bool> retained(segments.size(), false);
  std::set<PassagePortalId> attached_portal_ids;
  for (std::size_t root = 0U; root < segments.size(); ++root) {
    if (visited[root]) {
      continue;
    }
    std::vector<std::size_t> component;
    std::vector<std::size_t> pending{root};
    std::set<PassagePortalId> component_portals;
    visited[root] = true;
    while (!pending.empty()) {
      const std::size_t current = pending.back();
      pending.pop_back();
      component.push_back(current);
      const PassageSegment& segment = segments[current];
      component_portals.insert(segment.endpoint_portal_ids.begin(),
                               segment.endpoint_portal_ids.end());
      const auto enqueue = [&](const PassageSegmentId& neighbor_id) {
        const auto found = index_by_id.find(neighbor_id);
        if (found == index_by_id.end()) {
          throw std::logic_error{"unknown extracted passage segment neighbor"};
        }
        if (!visited[found->second]) {
          visited[found->second] = true;
          pending.push_back(found->second);
        }
      };
      for (const PassageSegmentId& neighbor : segment.first_endpoint_neighbors) {
        enqueue(neighbor);
      }
      for (const PassageSegmentId& neighbor : segment.second_endpoint_neighbors) {
        enqueue(neighbor);
      }
    }
    if (component_portals.size() < 2U) {
      continue;
    }
    attached_portal_ids.insert(component_portals.begin(), component_portals.end());
    for (const std::size_t index : component) {
      retained[index] = true;
    }
  }

  NavigablePassageGraph result;
  result.attached_portal_ids = std::move(attached_portal_ids);
  result.pruned_segment_count = segments.size();
  result.segments.reserve(segments.size());
  for (std::size_t index = 0U; index < segments.size(); ++index) {
    if (retained[index]) {
      result.segments.push_back(std::move(segments[index]));
    }
  }
  result.pruned_segment_count -= result.segments.size();
  return result;
}

} // namespace drone_city_nav::topology_extractor_detail

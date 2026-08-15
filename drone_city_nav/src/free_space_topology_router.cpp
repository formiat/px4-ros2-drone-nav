#include "drone_city_nav/free_space_topology_router.hpp"

#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

struct EndpointKey {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] auto operator<=>(const EndpointKey&) const noexcept = default;
};

struct SegmentArc {
  std::size_t destination_node{0U};
  std::size_t segment_index{0U};
  bool forward{true};
  double length_m{0.0};
};

struct PortalEndpoint {
  std::size_t portal_index{0U};
  std::size_t node_index{0U};
};

struct ComponentGraph {
  std::vector<std::size_t> node_indices;
  std::vector<PortalEndpoint> portals;
};

struct Predecessor {
  std::size_t node_index{0U};
  std::size_t segment_index{0U};
  bool forward{true};
  bool available{false};
};

struct ShortestPaths {
  std::vector<double> distances;
  std::vector<Predecessor> predecessors;
};

struct TraversalCandidate {
  std::size_t entry_portal_index{0U};
  std::size_t exit_portal_index{0U};
  double objective_m{0.0};
  std::shared_ptr<const ShortestPaths> paths;
};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double
centerlineLength(const std::vector<RouteSample3D>& centerline) noexcept {
  double result = 0.0;
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    result += distance3D(centerline[index - 1U].position, centerline[index].position);
  }
  return result;
}

[[nodiscard]] EndpointKey endpointKey(const Point3& point,
                                      const GridBounds3D& bounds) noexcept {
  return EndpointKey{
      .x = static_cast<int>(
          std::llround((point.x - bounds.origin_x) / bounds.resolution_m - 0.5)),
      .y = static_cast<int>(
          std::llround((point.y - bounds.origin_y) / bounds.resolution_m - 0.5)),
      .z = static_cast<int>(
          std::llround((point.z - bounds.origin_z) / bounds.resolution_m - 0.5)),
  };
}

void appendCenterline(std::vector<Point3>& points, const PassageSegment& segment,
                      const bool forward) {
  const auto append = [&points](const Point3& point) {
    if (points.empty() || distance3D(points.back(), point) > 1.0e-9) {
      points.push_back(point);
    }
  };
  if (forward) {
    for (const RouteSample3D& sample : segment.centerline) {
      append(sample.position);
    }
    return;
  }
  for (const RouteSample3D& sample : std::views::reverse(segment.centerline)) {
    append(sample.position);
  }
}

[[nodiscard]] std::string traversalId(const PassagePortal& entry,
                                      const PassagePortal& exit,
                                      const std::vector<PassageSegmentId>& segments) {
  std::uint64_t hash = UINT64_C(1469598103934665603);
  const auto add = [&hash](const std::string& value) {
    for (const char character : value) {
      hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(character));
      hash *= UINT64_C(1099511628211);
    }
    hash ^= UINT64_C(0xff);
    hash *= UINT64_C(1099511628211);
  };
  add(entry.id.value());
  add(exit.id.value());
  for (const PassageSegmentId& segment_id : segments) {
    add(segment_id.value());
  }
  return "traversal:" + std::to_string(hash);
}

} // namespace

class FreeSpaceTopologyRouter::Impl {
public:
  Impl(const FreeSpaceTopology3D& topology, const FreeSpaceTopologyRouterConfig& config)
      : topology_{topology},
        config_{config} {
    if (config_.maximum_entry_portals_per_component == 0U ||
        config_.maximum_traversals_per_component == 0U) {
      throw std::invalid_argument{"invalid FreeSpaceTopologyRouter config"};
    }
    if (!topology_.segments().empty()) {
      buildGraph();
    }
  }

  [[nodiscard]] FreeSpaceTopologyRoute resolve(const Point3& start,
                                               const Point3& goal) const {
    if (!finitePoint(start) || !finitePoint(goal)) {
      return {};
    }
    if (topology_.segments().empty()) {
      return FreeSpaceTopologyRoute{
          .traversals = std::make_shared<const std::vector<PassageTraversalEdge>>(
              topology_.traversalEdges()),
      };
    }

    FreeSpaceTopologyRoute result;
    result.stats.component_count = components_.size();
    std::vector<PassageTraversalEdge> traversals;
    for (const ComponentGraph& component : components_) {
      if (component.portals.size() < 2U) {
        continue;
      }
      std::vector<PortalEndpoint> entries = component.portals;
      std::ranges::sort(entries, [&](const PortalEndpoint& first,
                                     const PortalEndpoint& second) {
        const PassagePortal& first_portal = topology_.portals()[first.portal_index];
        const PassagePortal& second_portal = topology_.portals()[second.portal_index];
        return std::tuple{distance3D(start, first_portal.center), first_portal.id} <
               std::tuple{distance3D(start, second_portal.center), second_portal.id};
      });
      entries.resize(
          std::min(entries.size(), config_.maximum_entry_portals_per_component));

      std::vector<TraversalCandidate> candidates;
      for (const PortalEndpoint& entry : entries) {
        ++result.stats.entry_search_count;
        const auto paths =
            std::make_shared<const ShortestPaths>(shortestPaths(entry.node_index));
        for (const PortalEndpoint& exit : component.portals) {
          if (entry.portal_index == exit.portal_index ||
              entry.node_index == exit.node_index ||
              !std::isfinite(paths->distances[exit.node_index])) {
            continue;
          }
          ++result.stats.evaluated_portal_pairs;
          const PassagePortal& entry_portal = topology_.portals()[entry.portal_index];
          const PassagePortal& exit_portal = topology_.portals()[exit.portal_index];
          candidates.push_back(TraversalCandidate{
              .entry_portal_index = entry.portal_index,
              .exit_portal_index = exit.portal_index,
              .objective_m = distance3D(start, entry_portal.center) +
                             paths->distances[exit.node_index] +
                             distance3D(exit_portal.center, goal),
              .paths = paths,
          });
        }
      }
      std::ranges::sort(candidates, [&](const TraversalCandidate& first,
                                        const TraversalCandidate& second) {
        const PassagePortal& first_entry =
            topology_.portals()[first.entry_portal_index];
        const PassagePortal& first_exit = topology_.portals()[first.exit_portal_index];
        const PassagePortal& second_entry =
            topology_.portals()[second.entry_portal_index];
        const PassagePortal& second_exit =
            topology_.portals()[second.exit_portal_index];
        return std::tie(first.objective_m, first_entry.id, first_exit.id) <
               std::tie(second.objective_m, second_entry.id, second_exit.id);
      });

      std::set<std::pair<PassagePortalId, PassagePortalId>> selected_pairs;
      std::size_t selected = 0U;
      for (const TraversalCandidate& candidate : candidates) {
        const PassagePortal& entry = topology_.portals()[candidate.entry_portal_index];
        const PassagePortal& exit = topology_.portals()[candidate.exit_portal_index];
        const auto pair = std::minmax(entry.id, exit.id);
        if (!selected_pairs.emplace(pair.first, pair.second).second) {
          continue;
        }
        const auto [traversal, cache_hit] = resolveTraversal(candidate);
        if (!traversal.has_value()) {
          continue;
        }
        cache_hit ? ++result.stats.traversal_cache_hits
                  : ++result.stats.traversal_cache_misses;
        traversals.push_back(*traversal);
        ++selected;
        if (selected >= config_.maximum_traversals_per_component) {
          break;
        }
      }
    }
    result.traversals = std::make_shared<const std::vector<PassageTraversalEdge>>(
        std::move(traversals));
    return result;
  }

private:
  void buildGraph() {
    const GridBounds3D& bounds = topology_.occupancyBounds();
    std::map<EndpointKey, std::size_t> node_by_key;
    const auto node_for = [&](const Point3& point) {
      const EndpointKey key = endpointKey(point, bounds);
      const auto [found, inserted] = node_by_key.emplace(key, node_by_key.size());
      if (inserted) {
        graph_.emplace_back();
      }
      return found->second;
    };

    for (std::size_t index = 0U; index < topology_.segments().size(); ++index) {
      const PassageSegment& segment = topology_.segments()[index];
      const std::size_t first = node_for(segment.centerline.front().position);
      const std::size_t second = node_for(segment.centerline.back().position);
      const double length_m = centerlineLength(segment.centerline);
      graph_[first].push_back(SegmentArc{.destination_node = second,
                                         .segment_index = index,
                                         .forward = true,
                                         .length_m = length_m});
      graph_[second].push_back(SegmentArc{.destination_node = first,
                                          .segment_index = index,
                                          .forward = false,
                                          .length_m = length_m});
    }
    for (std::vector<SegmentArc>& arcs : graph_) {
      std::ranges::sort(arcs, [&](const SegmentArc& first, const SegmentArc& second) {
        return topology_.segments()[first.segment_index].id <
               topology_.segments()[second.segment_index].id;
      });
    }

    std::vector<std::size_t> component_by_node(graph_.size(),
                                               std::numeric_limits<std::size_t>::max());
    for (std::size_t root = 0U; root < graph_.size(); ++root) {
      if (component_by_node[root] != std::numeric_limits<std::size_t>::max()) {
        continue;
      }
      const std::size_t component_index = components_.size();
      components_.push_back(ComponentGraph{});
      std::queue<std::size_t> pending;
      pending.push(root);
      component_by_node[root] = component_index;
      while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop();
        components_.back().node_indices.push_back(current);
        for (const SegmentArc& arc : graph_[current]) {
          if (component_by_node[arc.destination_node] ==
              std::numeric_limits<std::size_t>::max()) {
            component_by_node[arc.destination_node] = component_index;
            pending.push(arc.destination_node);
          }
        }
      }
    }

    for (std::size_t portal_index = 0U; portal_index < topology_.portals().size();
         ++portal_index) {
      const PassagePortal& portal = topology_.portals()[portal_index];
      const auto found = node_by_key.find(endpointKey(portal.center, bounds));
      if (found == node_by_key.end()) {
        throw std::invalid_argument{"passage portal is not attached to sparse graph"};
      }
      const std::size_t component_index = component_by_node[found->second];
      components_[component_index].portals.push_back(
          PortalEndpoint{.portal_index = portal_index, .node_index = found->second});
      portal_node_indices_.push_back(found->second);
    }
  }

  [[nodiscard]] ShortestPaths shortestPaths(const std::size_t source) const {
    ShortestPaths result{
        .distances =
            std::vector<double>(graph_.size(), std::numeric_limits<double>::infinity()),
        .predecessors = std::vector<Predecessor>(graph_.size()),
    };
    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pending;
    result.distances[source] = 0.0;
    pending.emplace(0.0, source);
    while (!pending.empty()) {
      const auto [distance_m, node_index] = pending.top();
      pending.pop();
      if (distance_m > result.distances[node_index] + 1.0e-9) {
        continue;
      }
      for (const SegmentArc& arc : graph_[node_index]) {
        const double candidate_m = distance_m + arc.length_m;
        if (candidate_m + 1.0e-9 >= result.distances[arc.destination_node]) {
          continue;
        }
        result.distances[arc.destination_node] = candidate_m;
        result.predecessors[arc.destination_node] =
            Predecessor{.node_index = node_index,
                        .segment_index = arc.segment_index,
                        .forward = arc.forward,
                        .available = true};
        pending.emplace(candidate_m, arc.destination_node);
      }
    }
    return result;
  }

  [[nodiscard]] std::pair<std::optional<PassageTraversalEdge>, bool>
  resolveTraversal(const TraversalCandidate& candidate) const {
    const PassagePortal& entry = topology_.portals()[candidate.entry_portal_index];
    const PassagePortal& exit = topology_.portals()[candidate.exit_portal_index];
    const std::pair<std::string, std::string> cache_key{entry.id.value(),
                                                        exit.id.value()};
    {
      const std::scoped_lock lock{cache_mutex_};
      if (const auto found = traversal_cache_.find(cache_key);
          found != traversal_cache_.end()) {
        return {found->second, true};
      }
    }

    std::vector<Predecessor> path;
    std::size_t current = portal_node_indices_[candidate.exit_portal_index];
    const std::size_t source = portal_node_indices_[candidate.entry_portal_index];
    while (current != source) {
      const Predecessor predecessor = candidate.paths->predecessors[current];
      if (!predecessor.available) {
        return {std::nullopt, false};
      }
      path.push_back(predecessor);
      current = predecessor.node_index;
    }
    std::ranges::reverse(path);
    if (path.empty()) {
      return {std::nullopt, false};
    }

    std::vector<Point3> points;
    std::vector<PassageSegmentId> segment_ids;
    double minimum_clearance_m = std::numeric_limits<double>::infinity();
    double speed_limit_mps = std::numeric_limits<double>::infinity();
    for (const Predecessor& step : path) {
      const PassageSegment& segment = topology_.segments()[step.segment_index];
      appendCenterline(points, segment, step.forward);
      segment_ids.push_back(segment.id);
      minimum_clearance_m = std::min(minimum_clearance_m, segment.minimum_clearance_m);
      speed_limit_mps = std::min(speed_limit_mps, segment.speed_limit_mps);
    }
    if (points.size() < 2U || !std::isfinite(minimum_clearance_m) ||
        !(minimum_clearance_m > 0.0) || !std::isfinite(speed_limit_mps) ||
        !(speed_limit_mps > 0.0)) {
      return {std::nullopt, false};
    }
    std::vector<RouteSample3D> centerline = sampleRoute3D(
        points, topology_.occupancyBounds().resolution_m, speed_limit_mps);
    const auto [minimum_z, maximum_z] = std::ranges::minmax(
        centerline, {}, [](const RouteSample3D& sample) { return sample.position.z; });
    PassageTraversalEdge traversal{
        .id = PassageTraversalId{traversalId(entry, exit, segment_ids)},
        .region_id = entry.region_id,
        .entry_portal_id = entry.id,
        .exit_portal_id = exit.id,
        .centerline = std::move(centerline),
        .entry = entry.center,
        .exit = exit.center,
        .min_z_m = minimum_z.position.z - minimum_clearance_m,
        .max_z_m = maximum_z.position.z + minimum_clearance_m,
        .width_m = 2.0 * minimum_clearance_m,
        .height_m =
            maximum_z.position.z - minimum_z.position.z + 2.0 * minimum_clearance_m,
        .minimum_clearance_m = minimum_clearance_m,
        .speed_limit_mps = speed_limit_mps,
        .segment_ids = std::move(segment_ids),
    };
    {
      const std::scoped_lock lock{cache_mutex_};
      traversal_cache_.emplace(cache_key, traversal);
    }
    return {std::move(traversal), false};
  }

  const FreeSpaceTopology3D& topology_;
  FreeSpaceTopologyRouterConfig config_;
  std::vector<std::vector<SegmentArc>> graph_;
  std::vector<ComponentGraph> components_;
  std::vector<std::size_t> portal_node_indices_;
  mutable std::mutex cache_mutex_;
  mutable std::map<std::pair<std::string, std::string>, PassageTraversalEdge>
      traversal_cache_;
};

FreeSpaceTopologyRouter::FreeSpaceTopologyRouter(
    const FreeSpaceTopology3D& topology, const FreeSpaceTopologyRouterConfig& config)
    : impl_{std::make_unique<Impl>(topology, config)} {
}

FreeSpaceTopologyRouter::~FreeSpaceTopologyRouter() = default;

FreeSpaceTopologyRouter::FreeSpaceTopologyRouter(FreeSpaceTopologyRouter&&) noexcept =
    default;

FreeSpaceTopologyRouter&
FreeSpaceTopologyRouter::operator=(FreeSpaceTopologyRouter&&) noexcept = default;

FreeSpaceTopologyRoute FreeSpaceTopologyRouter::resolve(const Point3& start,
                                                        const Point3& goal) const {
  return impl_->resolve(start, goal);
}

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct FreeSpaceTopologyRouterConfig {
  std::size_t maximum_entry_portals_per_component{4U};
  std::size_t maximum_traversals_per_component{3U};
};

struct FreeSpaceTopologyRouteStats {
  std::size_t component_count{0U};
  std::size_t entry_search_count{0U};
  std::size_t evaluated_portal_pairs{0U};
  std::size_t traversal_cache_hits{0U};
  std::size_t traversal_cache_misses{0U};
};

struct FreeSpaceTopologyRoute {
  std::shared_ptr<const std::vector<PassageTraversalEdge>> traversals;
  FreeSpaceTopologyRouteStats stats{};
};

class FreeSpaceTopologyRouter {
public:
  explicit FreeSpaceTopologyRouter(const FreeSpaceTopology3D& topology,
                                   const FreeSpaceTopologyRouterConfig& config = {});
  ~FreeSpaceTopologyRouter();

  FreeSpaceTopologyRouter(const FreeSpaceTopologyRouter&) = delete;
  FreeSpaceTopologyRouter& operator=(const FreeSpaceTopologyRouter&) = delete;
  FreeSpaceTopologyRouter(FreeSpaceTopologyRouter&&) noexcept;
  FreeSpaceTopologyRouter& operator=(FreeSpaceTopologyRouter&&) noexcept;

  [[nodiscard]] FreeSpaceTopologyRoute resolve(const Point3& start,
                                               const Point3& goal) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav

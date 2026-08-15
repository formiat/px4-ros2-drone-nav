#pragma once

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

struct FreeSpaceTopologyFormatLimits final {
  FreeSpaceTopologyFormatLimits() = delete;

  static constexpr std::uint32_t maximum_region_count{10000U};
  static constexpr std::uint32_t maximum_portal_count{100000U};
  static constexpr std::uint32_t maximum_segment_count{1000000U};
  static constexpr std::uint32_t maximum_traversal_edge_count{100000U};
  static constexpr std::uint32_t maximum_geometry_point_count{100000U};
  static constexpr std::size_t maximum_total_geometry_point_count{10000000U};
};

} // namespace drone_city_nav

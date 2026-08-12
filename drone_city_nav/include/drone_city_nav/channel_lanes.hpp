#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct ChannelLaneConfig {
  double minimum_center_separation_m{5.0};
  double minimum_wall_clearance_m{1.0};
  std::size_t maximum_lane_count{5U};
  SweptFootprintConfig footprint{};
};

struct ChannelLane {
  std::size_t index{0U};
  double lateral_offset_m{0.0};
  std::vector<RouteSample3D> centerline;
  double minimum_wall_clearance_m{0.0};
};

struct ChannelLaneSet {
  std::string channel_id;
  double physical_width_m{0.0};
  double usable_center_width_m{0.0};
  std::vector<ChannelLane> lanes;

  [[nodiscard]] bool exclusive() const noexcept {
    return lanes.size() <= 1U;
  }
};

[[nodiscard]] std::vector<RouteSample3D>
offsetChannelCenterline(std::span<const RouteSample3D> centerline,
                        double lateral_offset_m);

[[nodiscard]] ChannelLaneSet
makeGeometricChannelLanes(const ConstrainedFreeSpaceEdge& channel,
                          const ChannelLaneConfig& config);

[[nodiscard]] ChannelLaneSet makeCollisionValidatedChannelLanes(
    const ConstrainedFreeSpaceEdge& channel, const ChannelLaneConfig& config,
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m);

[[nodiscard]] std::string channelConflictResourceId(std::string_view channel_id);

} // namespace drone_city_nav

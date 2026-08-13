#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct ChannelCorridorConfig {
  double desired_center_separation_m{5.0};
  double minimum_wall_clearance_m{1.0};
  double lateral_probe_step_m{0.5};
  double directional_offset_fraction{0.5};
  SweptFootprintConfig footprint{};
};

struct ChannelCorridor {
  std::string channel_id;
  double physical_width_m{0.0};
  double minimum_lateral_offset_m{0.0};
  double maximum_lateral_offset_m{0.0};
  double minimum_wall_clearance_m{0.0};
  bool raw_validated{false};

  [[nodiscard]] double usableWidthM() const noexcept {
    return maximum_lateral_offset_m - minimum_lateral_offset_m;
  }

  [[nodiscard]] bool exclusive(double required_separation_m) const noexcept {
    return usableWidthM() + 1.0e-9 < required_separation_m;
  }
};

struct ChannelCorridorResource {
  std::shared_ptr<const std::vector<ChannelCorridor>> corridors;
  bool shared_resource_reused{false};
};

[[nodiscard]] std::vector<RouteSample3D>
offsetChannelCenterline(std::span<const RouteSample3D> centerline,
                        double lateral_offset_m);

[[nodiscard]] ChannelCorridor
makeGeometricChannelCorridor(const ConstrainedFreeSpaceEdge& channel,
                             const ChannelCorridorConfig& config);

[[nodiscard]] ChannelCorridor
makeRawCollisionValidatedChannelCorridor(const ConstrainedFreeSpaceEdge& channel,
                                         const ChannelCorridorConfig& config,
                                         const OccupancyGrid3D& occupancy);

[[nodiscard]] ChannelCorridorResource
acquireRawValidatedChannelCorridors(std::span<const ConstrainedFreeSpaceEdge> channels,
                                    const ChannelCorridorConfig& config,
                                    const OccupancyGrid3D& occupancy);

[[nodiscard]] double
preferredDirectionalChannelOffset(const ChannelCorridor& corridor, int direction_sign,
                                  const ChannelCorridorConfig& config) noexcept;

[[nodiscard]] bool validateChannelOffset(const ConstrainedFreeSpaceEdge& channel,
                                         double lateral_offset_m,
                                         const ChannelCorridorConfig& config,
                                         const OccupancyGrid3D& occupancy) noexcept;

[[nodiscard]] std::string channelConflictResourceId(std::string_view channel_id);

} // namespace drone_city_nav

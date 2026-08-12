#include "drone_city_nav/channel_lanes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

struct Direction2 {
  double x{0.0};
  double y{0.0};
};

[[nodiscard]] Direction2 normalizedDirection(const Point3& first,
                                             const Point3& second) noexcept {
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double length = std::hypot(dx, dy);
  return length > 1.0e-9 ? Direction2{dx / length, dy / length} : Direction2{};
}

[[nodiscard]] Direction2 lateralLeft(const Direction2& direction) noexcept {
  return Direction2{-direction.y, direction.x};
}

[[nodiscard]] Vec3 normalized3D(const Point3& first, const Point3& second) noexcept {
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double dz = second.z - first.z;
  const double length = std::hypot(std::hypot(dx, dy), dz);
  return length > 1.0e-9 ? Vec3{dx / length, dy / length, dz / length} : Vec3{};
}

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] std::vector<double> laneOffsets(const std::size_t lane_count,
                                              const double spacing_m) {
  std::vector<double> offsets;
  offsets.reserve(lane_count);
  const double middle = 0.5 * static_cast<double>(lane_count - 1U);
  for (std::size_t index = 0U; index < lane_count; ++index) {
    offsets.push_back((static_cast<double>(index) - middle) * spacing_m);
  }
  return offsets;
}

[[nodiscard]] bool
laneInsideVerticalWindow(const ChannelLane& lane,
                         const ConstrainedFreeSpaceEdge& channel,
                         const SweptFootprintConfig& footprint) noexcept {
  return std::ranges::all_of(lane.centerline, [&](const RouteSample3D& sample) {
    return sample.position.z - footprint.lower_extent_m >= channel.min_z_m &&
           sample.position.z + footprint.upper_extent_m <= channel.max_z_m;
  });
}

[[nodiscard]] bool validateLane(ChannelLane& lane,
                                const ConstrainedFreeSpaceEdge& channel,
                                const ChannelLaneConfig& config,
                                const mppi::EsdfGrid& grid,
                                const std::span<const float> esdf_m) {
  if (!laneInsideVerticalWindow(lane, channel, config.footprint)) {
    return false;
  }
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < lane.centerline.size(); ++index) {
    const SweptFootprintResult result =
        validateSweptFootprint(grid, esdf_m, lane.centerline[index].position,
                               lane.centerline[index + 1U].position, config.footprint);
    if (!result.accepted()) {
      return false;
    }
    minimum_clearance_m = std::min(minimum_clearance_m, result.minimum_clearance_m);
  }
  lane.minimum_wall_clearance_m = minimum_clearance_m;
  return minimum_clearance_m + 1.0e-9 >= config.minimum_wall_clearance_m;
}

[[nodiscard]] bool validateRawLane(ChannelLane& lane, const OccupancyGrid3D& occupancy,
                                   const ChannelLaneConfig& config) noexcept {
  const FootprintBodyAxis body_axis{};
  for (std::size_t index = 0U; index + 1U < lane.centerline.size(); ++index) {
    const SweptFootprintResult result = validateRawSweptFootprint(
        occupancy, lane.centerline[index].position, body_axis,
        lane.centerline[index + 1U].position, body_axis, config.footprint);
    if (!result.accepted()) {
      return false;
    }
  }
  lane.minimum_wall_clearance_m = config.minimum_wall_clearance_m;
  return true;
}

void validateConfig(const ChannelLaneConfig& config) {
  if (!finitePositive(config.minimum_center_separation_m) ||
      !(config.minimum_wall_clearance_m >= 0.0) || config.maximum_lane_count == 0U ||
      config.maximum_lane_count > 32U || !(config.footprint.radius_m >= 0.0)) {
    throw std::invalid_argument{"invalid channel lane configuration"};
  }
}

} // namespace

std::vector<RouteSample3D>
offsetChannelCenterline(const std::span<const RouteSample3D> centerline,
                        const double lateral_offset_m) {
  if (centerline.size() < 2U || !std::isfinite(lateral_offset_m)) {
    return {};
  }
  std::vector<RouteSample3D> result(centerline.begin(), centerline.end());
  for (std::size_t index = 0U; index < centerline.size(); ++index) {
    Direction2 incoming{};
    Direction2 outgoing{};
    if (index > 0U) {
      incoming = normalizedDirection(centerline[index - 1U].position,
                                     centerline[index].position);
    }
    if (index + 1U < centerline.size()) {
      outgoing = normalizedDirection(centerline[index].position,
                                     centerline[index + 1U].position);
    }
    if (index == 0U) {
      incoming = outgoing;
    } else if (index + 1U == centerline.size()) {
      outgoing = incoming;
    }
    const Direction2 incoming_normal = lateralLeft(incoming);
    const Direction2 outgoing_normal = lateralLeft(outgoing);
    const double summed_x = incoming_normal.x + outgoing_normal.x;
    const double summed_y = incoming_normal.y + outgoing_normal.y;
    const double summed_length = std::hypot(summed_x, summed_y);
    Direction2 miter = incoming_normal;
    if (summed_length > 1.0e-9) {
      miter = Direction2{summed_x / summed_length, summed_y / summed_length};
    }
    const double denominator =
        miter.x * incoming_normal.x + miter.y * incoming_normal.y;
    const double scale = std::abs(denominator) > 1.0e-6 ? lateral_offset_m / denominator
                                                        : lateral_offset_m;
    result[index].position.x += scale * miter.x;
    result[index].position.y += scale * miter.y;
  }
  for (std::size_t index = 0U; index < result.size(); ++index) {
    const std::size_t first = index == 0U ? index : index - 1U;
    const std::size_t second = index + 1U < result.size() ? index + 1U : index;
    result[index].tangent =
        normalized3D(result[first].position, result[second].position);
  }
  return result;
}

ChannelLaneSet makeGeometricChannelLanes(const ConstrainedFreeSpaceEdge& channel,
                                         const ChannelLaneConfig& config) {
  validateConfig(config);
  if (channel.centerline.size() < 2U || !finitePositive(channel.width_m)) {
    throw std::invalid_argument{"invalid channel geometry for lane generation"};
  }
  const double boundary_allowance_m =
      config.footprint.radius_m + config.minimum_wall_clearance_m;
  const double usable_center_width_m =
      std::max(0.0, channel.width_m - 2.0 * boundary_allowance_m);
  std::size_t lane_count = 1U;
  if (usable_center_width_m > 0.0) {
    lane_count = static_cast<std::size_t>(std::floor(
                     usable_center_width_m / config.minimum_center_separation_m)) +
                 1U;
  }
  lane_count = std::clamp(lane_count, std::size_t{1U}, config.maximum_lane_count);

  ChannelLaneSet result{.channel_id = channel.id,
                        .physical_width_m = channel.width_m,
                        .usable_center_width_m = usable_center_width_m,
                        .lanes = {}};
  const std::vector<double> offsets =
      laneOffsets(lane_count, config.minimum_center_separation_m);
  result.lanes.reserve(offsets.size());
  for (std::size_t index = 0U; index < offsets.size(); ++index) {
    result.lanes.push_back(ChannelLane{
        .index = index,
        .lateral_offset_m = offsets[index],
        .centerline = offsetChannelCenterline(channel.centerline, offsets[index]),
    });
  }
  return result;
}

ChannelLaneSet makeCollisionValidatedChannelLanes(
    const ConstrainedFreeSpaceEdge& channel, const ChannelLaneConfig& config,
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m) {
  const ChannelLaneSet geometric = makeGeometricChannelLanes(channel, config);
  for (std::size_t lane_count = geometric.lanes.size(); lane_count > 0U; --lane_count) {
    ChannelLaneSet candidate{.channel_id = channel.id,
                             .physical_width_m = channel.width_m,
                             .usable_center_width_m = geometric.usable_center_width_m,
                             .lanes = {}};
    const std::vector<double> offsets =
        laneOffsets(lane_count, config.minimum_center_separation_m);
    candidate.lanes.reserve(lane_count);
    bool valid = true;
    for (std::size_t index = 0U; index < lane_count; ++index) {
      ChannelLane lane{.index = index,
                       .lateral_offset_m = offsets[index],
                       .centerline =
                           offsetChannelCenterline(channel.centerline, offsets[index])};
      if (!validateLane(lane, channel, config, grid, esdf_m)) {
        valid = false;
        break;
      }
      candidate.lanes.push_back(std::move(lane));
    }
    if (valid) {
      return candidate;
    }
  }
  return ChannelLaneSet{.channel_id = channel.id,
                        .physical_width_m = channel.width_m,
                        .usable_center_width_m = geometric.usable_center_width_m,
                        .lanes = {}};
}

ChannelLaneSet
makeRawCollisionValidatedChannelLanes(const ConstrainedFreeSpaceEdge& channel,
                                      const ChannelLaneConfig& config,
                                      const OccupancyGrid3D& occupancy) {
  const ChannelLaneSet geometric = makeGeometricChannelLanes(channel, config);
  for (std::size_t lane_count = geometric.lanes.size(); lane_count > 0U; --lane_count) {
    ChannelLaneSet candidate{.channel_id = channel.id,
                             .physical_width_m = channel.width_m,
                             .usable_center_width_m = geometric.usable_center_width_m,
                             .lanes = {}};
    const std::vector<double> offsets =
        laneOffsets(lane_count, config.minimum_center_separation_m);
    candidate.lanes.reserve(lane_count);
    bool valid = true;
    for (std::size_t index = 0U; index < lane_count; ++index) {
      ChannelLane lane{.index = index,
                       .lateral_offset_m = offsets[index],
                       .centerline =
                           offsetChannelCenterline(channel.centerline, offsets[index])};
      if (!laneInsideVerticalWindow(lane, channel, config.footprint) ||
          !validateRawLane(lane, occupancy, config)) {
        valid = false;
        break;
      }
      candidate.lanes.push_back(std::move(lane));
    }
    if (valid) {
      return candidate;
    }
  }
  return ChannelLaneSet{.channel_id = channel.id,
                        .physical_width_m = channel.width_m,
                        .usable_center_width_m = geometric.usable_center_width_m,
                        .lanes = {}};
}

std::string channelConflictResourceId(const std::string_view channel_id) {
  const std::size_t separator = channel_id.find(':');
  return std::string{channel_id.substr(0U, separator)};
}

} // namespace drone_city_nav

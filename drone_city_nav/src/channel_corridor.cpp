#include "drone_city_nav/channel_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <memory>
#include <mutex>
#include <ranges>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace drone_city_nav {
namespace {

struct Direction2 {
  double x{0.0};
  double y{0.0};
};

struct ValidOffsetRun {
  double minimum_m{0.0};
  double maximum_m{0.0};

  [[nodiscard]] double widthM() const noexcept {
    return maximum_m - minimum_m;
  }

  [[nodiscard]] bool containsCenter() const noexcept {
    return minimum_m <= 0.0 && maximum_m >= 0.0;
  }
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

void validateConfig(const ChannelCorridorConfig& config) {
  if (!finitePositive(config.desired_center_separation_m) ||
      !(config.minimum_wall_clearance_m >= 0.0) ||
      !finitePositive(config.lateral_probe_step_m) ||
      !(config.directional_offset_fraction >= 0.0) ||
      !(config.directional_offset_fraction <= 1.0) ||
      !(config.footprint.radius_m >= 0.0)) {
    throw std::invalid_argument{"invalid channel corridor configuration"};
  }
}

[[nodiscard]] bool
verticalWindowAccepted(const std::span<const RouteSample3D> centerline,
                       const ConstrainedFreeSpaceEdge& channel,
                       const SweptFootprintConfig& footprint) noexcept {
  return std::ranges::all_of(centerline, [&](const RouteSample3D& sample) {
    return sample.position.z - footprint.lower_extent_m >= channel.min_z_m &&
           sample.position.z + footprint.upper_extent_m <= channel.max_z_m;
  });
}

[[nodiscard]] std::vector<double> sampleOffsets(const double minimum_m,
                                                const double maximum_m,
                                                const double probe_step_m) {
  const std::size_t interval_count = static_cast<std::size_t>(
      std::max(1.0, std::ceil((maximum_m - minimum_m) / probe_step_m)));
  std::vector<double> offsets;
  offsets.reserve(interval_count + 2U);
  for (std::size_t index = 0U; index <= interval_count; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(interval_count);
    offsets.push_back(std::lerp(minimum_m, maximum_m, ratio));
  }
  if (minimum_m < 0.0 && maximum_m > 0.0) {
    offsets.push_back(0.0);
    std::ranges::sort(offsets);
  }
  return offsets;
}

[[nodiscard]] ValidOffsetRun
selectBestRun(const std::span<const double> offsets,
              const std::span<const std::uint8_t> accepted) noexcept {
  ValidOffsetRun best{};
  bool best_available = false;
  std::size_t index = 0U;
  while (index < offsets.size()) {
    while (index < offsets.size() && accepted[index] == 0U) {
      ++index;
    }
    if (index == offsets.size()) {
      break;
    }
    const std::size_t begin = index;
    while (index + 1U < offsets.size() && accepted[index + 1U] != 0U) {
      ++index;
    }
    const ValidOffsetRun candidate{offsets[begin], offsets[index]};
    const bool candidate_preferred =
        !best_available || (candidate.containsCenter() && !best.containsCenter()) ||
        (candidate.containsCenter() == best.containsCenter() &&
         candidate.widthM() > best.widthM() + 1.0e-9);
    if (candidate_preferred) {
      best = candidate;
      best_available = true;
    }
    ++index;
  }
  return best_available ? best : ValidOffsetRun{};
}

[[nodiscard]] std::string
corridorResourceKey(const std::span<const ConstrainedFreeSpaceEdge> channels,
                    const ChannelCorridorConfig& config,
                    const OccupancyGrid3D& occupancy) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << occupancy.fingerprint() << '|' << std::hexfloat
         << config.desired_center_separation_m << '|' << config.minimum_wall_clearance_m
         << '|' << config.lateral_probe_step_m << '|'
         << config.directional_offset_fraction << '|' << config.footprint.radius_m
         << '|' << config.footprint.lower_extent_m << '|'
         << config.footprint.upper_extent_m << '|' << config.footprint.sweep_step_m
         << '|' << std::defaultfloat << config.footprint.perimeter_samples << '|'
         << config.footprint.radial_rings << '|' << config.footprint.axial_samples
         << '|' << channels.size();
  for (const ConstrainedFreeSpaceEdge& channel : channels) {
    stream << '|' << channel.id.size() << ':' << channel.id << '|' << std::hexfloat
           << channel.min_z_m << '|' << channel.max_z_m << '|' << channel.width_m << '|'
           << channel.height_m << '|' << channel.centerline.size();
    for (const RouteSample3D& sample : channel.centerline) {
      stream << '|' << sample.position.x << ',' << sample.position.y << ','
             << sample.position.z;
    }
  }
  return stream.str();
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

ChannelCorridor makeGeometricChannelCorridor(const ConstrainedFreeSpaceEdge& channel,
                                             const ChannelCorridorConfig& config) {
  validateConfig(config);
  if (channel.centerline.size() < 2U || !finitePositive(channel.width_m)) {
    throw std::invalid_argument{"invalid channel geometry for corridor generation"};
  }
  const double boundary_allowance_m =
      config.footprint.radius_m + config.minimum_wall_clearance_m;
  const double usable_half_width_m =
      std::max(0.0, 0.5 * channel.width_m - boundary_allowance_m);
  return ChannelCorridor{
      .channel_id = channel.id,
      .physical_width_m = channel.width_m,
      .minimum_lateral_offset_m = -usable_half_width_m,
      .maximum_lateral_offset_m = usable_half_width_m,
      .minimum_wall_clearance_m = config.minimum_wall_clearance_m,
      .raw_validated = false,
  };
}

bool validateChannelOffset(const ConstrainedFreeSpaceEdge& channel,
                           const double lateral_offset_m,
                           const ChannelCorridorConfig& config,
                           const OccupancyGrid3D& occupancy) noexcept {
  const std::vector<RouteSample3D> centerline =
      offsetChannelCenterline(channel.centerline, lateral_offset_m);
  if (centerline.size() < 2U ||
      !verticalWindowAccepted(centerline, channel, config.footprint)) {
    return false;
  }
  const FootprintBodyAxis body_axis{};
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    if (!validateRawSweptFootprint(occupancy, centerline[index - 1U].position,
                                   body_axis, centerline[index].position, body_axis,
                                   config.footprint)
             .accepted()) {
      return false;
    }
  }
  return true;
}

ChannelCorridor
makeRawCollisionValidatedChannelCorridor(const ConstrainedFreeSpaceEdge& channel,
                                         const ChannelCorridorConfig& config,
                                         const OccupancyGrid3D& occupancy) {
  const ChannelCorridor geometric = makeGeometricChannelCorridor(channel, config);
  const std::vector<double> offsets =
      sampleOffsets(geometric.minimum_lateral_offset_m,
                    geometric.maximum_lateral_offset_m, config.lateral_probe_step_m);
  std::vector<std::uint8_t> accepted;
  accepted.reserve(offsets.size());
  for (const double offset_m : offsets) {
    accepted.push_back(static_cast<std::uint8_t>(
        validateChannelOffset(channel, offset_m, config, occupancy)));
  }
  const ValidOffsetRun best = selectBestRun(offsets, accepted);
  return ChannelCorridor{
      .channel_id = channel.id,
      .physical_width_m = channel.width_m,
      .minimum_lateral_offset_m = best.minimum_m,
      .maximum_lateral_offset_m = best.maximum_m,
      .minimum_wall_clearance_m = config.minimum_wall_clearance_m,
      .raw_validated = std::ranges::any_of(
          accepted, [](const std::uint8_t value) { return value != 0U; }),
  };
}

ChannelCorridorResource acquireRawValidatedChannelCorridors(
    const std::span<const ConstrainedFreeSpaceEdge> channels,
    const ChannelCorridorConfig& config, const OccupancyGrid3D& occupancy) {
  validateConfig(config);
  const std::string key = corridorResourceKey(channels, config, occupancy);
  using CorridorVector = std::vector<ChannelCorridor>;
  static std::mutex registry_mutex;
  static std::unordered_map<std::string, std::weak_ptr<const CorridorVector>> registry;
  const std::scoped_lock registry_lock{registry_mutex};
  if (const auto found = registry.find(key); found != registry.end()) {
    if (std::shared_ptr<const CorridorVector> existing = found->second.lock()) {
      return ChannelCorridorResource{
          .corridors = std::move(existing),
          .shared_resource_reused = true,
      };
    }
    registry.erase(found);
  }

  auto mutable_corridors = std::make_shared<CorridorVector>();
  mutable_corridors->reserve(channels.size());
  for (const ConstrainedFreeSpaceEdge& channel : channels) {
    mutable_corridors->push_back(
        makeRawCollisionValidatedChannelCorridor(channel, config, occupancy));
  }
  std::shared_ptr<const CorridorVector> corridors = std::move(mutable_corridors);
  registry.emplace(key, corridors);
  return ChannelCorridorResource{
      .corridors = std::move(corridors),
      .shared_resource_reused = false,
  };
}

double preferredDirectionalChannelOffset(const ChannelCorridor& corridor,
                                         const int direction_sign,
                                         const ChannelCorridorConfig& config) noexcept {
  if (!corridor.raw_validated || direction_sign == 0 ||
      !(corridor.maximum_lateral_offset_m >= corridor.minimum_lateral_offset_m)) {
    return 0.0;
  }
  const double sign = direction_sign > 0 ? -1.0 : 1.0;
  const double available_m = sign < 0.0 ? -corridor.minimum_lateral_offset_m
                                        : corridor.maximum_lateral_offset_m;
  const double desired_m =
      std::max(0.5 * config.desired_center_separation_m,
               config.directional_offset_fraction * std::max(0.0, available_m));
  return sign * std::min(std::max(0.0, available_m), desired_m);
}

std::string channelConflictResourceId(const std::string_view channel_id) {
  const std::size_t separator = channel_id.find(':');
  return std::string{channel_id.substr(0U, separator)};
}

} // namespace drone_city_nav

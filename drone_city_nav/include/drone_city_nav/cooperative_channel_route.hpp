#pragma once

#include "drone_city_nav/channel_corridor.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class CooperativeChannelRouteStatus : std::uint8_t {
  kCentered,
  kApplied,
  kMissingCorridorGeometry,
  kInsufficientTransition,
  kRawValidationRejected,
  kInvalidInput,
};

struct CooperativeChannelAssignment {
  std::string channel_id;
  std::uint64_t route_generation{0U};
  std::size_t span_index{0U};
  double physical_width_m{0.0};
  double minimum_lateral_offset_m{0.0};
  double maximum_lateral_offset_m{0.0};
  double requested_lateral_offset_m{0.0};
  double applied_lateral_offset_m{0.0};
  double desired_center_separation_m{0.0};
  CooperativeChannelRouteStatus status{CooperativeChannelRouteStatus::kCentered};

  [[nodiscard]] bool applied() const noexcept {
    return status == CooperativeChannelRouteStatus::kApplied;
  }

  [[nodiscard]] bool corridorAvailable() const noexcept {
    return maximum_lateral_offset_m >= minimum_lateral_offset_m;
  }
};

struct CooperativeChannelRouteConfig {
  double preferred_transition_length_m{10.0};
  double minimum_transition_length_m{3.0};
  double desired_center_separation_m{5.0};
  double directional_offset_fraction{0.5};
  SweptFootprintConfig footprint{};
};

struct CooperativeChannelRouteResult {
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::vector<CooperativeChannelAssignment> assignments;
  std::size_t applied_offset_count{0U};
  bool valid{false};
};

[[nodiscard]] CooperativeChannelRouteResult applyCooperativeChannelCorridors(
    std::span<const RouteSample3D> route,
    std::span<const ConstrainedRouteSpan> constrained_spans,
    std::span<const ChannelCorridor> corridors, const OccupancyGrid3D& occupancy,
    const CooperativeChannelRouteConfig& config);

[[nodiscard]] const char*
cooperativeChannelRouteStatusName(CooperativeChannelRouteStatus status) noexcept;

} // namespace drone_city_nav

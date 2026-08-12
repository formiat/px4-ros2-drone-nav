#pragma once

#include "drone_city_nav/channel_lanes.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class CooperativeChannelLaneRouteStatus : std::uint8_t {
  kExclusive,
  kApplied,
  kMissingLaneGeometry,
  kInsufficientTransition,
  kRawValidationRejected,
  kInvalidInput,
};

struct CooperativeChannelLaneAssignment {
  std::string channel_id;
  std::uint64_t route_generation{0U};
  std::size_t span_index{0U};
  std::size_t lane_index{0U};
  std::size_t lane_count{1U};
  double lateral_offset_m{0.0};
  CooperativeChannelLaneRouteStatus status{
      CooperativeChannelLaneRouteStatus::kExclusive};

  [[nodiscard]] bool applied() const noexcept {
    return status == CooperativeChannelLaneRouteStatus::kApplied;
  }
};

struct CooperativeChannelRouteConfig {
  double preferred_transition_length_m{10.0};
  double minimum_transition_length_m{3.0};
  SweptFootprintConfig footprint{};
};

struct CooperativeChannelRouteResult {
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::vector<CooperativeChannelLaneAssignment> assignments;
  std::size_t applied_lane_count{0U};
  bool valid{false};
};

[[nodiscard]] CooperativeChannelRouteResult
applyCooperativeChannelLanes(std::span<const RouteSample3D> route,
                             std::span<const ConstrainedRouteSpan> constrained_spans,
                             std::span<const ChannelLaneSet> lane_sets,
                             const OccupancyGrid3D& occupancy,
                             const CooperativeChannelRouteConfig& config);

[[nodiscard]] const char* cooperativeChannelLaneRouteStatusName(
    CooperativeChannelLaneRouteStatus status) noexcept;

} // namespace drone_city_nav

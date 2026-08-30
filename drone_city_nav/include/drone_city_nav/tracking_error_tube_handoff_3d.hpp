#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace drone_city_nav {

enum class TrackingErrorTubeHandoffStatus3D : std::uint8_t {
  kActive,
  kNotRequired,
  kInvalidContract,
  kOutsideValidityWindow,
  kReferenceAcquiredRouteTube,
  kSpeedLimitExceeded,
  kTrackingErrorExceeded,
};

struct TrackingErrorTubeHandoffObservation3D {
  std::int64_t stamp_ns{0};
  mppi::State state{};
};

struct TrackingErrorTubeHandoffAssessment3D {
  TrackingErrorTubeHandoffStatus3D status{
      TrackingErrorTubeHandoffStatus3D::kInvalidContract};
  std::size_t reference_state_index{0U};
  double reference_speed_limit_mps{0.0};
  double tracking_error_radius_m{0.0};
  double actual_tracking_error_m{0.0};

  [[nodiscard]] bool active() const noexcept {
    return status == TrackingErrorTubeHandoffStatus3D::kActive;
  }
};

[[nodiscard]] std::string_view
trackingErrorTubeHandoffStatus3DName(TrackingErrorTubeHandoffStatus3D status) noexcept;

// MPPI adapter for the controller-neutral tracking-tube contract. The
// connector exemption ends as soon as the reference enters the route-owned
// tube and cannot be reset by selecting another nearby horizon state.
[[nodiscard]] TrackingErrorTubeHandoffAssessment3D assessTrackingErrorTubeHandoff3D(
    std::span<const RouteSample3D> route, const TrackingErrorTubeProfile3D& profile,
    std::span<const mppi::State> handoff_states, double begin_route_station_m,
    std::int64_t valid_from_ns, std::int64_t valid_until_ns,
    std::int64_t control_interval_ns,
    const TrackingErrorTubeHandoffObservation3D& observation) noexcept;

} // namespace drone_city_nav

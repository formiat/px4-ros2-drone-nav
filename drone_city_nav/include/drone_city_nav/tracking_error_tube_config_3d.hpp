#pragma once

namespace drone_city_nav {

// Bounds the translational tracking error accumulated during the closed-loop
// response horizon. The tube collapses with commanded speed instead of
// inflating the hard planning footprint by a fixed margin. It is one law read
// by every layer: the route certification derives speed ceilings from it, the
// planner ranks a tight metre by the time it leaves execution, the speed
// policy caps the reference with it and the rollout cost prices its
// shortfall.
struct TrackingErrorTubeConfig3D {
  double response_time_s{0.15};
  // Lower bound on a constrained segment's speed ceiling wherever the physical
  // body itself clears raw occupancy. The tube then admits a small, bounded
  // tracking excursion instead of collapsing progress to centimetres per second
  // beside a wall. Zero keeps the pure clearance-derived ceiling.
  double minimum_progress_speed_mps{0.0};
};

[[nodiscard]] bool
trackingErrorTubeConfig3DIsValid(const TrackingErrorTubeConfig3D& config) noexcept;

// The tracking error the controller can accumulate at `speed_mps`.
[[nodiscard]] double trackingErrorTubeRadiusM(const TrackingErrorTubeConfig3D& config,
                                              double speed_mps) noexcept;

// The speed ceiling the tube admits at a body clearance: the clearance over
// the response time, no lower than the progress floor and no higher than
// `maximum_speed_mps`.
[[nodiscard]] double
trackingErrorTubeSpeedLimitMps(const TrackingErrorTubeConfig3D& config,
                               double clearance_m, double maximum_speed_mps) noexcept;

} // namespace drone_city_nav

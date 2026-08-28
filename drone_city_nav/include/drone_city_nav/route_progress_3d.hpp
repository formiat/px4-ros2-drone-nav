#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>

namespace drone_city_nav {

// Mission-route progress and release contracts are independent of planner internals.
enum class RouteReleaseReason3D : std::uint8_t {
  kNone,
  kNoActiveRoute,
  kBlocked,
  kExhausted,
  kStalled,
  kNoEligibleRollouts,
  kDiverged,
  kObjectiveChanged,
};

struct RouteProgressProjection3D {
  bool valid{false};
  double station_m{0.0};
  double total_length_m{0.0};
  double remaining_m{0.0};
  double cross_track_m{0.0};
  Point2 point{};
  Point2 tangent{};
};

struct RouteTrackingPolicy3D {
  double minimum_remaining_m{15.0};
  double maximum_cross_track_m{15.0};
};

struct RouteProgressConfig3D {
  double observation_window_s{1.0};
  double minimum_progress_m{0.5};
  double minimum_predicted_head_progress_m{0.5};
};

struct RouteProgressObservation3D {
  std::int64_t stamp_ns{0};
  std::uint64_t route_generation{0U};
  double station_m{0.0};
  double predicted_head_progress_m{0.0};
  double cross_track_m{0.0};
  bool recovery_active{false};
  bool controller_active{false};
};

enum class RouteProgressAction3D : std::uint8_t {
  kNone,
  kReseedLocalMppi,
  kReleaseLowPredictedProgress,
  kReleasePredictionMismatch,
};

struct RouteProgressUpdate3D {
  RouteProgressAction3D action{RouteProgressAction3D::kNone};
  bool stalled{false};
  bool local_reseed_requested{false};
  std::uint64_t stall_generation{0U};
  std::uint64_t local_reseed_generation{0U};
  double observation_age_s{0.0};
  double progress_m{0.0};
  double predicted_head_progress_m{0.0};
};

class RouteProgressTracker3D {
public:
  explicit RouteProgressTracker3D(const RouteProgressConfig3D& config = {});

  [[nodiscard]] RouteProgressUpdate3D
  evaluate(const RouteProgressObservation3D& observation);

private:
  void resetAnchor(const RouteProgressObservation3D& observation) noexcept;

  RouteProgressConfig3D config_{};
  bool anchor_valid_{false};
  std::int64_t anchor_stamp_ns_{0};
  std::uint64_t anchor_route_generation_{0U};
  double anchor_station_m_{0.0};
  double anchor_cross_track_m_{0.0};
  std::uint64_t stall_generation_{0U};
  std::uint64_t local_reseed_generation_{0U};
  bool local_reseed_pending_{false};
};

[[nodiscard]] const char*
routeReleaseReason3DName(RouteReleaseReason3D reason) noexcept;
[[nodiscard]] const char*
routeProgressAction3DName(RouteProgressAction3D action) noexcept;

} // namespace drone_city_nav

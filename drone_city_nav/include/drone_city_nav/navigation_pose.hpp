#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct NavigationPose2D {
  Pose2 pose{};
  double altitude_m{0.0};
  std::int64_t stamp_ns{0};
  bool position_valid{false};
  bool yaw_valid{false};
  bool altitude_valid{false};
};

struct Px4LocalPositionSample {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double heading_rad{0.0};
  std::int64_t stamp_ns{0};
  bool xy_valid{false};
  bool z_valid{false};
  bool heading_good_for_control{false};
};

struct Px4LocalPoseConfig {
  bool use_heading_for_yaw{true};
  double initial_heading_rad{0.0};
  double map_origin_x_m{0.0};
  double map_origin_y_m{0.0};
};

enum class Px4LocalPoseUpdateStatus {
  kAccepted,
  kInvalidPosition,
  kInvalidYaw,
};

[[nodiscard]] double normalizeYaw(double yaw_rad) noexcept;

[[nodiscard]] bool
timestampIsFresh(std::int64_t stamp_ns, std::int64_t now_ns,
                 std::int64_t max_staleness_ns,
                 std::int64_t max_future_skew_ns = 100'000'000) noexcept;

void invalidateNavigationPose(NavigationPose2D& pose) noexcept;

[[nodiscard]] bool navigationPoseReadyForScan(const NavigationPose2D& pose,
                                              std::int64_t last_update_ns,
                                              std::int64_t now_ns,
                                              std::int64_t max_staleness_ns) noexcept;

[[nodiscard]] std::optional<NavigationPose2D>
makeNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                       const Px4LocalPoseConfig& config) noexcept;

[[nodiscard]] Px4LocalPoseUpdateStatus
updateNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                         const Px4LocalPoseConfig& config,
                                         NavigationPose2D& state) noexcept;

} // namespace drone_city_nav

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

struct GeoReference {
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  bool initialized{false};
};

struct GpsFixSample {
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  std::int64_t stamp_ns{0};
  int status{0};
  bool horizontal_variance_known{false};
  double horizontal_variance_m2{0.0};
};

struct QuaternionSample {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
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
};

enum class Px4LocalPoseUpdateStatus {
  kAccepted,
  kInvalidPosition,
  kInvalidYaw,
};

struct GpsCompassConfig {
  int min_fix_status{0};
  std::int64_t max_gps_staleness_ns{1'000'000'000};
  double max_gps_horizontal_variance_m2{25.0};
  bool require_known_gps_covariance{false};
  bool auto_initialize_origin{true};
  double origin_latitude_deg{0.0};
  double origin_longitude_deg{0.0};
  double origin_altitude_m{0.0};
  double yaw_offset_rad{0.0};
  double magnetic_declination_rad{0.0};
  double compass_to_body_yaw_offset_rad{0.0};
};

[[nodiscard]] double normalizeYaw(double yaw_rad) noexcept;

[[nodiscard]] bool timestampIsFresh(std::int64_t stamp_ns, std::int64_t now_ns,
                                    std::int64_t max_staleness_ns) noexcept;

void invalidateNavigationPose(NavigationPose2D& pose) noexcept;

[[nodiscard]] bool navigationPoseReadyForScan(const NavigationPose2D& pose,
                                              std::int64_t last_update_ns,
                                              std::int64_t now_ns,
                                              std::int64_t max_staleness_ns) noexcept;

[[nodiscard]] bool validGpsFix(const GpsFixSample& fix, const GpsCompassConfig& config,
                               std::int64_t now_ns) noexcept;

[[nodiscard]] Point2 projectWgs84ToLocal(const GpsFixSample& fix,
                                         const GeoReference& origin) noexcept;

[[nodiscard]] std::optional<double>
yawFromQuaternion(const QuaternionSample& quaternion) noexcept;

[[nodiscard]] std::optional<NavigationPose2D>
makeNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                       const Px4LocalPoseConfig& config) noexcept;

[[nodiscard]] Px4LocalPoseUpdateStatus
updateNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                         const Px4LocalPoseConfig& config,
                                         NavigationPose2D& state) noexcept;

[[nodiscard]] std::optional<NavigationPose2D>
makeNavigationPoseFromGpsCompass(const GpsFixSample& fix, double compass_yaw_rad,
                                 std::int64_t now_ns, const GpsCompassConfig& config,
                                 GeoReference& origin) noexcept;

} // namespace drone_city_nav

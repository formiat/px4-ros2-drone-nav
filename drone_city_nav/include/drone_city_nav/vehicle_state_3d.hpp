#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <cstdint>

namespace drone_city_nav {

struct VehicleStateIdentity3D {
  std::uint64_t revision{0U};
  std::uint64_t source_timestamp_us{0U};
  std::int64_t receive_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    return revision != 0U && receive_stamp_ns > 0;
  }

  [[nodiscard]] bool operator==(const VehicleStateIdentity3D&) const noexcept = default;
};

// Exact controller-neutral vehicle state captured at one navigation input
// revision. Compilation seals this complete value so every trajectory cost and
// controller adapter refers to the same measured initial dynamics.
struct VehicleState3D {
  VehicleStateIdentity3D identity{};
  Point3 position{};
  Vec3 velocity{};
  double yaw_rad{0.0};
  double yaw_rate_radps{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return identity.valid() && std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(position.z) && std::isfinite(velocity.x) &&
           std::isfinite(velocity.y) && std::isfinite(velocity.z) &&
           std::isfinite(yaw_rad) && std::isfinite(yaw_rate_radps);
  }

  [[nodiscard]] bool operator==(const VehicleState3D& other) const noexcept {
    return identity == other.identity && position.x == other.position.x &&
           position.y == other.position.y && position.z == other.position.z &&
           velocity.x == other.velocity.x && velocity.y == other.velocity.y &&
           velocity.z == other.velocity.z && yaw_rad == other.yaw_rad &&
           yaw_rate_radps == other.yaw_rate_radps;
  }
};

} // namespace drone_city_nav

#include "drone_city_nav/swept_vehicle_separation.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] double norm(const Vec3& value) noexcept {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

[[nodiscard]] Vec3 subtract(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

} // namespace

SweptVehicleSeparation sweptVehicleSeparation(
    const TimedVehicleState& first, const TimedVehicleState& second,
    const std::optional<TimedVehicleState>& previous_first,
    const std::optional<TimedVehicleState>& previous_second) noexcept {
  const Vec3 current_relative = subtract(first.position, second.position);
  SweptVehicleSeparation result;
  result.current_m = norm(current_relative);
  result.minimum_m = result.current_m;
  if (!previous_first || !previous_second) {
    return result;
  }
  const Vec3 previous_relative =
      subtract(previous_first->position, previous_second->position);
  const Vec3 delta{current_relative.x - previous_relative.x,
                   current_relative.y - previous_relative.y,
                   current_relative.z - previous_relative.z};
  const double delta_norm_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  if (delta_norm_squared <= 1.0e-12) {
    const double previous_m = norm(previous_relative);
    if (previous_m < result.minimum_m) {
      result.minimum_m = previous_m;
      result.interpolation_fraction = 0.0;
    }
    return result;
  }
  const double projection =
      -(previous_relative.x * delta.x + previous_relative.y * delta.y +
        previous_relative.z * delta.z) /
      delta_norm_squared;
  const double fraction = std::clamp(projection, 0.0, 1.0);
  const Vec3 closest{previous_relative.x + fraction * delta.x,
                     previous_relative.y + fraction * delta.y,
                     previous_relative.z + fraction * delta.z};
  const double closest_m = norm(closest);
  if (closest_m < result.minimum_m) {
    result.minimum_m = closest_m;
    result.interpolation_fraction = fraction;
  }
  return result;
}

} // namespace drone_city_nav

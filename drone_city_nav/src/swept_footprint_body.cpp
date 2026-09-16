#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "swept_footprint_internal.hpp"

// The body the swept validators answer to: how the airframe's thrust axis
// tilts with the commanded acceleration, how far that tilt can reach under the
// dynamics, and the footprint whose envelope contains the hull at every such
// tilt.

namespace drone_city_nav {

using swept_footprint_detail::normalized;

FootprintBodyAxis bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                                                const double gravity_mps2) noexcept {
  return normalized(FootprintBodyAxis{acceleration_mps2.x, acceleration_mps2.y,
                                      acceleration_mps2.z + gravity_mps2});
}

double maximumBodyTiltRad(const double maximum_horizontal_acceleration_mps2,
                          const double maximum_vertical_acceleration_mps2,
                          const double gravity_mps2) noexcept {
  const double horizontal_mps2 =
      std::isfinite(maximum_horizontal_acceleration_mps2)
          ? std::max(0.0, maximum_horizontal_acceleration_mps2)
          : 0.0;
  const double vertical_mps2 = std::isfinite(maximum_vertical_acceleration_mps2)
                                   ? std::max(0.0, maximum_vertical_acceleration_mps2)
                                   : 0.0;
  const double gravity =
      std::isfinite(gravity_mps2) ? std::max(0.0, gravity_mps2) : 0.0;
  // Descending at the vertical limit leaves the least thrust along gravity,
  // so the same horizontal acceleration tilts the axis the furthest.
  const double thrust_along_gravity_mps2 = gravity - vertical_mps2;
  if (horizontal_mps2 <= 0.0) {
    return 0.0;
  }
  if (thrust_along_gravity_mps2 <= 0.0) {
    return std::numbers::pi / 2.0;
  }
  return std::atan2(horizontal_mps2, thrust_along_gravity_mps2);
}

SweptFootprintConfig tiltEnvelopedFootprint(const SweptFootprintConfig& footprint,
                                            const double tilt_rad) noexcept {
  const double tilt =
      std::isfinite(tilt_rad) ? std::clamp(tilt_rad, 0.0, std::numbers::pi / 2.0) : 0.0;
  const double sine = std::sin(tilt);
  const double cosine = std::cos(tilt);
  const SweptFootprintConfig body = physicalBodyFootprint(footprint);
  const double body_radius_m = std::max(0.0, body.radius_m);
  const double body_lower_m = std::max(0.0, body.lower_extent_m);
  const double body_upper_m = std::max(0.0, body.upper_extent_m);
  const double axial_extent_m = std::max(body_lower_m, body_upper_m);
  // The horizontal reach of the body cylinder tilted by the angle: its rim
  // projects to an ellipse displaced by the extent's lean, whose farthest
  // point from the axis is radius * cos + extent * sin while the tilt is
  // shallower than the body's diagonal, and the diagonal itself,
  // sqrt(radius^2 + extent^2), beyond that. The rim dips or rises by
  // radius * sin beyond the extents' own projection, and that is exact.
  const double leaned_body_radius_m =
      body_radius_m * sine <= axial_extent_m * cosine
          ? body_radius_m * cosine + axial_extent_m * sine
          : std::hypot(body_radius_m, axial_extent_m);
  // The body becomes the hull at every tilt: in flight the airframe may lean
  // that far at any moment, so contact evidence is judged against the volume
  // it can reach, not only the volume it occupies upright. The envelope grows
  // to contain that body and keeps whatever clearance it already carried
  // beyond it. The hull as configured stays the departure's body, where the
  // vehicle moves at hover.
  const double leaned_lower_m = body_lower_m * cosine + body_radius_m * sine;
  const double leaned_upper_m = body_upper_m * cosine + body_radius_m * sine;
  SweptFootprintConfig enveloped = footprint;
  enveloped.body_radius_m = leaned_body_radius_m;
  enveloped.body_lower_extent_m = leaned_lower_m;
  enveloped.body_upper_extent_m = leaned_upper_m;
  enveloped.radius_m =
      std::max(std::max(0.0, footprint.radius_m), leaned_body_radius_m);
  enveloped.lower_extent_m =
      std::max(std::max(0.0, footprint.lower_extent_m), leaned_lower_m);
  enveloped.upper_extent_m =
      std::max(std::max(0.0, footprint.upper_extent_m), leaned_upper_m);
  return enveloped;
}

const char* sweptFootprintStatusName(const SweptFootprintStatus status) noexcept {
  switch (status) {
    case SweptFootprintStatus::kValid:
      return "valid";
    case SweptFootprintStatus::kInvalidInput:
      return "invalid_input";
    case SweptFootprintStatus::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
}

SweptFootprintConfig clearanceReducedFootprint(const SweptFootprintConfig& footprint,
                                               const double reduction) noexcept {
  const double share = std::isfinite(reduction) ? std::clamp(reduction, 0.0, 1.0) : 1.0;
  const SweptFootprintConfig body = physicalBodyFootprint(footprint);
  if (!(share > 0.0)) {
    return footprint;
  }
  if (!(share < 1.0)) {
    return body;
  }
  const auto blend = [share](const double envelope_m, const double body_m) {
    return envelope_m + share * (body_m - envelope_m);
  };
  SweptFootprintConfig reduced = footprint;
  reduced.radius_m = blend(footprint.radius_m, body.radius_m);
  reduced.lower_extent_m = blend(footprint.lower_extent_m, body.lower_extent_m);
  reduced.upper_extent_m = blend(footprint.upper_extent_m, body.upper_extent_m);
  return reduced;
}

} // namespace drone_city_nav

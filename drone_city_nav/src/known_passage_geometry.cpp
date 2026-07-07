#include "drone_city_nav/known_passage_geometry.hpp"

#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-9;

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

} // namespace

std::optional<KnownPassageOpeningFrame>
knownPassageOpeningFrame(const PassageOpening& opening) {
  const Point2 normal = normalized(opening.normal_xy);
  if (!(norm(normal) > kTinyDistanceM) || !(opening.width_m > 0.0) ||
      !(opening.depth_m > 0.0)) {
    return std::nullopt;
  }
  return KnownPassageOpeningFrame{
      .center = Point2{opening.center.x, opening.center.y},
      .normal = normal,
      .lateral = Point2{-normal.y, normal.x},
      .half_width_m = opening.width_m / 2.0,
      .half_depth_m = opening.depth_m / 2.0,
  };
}

KnownPassageOpeningLocalPoint
knownPassageOpeningLocalPoint(const KnownPassageOpeningWorldPoint& point,
                              const KnownPassageOpeningFrame& frame) {
  const double dx = point.point.x - frame.center.x;
  const double dy = point.point.y - frame.center.y;
  return KnownPassageOpeningLocalPoint{
      .u_m = dx * frame.normal.x + dy * frame.normal.y,
      .v_m = dx * frame.lateral.x + dy * frame.lateral.y,
      .z_m = point.z_m,
      .s_m = point.s_m,
  };
}

double
knownPassageOpeningLateralClearanceM(const KnownPassageOpeningLocalPoint& point,
                                     const KnownPassageOpeningFrame& frame) noexcept {
  return frame.half_width_m - std::abs(point.v_m);
}

double knownPassageOpeningVerticalClearanceM(const KnownPassageOpeningLocalPoint& point,
                                             const PassageOpening& opening) noexcept {
  return std::min(point.z_m - opening.min_z_m, opening.max_z_m - point.z_m);
}

double
knownPassageOpeningPassageClearanceM(const KnownPassageOpeningLocalPoint& point,
                                     const PassageOpening& opening,
                                     const KnownPassageOpeningFrame& frame) noexcept {
  return std::min(knownPassageOpeningLateralClearanceM(point, frame),
                  knownPassageOpeningVerticalClearanceM(point, opening));
}

double
knownPassageOpeningSignedVolumeMarginM(const KnownPassageOpeningLocalPoint& point,
                                       const PassageOpening& opening,
                                       const KnownPassageOpeningFrame& frame) noexcept {
  return std::min(frame.half_depth_m - std::abs(point.u_m),
                  knownPassageOpeningPassageClearanceM(point, opening, frame));
}

Point2
knownPassageOpeningGateEntryPoint(const PassageOpening& opening,
                                  const KnownPassageOpeningFrame& frame) noexcept {
  return Point2{opening.center.x - frame.normal.x * frame.half_depth_m,
                opening.center.y - frame.normal.y * frame.half_depth_m};
}

Point2
knownPassageOpeningGateExitPoint(const PassageOpening& opening,
                                 const KnownPassageOpeningFrame& frame) noexcept {
  return Point2{opening.center.x + frame.normal.x * frame.half_depth_m,
                opening.center.y + frame.normal.y * frame.half_depth_m};
}

} // namespace drone_city_nav

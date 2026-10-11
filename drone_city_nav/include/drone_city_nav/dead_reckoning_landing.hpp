#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

// The landing rung of the ladder a vehicle without sight descends (roadmap
// item 17 stage 5, specification K19). A position source that declares dead
// reckoning carries the vehicle off at 0.2 to 0.6 m/s from its first seconds
// (r856, r878, r879: a wall 2.4 to 3.5 m away met within 10 to 15 s, while the
// vehicle held and then let the autopilot land it blind). Once the source has
// dead reckoned for three seconds, four seconds unaided, half a second past
// the dark that judges the carried light (K12), the vehicle descends; a
// source that sees again before the vehicle is down takes the flight back.
// The descent asks the autopilot for a vertical velocity and no position, in
// the velocity mode, so it goes on when the autopilot's own estimate stops
// being a position: r1093 lost it at the touchdown, was sent the takeoff
// setpoint in the descent's place, and the autopilot's failsafe held the
// vehicle armed on the platform for six minutes.
class DeadReckoningLanding final {
public:
  // One pose of the position source, at `stamp_ns`. A source that falls silent
  // after declaring dead reckoning is still dead reckoning.
  void observe(const std::int64_t stamp_ns, const bool dead_reckoning) noexcept {
    if (!dead_reckoning) {
      since_ns_.reset();
    } else if (!since_ns_.has_value()) {
      since_ns_ = stamp_ns;
    }
  }

  [[nodiscard]] bool due(const std::int64_t now_ns) const noexcept {
    constexpr std::int64_t kDeadReckonedNs{3'000'000'000};
    return since_ns_.has_value() && now_ns - *since_ns_ >= kDeadReckonedNs;
  }

private:
  std::optional<std::int64_t> since_ns_;
};

// The reflex of the descent (K19): the time-of-flight ring's lateral units
// keep the surfaces they see off the body while the descent holds no
// position. Dead reckoning carried the vehicle 0.55 to 1.25 m in the 6 to 9 s
// to the ground (r934 to r943), and r1181 met the ledge of the staging area
// with a rotor at 0.6 m/s; the ring sees 2.8 m to each side, in truth. A unit
// that reads a surface nearer than 1.2 m, the body's radius and the rest
// clearance and a quarter metre of the drift, sends the vehicle away from it
// at 0.3 m/s; two facing surfaces cancel and the vehicle stays between them.
// The ranges are the units' own, forward, backward, left and right of the
// body; a unit without a fresh reading reads nothing. The velocity comes
// back in the autopilot's local frame for the body heading there.
struct TofRingRanges {
  std::optional<double> front_m;
  std::optional<double> back_m;
  std::optional<double> left_m;
  std::optional<double> right_m;
};

[[nodiscard]] inline Point2
blindDescentReflexVelocityNed(const TofRingRanges& ranges,
                              const double heading_ned_rad) noexcept {
  constexpr double kReflexRangeM{1.2};
  constexpr double kReflexSpeedMps{0.3};
  const auto near = [](const std::optional<double>& range_m) {
    return range_m.has_value() && std::isfinite(*range_m) && *range_m < kReflexRangeM;
  };
  // The body's forward-right-down frame: away from a surface on the left is
  // to the right, away from one ahead is backward.
  const double forward = (near(ranges.back_m) ? kReflexSpeedMps : 0.0) -
                         (near(ranges.front_m) ? kReflexSpeedMps : 0.0);
  const double right = (near(ranges.left_m) ? kReflexSpeedMps : 0.0) -
                       (near(ranges.right_m) ? kReflexSpeedMps : 0.0);
  const double cosine = std::cos(heading_ned_rad);
  const double sine = std::sin(heading_ned_rad);
  return Point2{cosine * forward - sine * right, sine * forward + cosine * right};
}

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/types.hpp"

#include <algorithm>
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
// Before the descent, dead reckoning in the dark flies no plan: the planner
// flew its plan on the dead-reckoned estimate through the three seconds, up
// to 1.3 m by the truth (r1441), and entered the descent at up to 0.6 m/s
// (r1421, r1442). Dark is K12's blind frame; a frame that sees while the
// estimator has no features (the flat staging base in daylight, 0.1 to 0.5 s
// at a time, r1430 to r1434) keeps its plan.
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

  // The source is dead reckoning now, however long it has been.
  [[nodiscard]] bool active() const noexcept {
    return since_ns_.has_value();
  }

  [[nodiscard]] bool due(const std::int64_t now_ns) const noexcept {
    constexpr std::int64_t kDeadReckonedNs{3'000'000'000};
    return since_ns_.has_value() && now_ns - *since_ns_ >= kDeadReckonedNs;
  }

  // The share of its configured range the frame alone grants, the measure
  // the light is judged by (K12), at `stamp_ns`.
  void observeSensorShare(const std::int64_t stamp_ns,
                          const double sensor_share) noexcept {
    share_stamp_ns_ = stamp_ns;
    share_blind_ = sensor_share >= 0.0 &&
                   sensor_share < LightReliabilityJudgment::kBlindSensorShare;
  }

  // The source dead reckons and the latest frame, no older than a second, is
  // blind.
  [[nodiscard]] bool blind(const std::int64_t now_ns) const noexcept {
    constexpr std::int64_t kShareFreshNs{1'000'000'000};
    return active() && share_blind_ && share_stamp_ns_.has_value() &&
           now_ns - *share_stamp_ns_ <= kShareFreshNs;
  }

private:
  std::optional<std::int64_t> since_ns_;
  std::optional<std::int64_t> share_stamp_ns_;
  bool share_blind_{false};
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
// The velocity is closed on the range itself: the autopilot runs its velocity
// loop on the dead-reckoned estimate, whose error is the drift the reflex
// answers for (0.25 to 0.3 m/s on r1237 to r1240, the vehicle still in truth
// while the estimate moved at the 0.3 m/s asked), so the range the unit reads
// is the only measure of the motion. The range is asked to open at 0.3 m/s
// from where the push began, and each metre it lags adds a metre per second,
// up to 1.0 m/s; the push never pulls toward the surface.
// The lateral units count only their near-horizontal rows (the caller's), and
// no push is given once the down unit reads the floor within 0.65 m (the feet
// half a metre above it): a sloped floor met by the descent closes the
// range by itself, and the closed law read it as lag and drove the vehicle
// downhill at 0.65 m/s into the ramp of the staging area (r1267, r1268, a
// landing at more than 0.5 m/s sideways is a crash by A9).
struct TofRingRanges {
  std::optional<double> front_m;
  std::optional<double> back_m;
  std::optional<double> left_m;
  std::optional<double> right_m;
  std::optional<double> down_m;
};

class BlindDescentReflex final {
public:
  static constexpr double kReflexRangeM{1.2};
  static constexpr double kReflexSpeedMps{0.3};
  static constexpr double kLagGainPerS{1.0};
  static constexpr double kMaximumSpeedMps{1.0};
  static constexpr double kFloorStopM{0.65};

  [[nodiscard]] Point2 observe(const TofRingRanges& ranges, const std::int64_t stamp_ns,
                               const double heading_ned_rad) noexcept {
    if (ranges.down_m.has_value() && *ranges.down_m < kFloorStopM) {
      reset();
      return Point2{0.0, 0.0};
    }
    // The body's forward-right-down frame: away from a surface on the left is
    // to the right, away from one ahead is backward.
    const double forward = axisSpeed(forward_, ranges.back_m, ranges.front_m, stamp_ns);
    const double right = axisSpeed(right_, ranges.left_m, ranges.right_m, stamp_ns);
    const double cosine = std::cos(heading_ned_rad);
    const double sine = std::sin(heading_ned_rad);
    return Point2{cosine * forward - sine * right, sine * forward + cosine * right};
  }

  // The descent over: the next one begins its pushes afresh.
  void reset() noexcept {
    forward_ = Axis{};
    right_ = Axis{};
  }

private:
  struct Axis {
    std::optional<std::int64_t> since_ns;
    double range_at_start_m{0.0};
    double sign{0.0};
  };

  [[nodiscard]] static bool near(const std::optional<double>& range_m) noexcept {
    return range_m.has_value() && std::isfinite(*range_m) && *range_m < kReflexRangeM;
  }

  // The speed along one axis, positive toward `positive_side`'s unit: the
  // push away from the one near unit, nothing between two or none.
  [[nodiscard]] static double axisSpeed(Axis& axis,
                                        const std::optional<double>& positive_side_m,
                                        const std::optional<double>& negative_side_m,
                                        const std::int64_t stamp_ns) noexcept {
    const bool positive_near = near(positive_side_m);
    const bool negative_near = near(negative_side_m);
    if (positive_near == negative_near) {
      axis = Axis{};
      return 0.0;
    }
    const double sign = positive_near ? 1.0 : -1.0;
    const double range_m = positive_near ? *positive_side_m : *negative_side_m;
    if (!axis.since_ns.has_value() || axis.sign != sign) {
      axis = Axis{.since_ns = stamp_ns, .range_at_start_m = range_m, .sign = sign};
    }
    const double elapsed_s = 1.0e-9 * static_cast<double>(stamp_ns - *axis.since_ns);
    const double asked_m = axis.range_at_start_m + kReflexSpeedMps * elapsed_s;
    const double speed = std::clamp(
        kReflexSpeedMps + kLagGainPerS * (asked_m - range_m), 0.0, kMaximumSpeedMps);
    return sign * speed;
  }

  Axis forward_;
  Axis right_;
};

} // namespace drone_city_nav

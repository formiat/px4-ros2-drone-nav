#pragma once

#include <deque>

namespace drone_city_nav {

// Roadmap item 17 stage 5: what the mission monitor concludes about the
// carried light. The vehicle is never told when or how its light fails; it
// judges the light from what it sees, the braking contract's measured range
// against the range the sensor set guarantees when it sees, and it is told
// the charge of the light's battery, as any airframe knows its batteries.
//
// An outage is a frame that observes nothing: the measured range at its
// floor, the physical margin. A range merely short of the guaranteed one is
// a dim frame, which a place makes as well as a failing light: the vehicle
// stopped at the edge of a zone that fails the light (roadmap item 17 stage
// 7) sees dimly and marks what it cannot see (stage 8), and its light is not
// at fault. A light is judged unreliable once one outage has lasted
// `kLongestOutageS`, or once outages have taken
// `kOutageShare` of the last `kWindowS`. Both are measured on the two regimes
// of the owner's decision: the moderate flicker every flight carries dipped
// the range for at most 1.8 s and 1.5 percent of the time (r785), no more
// than a flight without it (2.6 s and 3 percent, r781), and the severe
// failure must reach them. The outage is judged while the vehicle still holds
// on dead reckoning, before the drift of a longer one outgrows a doorway
// (r795, r796): at 10 s the severe failure of r789 outlived the hold and the
// autopilot landed blind. The judgment stands once reached: it sends the
// vehicle home.
class LightReliabilityJudgment final {
public:
  static constexpr double kLongestOutageS{4.0};
  static constexpr double kWindowS{120.0};
  static constexpr double kOutageShare{0.6};

  void observe(double stamp_s, double measured_range_m, double physical_margin_m);

  [[nodiscard]] bool unreliable() const noexcept {
    return unreliable_;
  }

  [[nodiscard]] double currentOutageS() const noexcept;
  [[nodiscard]] double outageShare() const noexcept;

private:
  struct Sample {
    double stamp_s;
    bool outage;
  };

  std::deque<Sample> samples_;
  double outage_started_s_{-1.0};
  bool unreliable_{false};
};

// The time the way to B will take, from the committed route's length left to
// it and the flight's mean speed: the route through unknown space is the
// shortest the vehicle may find, and on five camera flights the way it flew
// took 2.5 to 3.7 times the estimate at the median and at most 8.7 times
// (r773, r776, r781, r784, r785). The mean speed is floored at 0.5 m/s, as
// the vehicle stands at the start.
[[nodiscard]] double carriedLightGoalEstimateS(double route_remaining_m,
                                               double mean_speed_mps) noexcept;

} // namespace drone_city_nav

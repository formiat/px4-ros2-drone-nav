#pragma once

#include <deque>

namespace drone_city_nav {

// Roadmap items 17 stage 5 and 29: what the mission monitor concludes about
// the carried light. The vehicle is never told when or how its light fails;
// it reads what the light's driver reports (its output, as a current sensor
// or a photodiode at the emitter reads it), it reads its frames against what
// its time-of-flight ring sees, and it is told the charge of the light's
// battery, as any airframe knows its batteries.
//
// An outage is the light's reading under a quarter of its nominal output
// (the light then reaches half as far), or a frame the braking contract reads
// as blind (the share of its configured range it grants under 0.35, the line
// of the unobservable evidence, specification K12, K14) while the ring sees
// surfaces within its range in the camera's field that the frame shows dark:
// a failure the reading cannot see, a soiled lens or a lamp that dims on its
// current. A blind frame alone is the place's darkness, not the light's: a
// large dark space where the light reaches nothing (roadmap item 29). With no
// reading (no driver), a blind frame is an outage as before item 29: the
// vehicle could not tell a place that fails its light from a light that fails
// (I7), and a light that leaves it blind is unreliable wherever it is. It
// flies home back the way it came. A light is judged unreliable once one outage has
// lasted `kLongestOutageS`, or once outages have taken `kOutageShare` of the last
// `kWindowS`. Both are measured on the two regimes of the owner's decision: the
// moderate flicker every flight carries dipped the range for at most 1.8 s and 1.5
// percent of the time (r785), no more than a flight without it (2.6 s and 3 percent,
// r781), and the severe failure must reach them. The outage is judged while the vehicle
// still holds on dead reckoning, before the drift of a longer one outgrows a doorway
// (r795, r796): at 10 s the severe failure of r789 outlived the hold and the
// autopilot landed blind. A dark stretch reads half a second shorter than it
// is, the camera's gain hiding its ramps and the frames arriving late: under
// a line of 4 s the severe failure's 4.5 s stretches read 3.6 to 4.1 s and
// were judged one to two minutes late (r890, r925 to r927). The judgment
// stands once reached: it sends the vehicle home.
class LightReliabilityJudgment final {
public:
  // The share of its range the braking contract grants under which it reads
  // the forward sensor as blind, the line of the unobservable evidence too
  // (specification K12, K14): the range within a quarter metre of the margin.
  static constexpr double kBlindSensorShare{0.35};
  // Under it the light's reading is an outage, the light reaching half as
  // far: the moderate flicker every flight carries stays under it 2.93 s at
  // the longest over a thousand of its schedules and 3.0 s at most by its
  // bounds (2 s dark, ramps of 2 s a quarter under the line); the severe
  // failure is judged 55 s after take-off and the light lost 48.6 s
  // (tools/light_lines.py), where the frames judged the severe one at 100.6 s
  // (r1445).
  static constexpr double kLightReadingOutage{0.25};
  // The share of the ring's surfaces in the camera's field the frame shows
  // dark at which the dark frame is the light's failure.
  static constexpr double kUnlitSurfaceShare{0.5};
  static constexpr double kLongestOutageS{3.5};
  static constexpr double kWindowS{120.0};
  static constexpr double kOutageShare{0.6};

  // `light_reading` is the light's output over its nominal (NaN: no reading),
  // `unlit_surface_share` the share of the ring's surfaces in the camera's
  // field the frame shows dark (NaN: none within its range).
  void observe(double stamp_s, double sensor_share, double light_reading,
               double unlit_surface_share);

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

} // namespace drone_city_nav

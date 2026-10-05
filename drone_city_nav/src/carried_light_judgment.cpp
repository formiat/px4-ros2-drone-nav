#include "drone_city_nav/carried_light_judgment.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

// The share of its range the braking contract grants under which it reads
// the forward sensor as blind, the line of the unobservable evidence too
// (specification K12, K14): the range within a quarter metre of the margin.
constexpr double kBlindSensorShare{0.35};

} // namespace

void LightReliabilityJudgment::observe(const double stamp_s,
                                       const double sensor_share) {
  if (!std::isfinite(stamp_s) || !std::isfinite(sensor_share) || sensor_share < 0.0 ||
      (!samples_.empty() && stamp_s < samples_.back().stamp_s)) {
    return;
  }
  const bool outage = sensor_share < kBlindSensorShare;
  if (outage && outage_started_s_ < 0.0) {
    outage_started_s_ = stamp_s;
  } else if (!outage) {
    outage_started_s_ = -1.0;
  }
  samples_.push_back(Sample{.stamp_s = stamp_s, .outage = outage});
  while (samples_.size() > 1U && samples_.front().stamp_s < stamp_s - kWindowS) {
    samples_.pop_front();
  }
  const bool window_full = stamp_s - samples_.front().stamp_s >= 0.5 * kWindowS;
  unreliable_ = unreliable_ || currentOutageS() >= kLongestOutageS ||
                (window_full && outageShare() >= kOutageShare);
}

double LightReliabilityJudgment::currentOutageS() const noexcept {
  return outage_started_s_ < 0.0 || samples_.empty()
             ? 0.0
             : samples_.back().stamp_s - outage_started_s_;
}

double LightReliabilityJudgment::outageShare() const noexcept {
  // Each sample stands for the time until the next.
  double outage_s{0.0};
  double total_s{0.0};
  for (std::size_t index = 1U; index < samples_.size(); ++index) {
    const double span_s = samples_[index].stamp_s - samples_[index - 1U].stamp_s;
    total_s += span_s;
    outage_s += samples_[index - 1U].outage ? span_s : 0.0;
  }
  return total_s > 0.0 ? outage_s / total_s : 0.0;
}

} // namespace drone_city_nav

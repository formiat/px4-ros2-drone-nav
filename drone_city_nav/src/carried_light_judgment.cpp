#include "drone_city_nav/carried_light_judgment.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

// A range a centimetre short of the guaranteed one is still the lit figure.
constexpr double kLitRangeToleranceM{0.01};
constexpr double kGoalTimeMargin{9.0};
constexpr double kMinimumMeanSpeedMps{0.5};
constexpr double kGoalReserveS{20.0};

} // namespace

void LightReliabilityJudgment::observe(const double stamp_s,
                                       const double measured_range_m,
                                       const double guaranteed_range_m) {
  if (!std::isfinite(stamp_s) || !std::isfinite(measured_range_m) ||
      !std::isfinite(guaranteed_range_m) ||
      (!samples_.empty() && stamp_s < samples_.back().stamp_s)) {
    return;
  }
  const bool outage = measured_range_m < guaranteed_range_m - kLitRangeToleranceM;
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

double carriedLightGoalEstimateS(const double route_remaining_m,
                                 const double mean_speed_mps) noexcept {
  const double speed_mps = std::isfinite(mean_speed_mps)
                               ? std::max(mean_speed_mps, kMinimumMeanSpeedMps)
                               : kMinimumMeanSpeedMps;
  return kGoalTimeMargin * std::max(0.0, route_remaining_m) / speed_mps + kGoalReserveS;
}

} // namespace drone_city_nav

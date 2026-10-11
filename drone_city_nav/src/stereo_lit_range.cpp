#include "drone_city_nav/stereo_lit_range.hpp"

#include <algorithm>

namespace drone_city_nav {

double LitRangeTracker::Baseline::median() const {
  std::vector<double> sorted = ratios;
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2U);
  std::nth_element(sorted.begin(), middle, sorted.end());
  return *middle;
}

double LitRangeTracker::observe(const std::span<const double> brightness_by_metre,
                                const bool lit) {
  const std::size_t metres = std::min(kMetres, brightness_by_metre.size());
  if (metres <= kReferenceMetre || !(brightness_by_metre[kReferenceMetre] > 0.0)) {
    return static_cast<double>(metres);
  }
  const double reference = brightness_by_metre[kReferenceMetre];
  double lit_range_m = static_cast<double>(kReferenceMetre + 1U);
  bool faded = false;
  for (std::size_t metre = kReferenceMetre + 1U; metre < metres; ++metre) {
    const double brightness = brightness_by_metre[metre];
    if (!(brightness > 0.0)) {
      if (!faded) {
        lit_range_m = static_cast<double>(metre + 1U);
      }
      continue;
    }
    const double ratio = brightness / reference;
    Baseline& baseline = baselines_[metre];
    if (!faded && baseline.ratios.size() >= kMinimumBaselineFrames &&
        ratio < kFadeLine * baseline.median()) {
      faded = true;
      lit_range_m = static_cast<double>(metre);
    } else if (!faded) {
      lit_range_m = static_cast<double>(metre + 1U);
    }
    if (lit) {
      if (baseline.ratios.size() < kBaselineFrames) {
        baseline.ratios.push_back(ratio);
      } else {
        baseline.ratios[baseline.next] = ratio;
        baseline.next = (baseline.next + 1U) % kBaselineFrames;
      }
    }
  }
  return lit_range_m;
}

} // namespace drone_city_nav

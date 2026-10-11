#include "drone_city_nav/stereo_lit_range.hpp"

#include <algorithm>

namespace drone_city_nav {

void LitBaseline::observe(const double value) {
  if (values_.size() < kFrames) {
    values_.push_back(value);
  } else {
    values_[next_] = value;
    next_ = (next_ + 1U) % kFrames;
  }
}

std::optional<double> LitBaseline::median() const {
  if (values_.size() < kMinimumFrames) {
    return std::nullopt;
  }
  std::vector<double> sorted = values_;
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
    LitBaseline& baseline = baselines_[metre];
    const std::optional<double> lit_ratio = baseline.median();
    if (!faded && lit_ratio.has_value() && ratio < kFadeLine * *lit_ratio) {
      faded = true;
      lit_range_m = static_cast<double>(metre);
    } else if (!faded) {
      lit_range_m = static_cast<double>(metre + 1U);
    }
    if (lit) {
      baseline.observe(ratio);
    }
  }
  return lit_range_m;
}

} // namespace drone_city_nav

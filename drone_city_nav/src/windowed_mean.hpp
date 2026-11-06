#pragma once

#include <cstddef>
#include <deque>

namespace drone_city_nav {

// The mean of the last `length` values, summed afresh on every value: a
// smoothing whose state holds nothing older than its window. An exponential
// smoothing remembers its first value for ever, a little: a flight's
// estimator, started seconds before its first frame, and the replay of the
// flight's bag, which begins where the recorder did, smoothed the same
// barometer from different first readings and stood a centimetre apart over
// the replay's first frames (r2920; roadmap item 36, specification K62).
class WindowedMean {
public:
  explicit WindowedMean(const std::size_t length)
      : length_{length} {
  }

  // The mean over the window with `value` taken in.
  double push(const double value) {
    values_.push_back(value);
    if (values_.size() > length_) {
      values_.pop_front();
    }
    double sum = 0.0;
    for (const double held : values_) {
      sum += held;
    }
    return sum / static_cast<double>(values_.size());
  }

private:
  std::size_t length_;
  std::deque<double> values_;
};

} // namespace drone_city_nav

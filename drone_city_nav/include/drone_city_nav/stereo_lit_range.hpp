#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

// The range to which a frame's brightness follows the carried light's falloff.
// In clear air each metre of depth is lit in a fixed proportion to the metre
// nearest the vehicle, whatever the light does: a flicker dims every metre
// alike. A fade over distance, or smoke, darkens the far metres first, so a
// metre that reads under a fifth of the proportion it held over the last
// thirty seconds of lit frames is where the light stops reaching; the lit
// range ends there (technical_debt.md, Camera Perception: the frames gave no
// warning of a fading light, r800 and r906). Metres where nothing matched
// give no verdict, and the range runs on through them.
// The value a lit frame reads, as the median of the last thirty seconds of
// lit frames (225 at the pair's rate); none until ten seconds of them.
class LitBaseline {
public:
  static constexpr std::size_t kFrames{225U};
  static constexpr std::size_t kMinimumFrames{75U};

  void observe(double value);
  [[nodiscard]] std::optional<double> median() const;

private:
  std::vector<double> values_;
  std::size_t next_{0U};
};

class LitRangeTracker {
public:
  static constexpr std::size_t kMetres{8U};
  static constexpr std::size_t kReferenceMetre{2U};
  static constexpr double kFadeLine{0.2};

  // `brightness_by_metre`: the mean grey of the matched pixels in each metre
  // of depth from the first, negative where none matched. `lit` says whether
  // the frame is one the baseline may learn from. Returns the lit range in
  // metres, the whole span when nothing says less.
  [[nodiscard]] double observe(std::span<const double> brightness_by_metre, bool lit);

private:
  std::array<LitBaseline, kMetres> baselines_{};
};

} // namespace drone_city_nav

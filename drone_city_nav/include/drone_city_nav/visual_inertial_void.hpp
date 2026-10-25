#pragma once

#include <cstdint>

namespace drone_city_nav {

// The void of the camera estimator (roadmap item 31, specification K46): a
// frame with nothing to track while the pair's latest judgment, younger than
// a few of its periods, is a void frame (empty to the light's reach under a
// sound light). The void is entered by the frame itself, so that the estimate
// is in it the moment its unaided second runs out (an entry at the next frame
// left a dead-reckoning blip of a tenth to a third of a second, which the
// offboard answered with a hold, r2175), and left by the next update, through
// the aided frames whose tracks have not yet closed into one (the judgment of
// the frame they are seen in comes after them). The
// void's time is kept apart from the dead-reckoning budget: the frames were
// empty, not blind. A frame with nothing to track and no fresh judgment ends
// the void (a failed light is dead reckoning from that moment).
class VisualInertialVoid final {
public:
  static constexpr std::int64_t kWitnessFreshNs{600'000'000};

  // The pair's judgment of its latest frame, at `stamp_ns`.
  void observe(const std::int64_t stamp_ns, const bool void_frame) noexcept {
    witness_ = void_frame;
    witness_stamp_ns_ = stamp_ns;
  }

  // One frame at `stamp_ns`, the previous frame's stamp, the last update's,
  // and whether the frame has too few features to aid. Returns whether the
  // estimate is in the void after it; the void's time counts from the last
  // update.
  bool frame(std::int64_t stamp_ns, std::int64_t previous_frame_stamp_ns,
             std::int64_t last_update_stamp_ns, bool few_features) noexcept;

  // An update: the void and its time end.
  void updated() noexcept {
    in_void_ = false;
    seconds_ = 0.0;
  }

  [[nodiscard]] bool inVoid() const noexcept {
    return in_void_;
  }

  // The time since the last update the void covered.
  [[nodiscard]] double seconds() const noexcept {
    return seconds_;
  }

private:
  bool witness_{false};
  std::int64_t witness_stamp_ns_{0};
  bool in_void_{false};
  double seconds_{0.0};
};

} // namespace drone_city_nav

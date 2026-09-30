#pragma once

#include <cstdint>
#include <optional>

namespace drone_city_nav {

// The landing rung of the ladder a vehicle without sight descends (roadmap
// item 17 stage 5, specification K13). A position source that declares dead
// reckoning carries the vehicle off at 0.2 to 0.6 m/s from its first seconds
// (r856, r878, r879: a wall 2.4 to 3.5 m away met within 10 to 15 s, while the
// vehicle held and then let the autopilot land it blind). Once the source has
// dead reckoned for three seconds, the fourth of the dark that judges the
// carried light (K12), the vehicle lands under the estimate it still has; a
// source that sees again before the vehicle is down takes the flight back.
class DeadReckoningLanding final {
public:
  // One pose of the position source, at `stamp_ns`. A source that falls silent
  // after declaring dead reckoning is still dead reckoning.
  void observe(const std::int64_t stamp_ns, const bool dead_reckoning) noexcept {
    if (!dead_reckoning) {
      since_ns_.reset();
      commanded_ns_.reset();
    } else if (!since_ns_.has_value()) {
      since_ns_ = stamp_ns;
    }
  }

  [[nodiscard]] bool due(const std::int64_t now_ns) const noexcept {
    constexpr std::int64_t kDeadReckonedNs{3'000'000'000};
    return since_ns_.has_value() && now_ns - *since_ns_ >= kDeadReckonedNs;
  }

  // True when the landing is to be commanded now: while the autopilot still
  // flies the offboard setpoints, every two seconds until it has taken it.
  [[nodiscard]] bool command(const std::int64_t now_ns,
                             const bool autopilot_flies_offboard) noexcept {
    constexpr std::int64_t kRepeatNs{2'000'000'000};
    if (!due(now_ns) || !autopilot_flies_offboard ||
        (commanded_ns_.has_value() && now_ns - *commanded_ns_ < kRepeatNs)) {
      return false;
    }
    commanded_ns_ = now_ns;
    return true;
  }

private:
  std::optional<std::int64_t> since_ns_;
  std::optional<std::int64_t> commanded_ns_;
};

} // namespace drone_city_nav

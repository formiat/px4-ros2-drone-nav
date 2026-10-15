#pragma once

// Roadmap item 18 stage 4, the ladder in smoke (specification K40): a vehicle
// blind to the braking contract while the ring returns smoke from half its
// zones or more is inside a plume. Its estimator dead reckons within a second
// of the entry, so the way out is the offboard's back-out on dead reckoning
// along the heading (I3); this ladder reads the entry and the exit. Out of
// the first plume it holds where it sees for the smoke to move or thin, then
// takes its goal again; out of a second it gives the goal up for the start,
// as any goal is given up (roadmap item 19): a camera vehicle cannot map a
// plume it cannot see into, so no proof of item 19 closes a way smoke
// closes, and a way entered twice and twice backed out of is the measure
// that it is closed (r1533: the proof's flood exhausted its budget through
// the unknown beyond the plume while the vehicle wandered 800 s).

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct SmokeLadderInput {
  // The frame's share of its range under the contract's blind line (K12).
  bool blind{false};
  // The share of the ring's zones returning smoke (K38).
  double smoke_share{0.0};
  bool position_fresh{false};
  Point3 position{};
  std::int64_t now_ns{0};
};

enum class SmokeLadderAction : std::uint8_t {
  kNone,
  // Out of the first plume, seeing again: hold here.
  kHold,
  // The hold over: the mission's goal again.
  kRestore,
  // Out of a second plume: give the goal up for the start.
  kGiveUp,
};

class SmokeLadder final {
public:
  static constexpr double kZoneShare{0.5};
  static constexpr double kClearShare{0.1};
  static constexpr double kHoldS{30.0};

  [[nodiscard]] SmokeLadderAction observe(const SmokeLadderInput& input);

  // Plumes entered so far.
  [[nodiscard]] std::size_t entries() const noexcept {
    return entries_;
  }

  [[nodiscard]] bool holding() const noexcept {
    return state_ == State::kHolding;
  }

private:
  enum class State : std::uint8_t {
    kNone,
    kInside,
    kHolding,
    kGivenUp
  };
  State state_{State::kNone};
  std::size_t entries_{0U};
  std::int64_t hold_since_ns_{0};
};

} // namespace drone_city_nav

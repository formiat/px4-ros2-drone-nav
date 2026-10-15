#pragma once

// Roadmap item 18 stage 4, the ladder in smoke (specification K40): a vehicle
// blind to the braking contract while the ring returns smoke from half its
// zones or more is inside a plume. It retreats along its own trail to the
// last place it saw from, holds there for the smoke to move or thin, and if
// its range has not come back gives the goal up for the start, as any goal
// is given up (roadmap item 19), while a position source lasts. The way out
// is the way it came (I3); the plume's marks lift when seen through (K10).

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
  // Fly to seeingPlace(): the last place the vehicle saw from.
  kRetreat,
  // Hold there.
  kHold,
  // The range has not come back: give the goal up for the start.
  kGiveUp,
  // The range came back after a retreat: the mission's goal again.
  kRestore,
};

class SmokeLadder final {
public:
  static constexpr double kZoneShare{0.5};
  static constexpr double kClearShare{0.1};
  static constexpr double kBlindS{2.0};
  static constexpr double kSeeingReachM{2.0};
  static constexpr double kHoldS{30.0};

  [[nodiscard]] SmokeLadderAction observe(const SmokeLadderInput& input);

  [[nodiscard]] const std::optional<Point3>& seeingPlace() const noexcept {
    return seeing_place_;
  }

  [[nodiscard]] bool active() const noexcept {
    return state_ != State::kNone;
  }

private:
  enum class State : std::uint8_t {
    kNone,
    kRetreating,
    kHolding,
    kGivenUp
  };
  State state_{State::kNone};
  std::optional<Point3> seeing_place_;
  std::int64_t smoke_since_ns_{0};
  std::int64_t hold_since_ns_{0};
};

} // namespace drone_city_nav

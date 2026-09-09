#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cmath>
#include <optional>

namespace drone_city_nav {

// The position the offboard's local hold pins the vehicle to while nothing
// owns its motion. The pin survives a horizon that takes the vehicle nowhere:
// it is judged against the vehicle when the hold is next needed, kept while
// the vehicle still stands within the stationary tolerance of it and replaced
// only once a horizon has actually carried the vehicle away. Re-pinned at
// every revocation instead, a run of short-lived horizons walked the hold a
// few centimetres at a time, and one recorded flight ratcheted twenty
// centimetres into a wall it had come to rest a hull's width from, while
// supposedly holding.
class LocalHoldPin final {
public:
  // Takes the hold at `position`, keeping a pin the vehicle still stands on
  // and replacing one it has left. True when the pin is new.
  bool acquire(const Point3& position) noexcept {
    if (pin_.has_value() && !lease_active_ &&
        std::hypot(position.x - pin_->x, position.y - pin_->y, position.z - pin_->z) >
            kStationaryExecutionHoldPositionToleranceM) {
      pin_.reset();
    }
    lease_active_ = true;
    if (pin_.has_value()) {
      return false;
    }
    pin_ = position;
    return true;
  }

  // The hold is no longer the setpoint in force -- a horizon took the vehicle
  // over. The pin is kept for the next acquire to judge.
  void release() noexcept {
    lease_active_ = false;
  }

  void reset() noexcept {
    pin_.reset();
    lease_active_ = false;
  }

  [[nodiscard]] const std::optional<Point3>& pin() const noexcept {
    return pin_;
  }

private:
  std::optional<Point3> pin_;
  bool lease_active_{false};
};

} // namespace drone_city_nav

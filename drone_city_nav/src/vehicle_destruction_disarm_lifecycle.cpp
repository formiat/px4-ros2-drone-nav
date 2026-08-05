#include "drone_city_nav/vehicle_destruction_disarm_lifecycle.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

VehicleDestructionDisarmLifecycle::VehicleDestructionDisarmLifecycle(
    const VehicleDestructionDisarmConfig& config)
    : config_{config} {
  if (!std::isfinite(config_.retry_period_s) || !(config_.retry_period_s > 0.0)) {
    throw std::invalid_argument{"invalid vehicle destruction disarm configuration"};
  }
}

void VehicleDestructionDisarmLifecycle::latch(const std::int64_t stamp_ns) noexcept {
  if (latched_) {
    return;
  }
  latched_ = true;
  latched_stamp_ns_ = std::max<std::int64_t>(0, stamp_ns);
}

VehicleDestructionDisarmUpdate
VehicleDestructionDisarmLifecycle::update(const std::int64_t stamp_ns,
                                          const bool vehicle_status_seen,
                                          const bool armed) noexcept {
  VehicleDestructionDisarmUpdate result{.latched = latched_, .confirmed = confirmed_};
  if (!latched_ || confirmed_) {
    return result;
  }
  if (command_sent_ && vehicle_status_seen && !armed) {
    confirmed_ = true;
    result.confirmed = true;
    return result;
  }
  const auto retry_period_ns =
      static_cast<std::int64_t>(config_.retry_period_s * 1.0e9);
  const bool first_request = !command_sent_;
  const bool retry_due = command_sent_ && stamp_ns >= last_request_stamp_ns_ &&
                         stamp_ns - last_request_stamp_ns_ >= retry_period_ns;
  if (first_request || retry_due) {
    command_sent_ = true;
    last_request_stamp_ns_ = std::max(stamp_ns, latched_stamp_ns_);
    result.force_disarm_requested = true;
  }
  return result;
}

bool VehicleDestructionDisarmLifecycle::latched() const noexcept {
  return latched_;
}

} // namespace drone_city_nav

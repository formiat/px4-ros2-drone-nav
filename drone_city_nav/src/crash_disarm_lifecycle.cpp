#include "drone_city_nav/crash_disarm_lifecycle.hpp"

#include <algorithm>
#include <stdexcept>

namespace drone_city_nav {

CrashDisarmLifecycle::CrashDisarmLifecycle(const CrashDisarmLifecycleConfig& config)
    : config_{config} {
  if (!(config_.retry_period_s > 0.0)) {
    throw std::invalid_argument{"crash disarm retry period must be positive"};
  }
}

void CrashDisarmLifecycle::latch(const std::int64_t stamp_ns) noexcept {
  if (latched_) {
    return;
  }
  latched_ = true;
  last_command_stamp_ns_ = stamp_ns;
}

CrashDisarmUpdate CrashDisarmLifecycle::update(const std::int64_t stamp_ns,
                                               const bool arming_state_known,
                                               const bool armed) noexcept {
  CrashDisarmUpdate result{.latched = latched_, .confirmed = confirmed_};
  if (!latched_ || confirmed_) {
    return result;
  }
  if (command_sent_ && arming_state_known && !armed) {
    confirmed_ = true;
    result.confirmed = true;
    return result;
  }

  const auto retry_period_ns =
      static_cast<std::int64_t>(config_.retry_period_s * 1.0e9);
  const bool clock_regressed = stamp_ns < last_command_stamp_ns_;
  const bool retry_due =
      !command_sent_ || clock_regressed ||
      stamp_ns - last_command_stamp_ns_ >= std::max<std::int64_t>(1, retry_period_ns);
  if (retry_due) {
    command_sent_ = true;
    last_command_stamp_ns_ = stamp_ns;
    result.force_disarm_requested = true;
  }
  return result;
}

bool CrashDisarmLifecycle::latched() const noexcept {
  return latched_;
}

} // namespace drone_city_nav

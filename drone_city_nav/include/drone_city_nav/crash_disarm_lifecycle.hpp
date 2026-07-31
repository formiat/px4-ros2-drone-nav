#pragma once

#include <cstdint>

namespace drone_city_nav {

struct CrashDisarmLifecycleConfig {
  double retry_period_s{0.2};
};

struct CrashDisarmUpdate {
  bool latched{false};
  bool confirmed{false};
  bool force_disarm_requested{false};
};

class CrashDisarmLifecycle {
public:
  explicit CrashDisarmLifecycle(const CrashDisarmLifecycleConfig& config = {});

  void latch(std::int64_t stamp_ns) noexcept;

  [[nodiscard]] CrashDisarmUpdate update(std::int64_t stamp_ns, bool arming_state_known,
                                         bool armed) noexcept;

  [[nodiscard]] bool latched() const noexcept;

private:
  CrashDisarmLifecycleConfig config_{};
  std::int64_t last_command_stamp_ns_{0};
  bool latched_{false};
  bool command_sent_{false};
  bool confirmed_{false};
};

} // namespace drone_city_nav

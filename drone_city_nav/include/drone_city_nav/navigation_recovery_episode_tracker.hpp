#pragma once

#include <cstdint>
#include <mutex>

namespace drone_city_nav {

// Counts recovery episodes rather than retries. Repeated planning attempts in
// one active recovery interval advance health accounting only once.
class NavigationRecoveryEpisodeTracker final {
public:
  [[nodiscard]] bool observe(std::uint64_t mission_epoch, bool recovery_active);
  [[nodiscard]] std::uint64_t sequence() const;

private:
  mutable std::mutex mutex_;
  std::uint64_t latest_mission_epoch_{0U};
  std::uint64_t active_recovery_mission_epoch_{0U};
  std::uint64_t sequence_{0U};
};

} // namespace drone_city_nav

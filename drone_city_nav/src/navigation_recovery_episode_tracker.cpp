#include "drone_city_nav/navigation_recovery_episode_tracker.hpp"

#include <limits>

namespace drone_city_nav {

bool NavigationRecoveryEpisodeTracker::observe(const std::uint64_t mission_epoch,
                                               const bool recovery_active) {
  if (mission_epoch == 0U) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (mission_epoch < latest_mission_epoch_) {
    return false;
  }
  if (mission_epoch > latest_mission_epoch_) {
    latest_mission_epoch_ = mission_epoch;
    active_recovery_mission_epoch_ = 0U;
  }
  if (!recovery_active) {
    active_recovery_mission_epoch_ = 0U;
    return false;
  }
  if (active_recovery_mission_epoch_ == mission_epoch) {
    return false;
  }
  active_recovery_mission_epoch_ = mission_epoch;
  if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  ++sequence_;
  return true;
}

std::uint64_t NavigationRecoveryEpisodeTracker::sequence() const {
  const std::scoped_lock lock{mutex_};
  return sequence_;
}

} // namespace drone_city_nav

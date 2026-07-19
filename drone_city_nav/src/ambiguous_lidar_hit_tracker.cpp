#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <algorithm>

namespace drone_city_nav {

AmbiguousLidarHitTracker::AmbiguousLidarHitTracker(
    AmbiguousLidarHitTrackerConfig config)
    : config_{config} {
  configure(config);
}

void AmbiguousLidarHitTracker::configure(AmbiguousLidarHitTrackerConfig config) {
  config_ = config;
  config_.required_independent_scans =
      std::max<std::size_t>(1U, config_.required_independent_scans);
  config_.max_scan_gap_ns = std::max<std::int64_t>(1, config_.max_scan_gap_ns);
  config_.retention_ns = std::max(config_.max_scan_gap_ns, config_.retention_ns);
  clear();
}

AmbiguousLidarHitConfirmation
AmbiguousLidarHitTracker::observe(const GridIndex cell,
                                  const std::int64_t scan_stamp_ns) {
  if (scan_stamp_ns <= 0) {
    return {};
  }
  prune(scan_stamp_ns);
  Evidence& evidence = evidence_[cellKey(cell)];
  bool new_scan_vote = false;
  if (evidence.last_scan_stamp_ns == 0 ||
      scan_stamp_ns - evidence.last_scan_stamp_ns > config_.max_scan_gap_ns) {
    evidence = Evidence{scan_stamp_ns, scan_stamp_ns, 1U};
    new_scan_vote = true;
  } else if (scan_stamp_ns > evidence.last_scan_stamp_ns) {
    evidence.last_scan_stamp_ns = scan_stamp_ns;
    ++evidence.independent_scans;
    new_scan_vote = true;
  }
  return AmbiguousLidarHitConfirmation{
      .independent_scans = evidence.independent_scans,
      .confirmed = evidence.independent_scans >= config_.required_independent_scans,
      .new_scan_vote = new_scan_vote,
  };
}

void AmbiguousLidarHitTracker::clear() noexcept {
  evidence_.clear();
}

std::size_t AmbiguousLidarHitTracker::candidateCount() const noexcept {
  return evidence_.size();
}

std::uint64_t AmbiguousLidarHitTracker::cellKey(const GridIndex cell) noexcept {
  return static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.x)) << 32U |
         static_cast<std::uint32_t>(cell.y);
}

void AmbiguousLidarHitTracker::prune(const std::int64_t scan_stamp_ns) {
  std::erase_if(evidence_, [this, scan_stamp_ns](const auto& item) {
    return scan_stamp_ns - item.second.last_scan_stamp_ns > config_.retention_ns;
  });
}

} // namespace drone_city_nav

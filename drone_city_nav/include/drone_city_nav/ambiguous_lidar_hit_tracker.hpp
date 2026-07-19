#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace drone_city_nav {

struct AmbiguousLidarHitTrackerConfig {
  std::size_t required_independent_scans{3U};
  std::int64_t max_scan_gap_ns{500'000'000};
  std::int64_t retention_ns{2'000'000'000};
};

struct AmbiguousLidarHitConfirmation {
  std::size_t independent_scans{0U};
  bool confirmed{false};
  bool new_scan_vote{false};
};

class AmbiguousLidarHitTracker {
public:
  explicit AmbiguousLidarHitTracker(AmbiguousLidarHitTrackerConfig config = {});

  void configure(AmbiguousLidarHitTrackerConfig config);
  [[nodiscard]] AmbiguousLidarHitConfirmation observe(GridIndex cell,
                                                      std::int64_t scan_stamp_ns);
  void clear() noexcept;
  [[nodiscard]] std::size_t candidateCount() const noexcept;

private:
  struct Evidence {
    std::int64_t first_scan_stamp_ns{0};
    std::int64_t last_scan_stamp_ns{0};
    std::size_t independent_scans{0U};
  };

  [[nodiscard]] static std::uint64_t cellKey(GridIndex cell) noexcept;
  void prune(std::int64_t scan_stamp_ns);

  AmbiguousLidarHitTrackerConfig config_{};
  std::unordered_map<std::uint64_t, Evidence> evidence_;
};

} // namespace drone_city_nav

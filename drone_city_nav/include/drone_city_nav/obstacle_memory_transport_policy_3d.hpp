#pragma once

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

struct ObstacleMemoryTransportPolicy3DConfig {
  double minimum_snapshot_period_s{10.0};
  double maximum_snapshot_period_s{30.0};
  double snapshot_rebase_dirty_ratio{0.75};
};

enum class ObstacleMemoryTransportDecision3D : std::uint8_t {
  kNone,
  kSnapshot,
  kDelta,
};

struct ObstacleMemoryTransportPolicy3DInput {
  std::int64_t now_steady_ns{0};
  std::uint64_t revision{0U};
  std::size_t current_chunk_count{0U};
  std::size_t dirty_chunk_count{0U};
  bool full_reset{false};
};

class ObstacleMemoryTransportPolicy3D final {
public:
  explicit ObstacleMemoryTransportPolicy3D(
      const ObstacleMemoryTransportPolicy3DConfig& config = {});

  [[nodiscard]] ObstacleMemoryTransportDecision3D
  decide(const ObstacleMemoryTransportPolicy3DInput& input) const noexcept;
  void recordSnapshot(std::uint64_t revision, std::int64_t now_steady_ns) noexcept;
  [[nodiscard]] std::uint64_t baseSnapshotRevision() const noexcept;

private:
  ObstacleMemoryTransportPolicy3DConfig config_{};
  std::uint64_t base_snapshot_revision_{0U};
  std::int64_t last_snapshot_steady_ns_{0};
};

} // namespace drone_city_nav

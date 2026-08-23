#include "drone_city_nav/obstacle_memory_transport_policy_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

ObstacleMemoryTransportPolicy3D::ObstacleMemoryTransportPolicy3D(
    const ObstacleMemoryTransportPolicy3DConfig& config)
    : config_{config} {
  if (!std::isfinite(config_.minimum_snapshot_period_s) ||
      !std::isfinite(config_.maximum_snapshot_period_s) ||
      !std::isfinite(config_.snapshot_rebase_dirty_ratio) ||
      config_.minimum_snapshot_period_s < 0.0 ||
      config_.maximum_snapshot_period_s < config_.minimum_snapshot_period_s ||
      config_.snapshot_rebase_dirty_ratio <= 0.0 ||
      config_.snapshot_rebase_dirty_ratio > 1.0) {
    throw std::invalid_argument{"invalid 3D obstacle-memory transport policy"};
  }
}

ObstacleMemoryTransportDecision3D ObstacleMemoryTransportPolicy3D::decide(
    const ObstacleMemoryTransportPolicy3DInput& input) const noexcept {
  if (input.revision == 0U) {
    return ObstacleMemoryTransportDecision3D::kNone;
  }
  if (input.full_reset || base_snapshot_revision_ == 0U) {
    return ObstacleMemoryTransportDecision3D::kSnapshot;
  }
  if (input.revision <= base_snapshot_revision_ || input.dirty_chunk_count == 0U) {
    return ObstacleMemoryTransportDecision3D::kNone;
  }

  const std::int64_t elapsed_ns =
      std::max<std::int64_t>(0, input.now_steady_ns - last_snapshot_steady_ns_);
  const double elapsed_s = static_cast<double>(elapsed_ns) * 1.0e-9;
  const double dirty_ratio = input.current_chunk_count == 0U
                                 ? 1.0
                                 : static_cast<double>(input.dirty_chunk_count) /
                                       static_cast<double>(input.current_chunk_count);
  const bool maximum_age_reached = elapsed_s >= config_.maximum_snapshot_period_s;
  const bool economical_rebase = elapsed_s >= config_.minimum_snapshot_period_s &&
                                 dirty_ratio >= config_.snapshot_rebase_dirty_ratio;
  return maximum_age_reached || economical_rebase
             ? ObstacleMemoryTransportDecision3D::kSnapshot
             : ObstacleMemoryTransportDecision3D::kDelta;
}

void ObstacleMemoryTransportPolicy3D::recordSnapshot(
    const std::uint64_t revision, const std::int64_t now_steady_ns) noexcept {
  base_snapshot_revision_ = revision;
  last_snapshot_steady_ns_ = now_steady_ns;
}

std::uint64_t ObstacleMemoryTransportPolicy3D::baseSnapshotRevision() const noexcept {
  return base_snapshot_revision_;
}

} // namespace drone_city_nav

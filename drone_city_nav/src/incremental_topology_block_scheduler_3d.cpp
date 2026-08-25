#include "drone_city_nav/incremental_topology_block_scheduler_3d.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <tuple>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 blockCenter(const IncrementalTopologyBlockIndex3D block,
                                 const GridBounds3D& bounds,
                                 const int block_size_cells) noexcept {
  const double extent_m = static_cast<double>(block_size_cells) * bounds.resolution_m;
  return {bounds.origin_x + (static_cast<double>(block.x) + 0.5) * extent_m,
          bounds.origin_y + (static_cast<double>(block.y) + 0.5) * extent_m,
          bounds.origin_z + (static_cast<double>(block.z) + 0.5) * extent_m};
}

struct BlockPriorityMetrics3D {
  IncrementalTopologyBlockPriorityTier3D tier{
      IncrementalTopologyBlockPriorityTier3D::kBacklog};
  double primary_score{0.0};
  double secondary_score{0.0};
};

[[nodiscard]] BlockPriorityMetrics3D
buildPriorityMetrics(const IncrementalTopologyBlockIndex3D block,
                     const GridBounds3D& bounds, const int block_size_cells,
                     const IncrementalTopologyBuildPriority3D& priority) noexcept {
  const Point3 center = blockCenter(block, bounds, block_size_cells);
  const Vec3 offset{center.x - priority.position.x, center.y - priority.position.y,
                    center.z - priority.position.z};
  const Vec3 direction{priority.target.x - priority.position.x,
                       priority.target.y - priority.position.y,
                       priority.target.z - priority.position.z};
  const double direction_m =
      std::sqrt(direction.x * direction.x + direction.y * direction.y +
                direction.z * direction.z);
  const double offset_squared =
      offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
  const double distance_m = std::sqrt(offset_squared);
  if (distance_m <= priority.local_radius_m) {
    return {.tier = IncrementalTopologyBlockPriorityTier3D::kLocalSafety,
            .primary_score = distance_m};
  }
  if (direction_m <= 1.0e-6) {
    return {.primary_score = distance_m};
  }
  const double forward_m =
      (offset.x * direction.x + offset.y * direction.y + offset.z * direction.z) /
      direction_m;
  const double cross_track_m =
      std::sqrt(std::max(0.0, offset_squared - forward_m * forward_m));
  if (forward_m >= 0.0 && forward_m <= priority.forward_corridor_lookahead_m &&
      cross_track_m <= priority.forward_corridor_radius_m) {
    return {.tier = IncrementalTopologyBlockPriorityTier3D::kForwardCorridor,
            .primary_score = forward_m,
            .secondary_score = cross_track_m};
  }
  return {.primary_score = distance_m,
          .secondary_score = forward_m < 0.0 ? -forward_m : cross_track_m};
}

} // namespace

IncrementalTopologyProgressWatchdog3D::IncrementalTopologyProgressWatchdog3D(
    const IncrementalTopologyProgressWatchdog3DConfig& config)
    : config_{config} {
  if (config_.initial_backoff.count() <= 0 ||
      config_.maximum_backoff < config_.initial_backoff ||
      config_.warning_streak == 0U) {
    throw std::invalid_argument{"invalid topology progress watchdog configuration"};
  }
}

IncrementalTopologyProgressWatchdog3DState
IncrementalTopologyProgressWatchdog3D::observe(
    const std::size_t pending_blocks, const std::size_t rebuilt_blocks) noexcept {
  if (pending_blocks == 0U || rebuilt_blocks > 0U) {
    reset();
    return {};
  }
  ++zero_progress_streak_;
  const std::size_t shift = std::min<std::size_t>(zero_progress_streak_ - 1U, 16U);
  using BackoffRep = std::chrono::milliseconds::rep;
  const BackoffRep factor = static_cast<BackoffRep>(std::uint64_t{1U} << shift);
  const BackoffRep maximum_count = config_.maximum_backoff.count();
  const BackoffRep initial_count = config_.initial_backoff.count();
  const std::chrono::milliseconds scaled{
      initial_count > maximum_count / factor ? maximum_count : initial_count * factor};
  return IncrementalTopologyProgressWatchdog3DState{
      .zero_progress_streak = zero_progress_streak_,
      .retry_backoff = std::min(scaled, config_.maximum_backoff),
      .stalled = zero_progress_streak_ >= config_.warning_streak,
  };
}

void IncrementalTopologyProgressWatchdog3D::reset() noexcept {
  zero_progress_streak_ = 0U;
}

IncrementalTopologyBlockPriorityTier3D incrementalTopologyBlockPriorityTier3D(
    const IncrementalTopologyBlockIndex3D block, const GridBounds3D& bounds,
    const int block_size_cells,
    const IncrementalTopologyBuildPriority3D& priority) noexcept {
  return buildPriorityMetrics(block, bounds, block_size_cells, priority).tier;
}

std::vector<IncrementalTopologyBlockIndex3D> selectIncrementalTopologyBlocks3D(
    const std::span<const IncrementalTopologyBlockIndex3D> pending_blocks,
    const std::size_t maximum_blocks, const GridBounds3D& bounds,
    const int block_size_cells,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority) {
  if (block_size_cells <= 0 ||
      (priority.has_value() &&
       (!std::isfinite(priority->local_radius_m) || priority->local_radius_m <= 0.0 ||
        !std::isfinite(priority->forward_corridor_radius_m) ||
        priority->forward_corridor_radius_m <= 0.0 ||
        !std::isfinite(priority->forward_corridor_lookahead_m) ||
        priority->forward_corridor_lookahead_m <= 0.0))) {
    throw std::invalid_argument{"invalid topology block priority request"};
  }
  std::vector<IncrementalTopologyBlockIndex3D> ordered{pending_blocks.begin(),
                                                       pending_blocks.end()};
  std::ranges::sort(ordered, [&](const IncrementalTopologyBlockIndex3D first,
                                 const IncrementalTopologyBlockIndex3D second) {
    if (priority.has_value()) {
      const BlockPriorityMetrics3D first_metrics =
          buildPriorityMetrics(first, bounds, block_size_cells, *priority);
      const BlockPriorityMetrics3D second_metrics =
          buildPriorityMetrics(second, bounds, block_size_cells, *priority);
      if (first_metrics.tier != second_metrics.tier) {
        return first_metrics.tier < second_metrics.tier;
      }
      if (std::abs(first_metrics.primary_score - second_metrics.primary_score) >
          1.0e-9) {
        return first_metrics.primary_score < second_metrics.primary_score;
      }
      if (std::abs(first_metrics.secondary_score - second_metrics.secondary_score) >
          1.0e-9) {
        return first_metrics.secondary_score < second_metrics.secondary_score;
      }
    }
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  ordered.resize(std::min(maximum_blocks, ordered.size()));
  return ordered;
}

} // namespace drone_city_nav

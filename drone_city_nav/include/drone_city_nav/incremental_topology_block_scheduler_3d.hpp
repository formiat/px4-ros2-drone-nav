#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class IncrementalTopologyBlockPriorityTier3D : std::uint8_t {
  kLocalSafety,
  kForwardCorridor,
  kBacklog,
};

struct IncrementalTopologyProgressWatchdog3DConfig {
  std::chrono::milliseconds initial_backoff{5};
  std::chrono::milliseconds maximum_backoff{250};
  std::size_t warning_streak{3U};
};

struct IncrementalTopologyProgressWatchdog3DState {
  std::size_t zero_progress_streak{0U};
  std::chrono::milliseconds retry_backoff{0};
  bool stalled{false};
};

class IncrementalTopologyProgressWatchdog3D final {
public:
  explicit IncrementalTopologyProgressWatchdog3D(
      const IncrementalTopologyProgressWatchdog3DConfig& config = {});

  [[nodiscard]] IncrementalTopologyProgressWatchdog3DState
  observe(std::size_t pending_blocks, std::size_t rebuilt_blocks) noexcept;
  void reset() noexcept;

private:
  IncrementalTopologyProgressWatchdog3DConfig config_{};
  std::size_t zero_progress_streak_{0U};
};

[[nodiscard]] IncrementalTopologyBlockPriorityTier3D
incrementalTopologyBlockPriorityTier3D(
    IncrementalTopologyBlockIndex3D block, const GridBounds3D& bounds,
    int block_size_cells, const IncrementalTopologyBuildPriority3D& priority) noexcept;

[[nodiscard]] std::vector<IncrementalTopologyBlockIndex3D>
selectIncrementalTopologyBlocks3D(
    std::span<const IncrementalTopologyBlockIndex3D> pending_blocks,
    std::size_t maximum_blocks, const GridBounds3D& bounds, int block_size_cells,
    const std::optional<IncrementalTopologyBuildPriority3D>& priority);

} // namespace drone_city_nav

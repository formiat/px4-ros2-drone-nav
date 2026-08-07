#include "drone_city_nav/raw_obstacle_delta.hpp"

#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace drone_city_nav {
namespace {

constexpr std::uint32_t kMaximumChunkSizeCells{256U};

[[nodiscard]] bool validChunkSize(const std::uint32_t chunk_size_cells) noexcept {
  return chunk_size_cells > 0U && chunk_size_cells <= kMaximumChunkSizeCells;
}

[[nodiscard]] std::size_t divideRoundUp(const std::size_t value,
                                        const std::size_t divisor) noexcept {
  return (value + divisor - 1U) / divisor;
}

[[nodiscard]] bool sameGeometry(const GridBounds& bounds,
                                const nav_msgs::msg::MapMetaData& info) noexcept {
  return info.width == static_cast<std::uint32_t>(bounds.width_cells) &&
         info.height == static_cast<std::uint32_t>(bounds.height_cells) &&
         std::abs(static_cast<double>(info.resolution) - bounds.resolution_m) <=
             1.0e-9 &&
         std::abs(info.origin.position.x - bounds.origin_x) <= 1.0e-9 &&
         std::abs(info.origin.position.y - bounds.origin_y) <= 1.0e-9;
}

[[nodiscard]] CellState decodedCellState(const std::int8_t value) noexcept {
  if (value >= static_cast<std::int8_t>(CellState::kOccupied)) {
    return CellState::kOccupied;
  }
  if (value == static_cast<std::int8_t>(CellState::kFree)) {
    return CellState::kFree;
  }
  return CellState::kUnknown;
}

void setCellState(OccupancyGrid2D& grid, const GridIndex cell, const CellState state) {
  switch (state) {
    case CellState::kUnknown:
      grid.setUnknown(cell);
      return;
    case CellState::kFree:
      grid.setFree(cell);
      return;
    case CellState::kOccupied:
      grid.setOccupied(cell);
      return;
  }
}

[[nodiscard]] nav_msgs::msg::MapMetaData mapInfo(const GridBounds& bounds) {
  nav_msgs::msg::MapMetaData info;
  info.resolution = static_cast<float>(bounds.resolution_m);
  info.width = static_cast<std::uint32_t>(bounds.width_cells);
  info.height = static_cast<std::uint32_t>(bounds.height_cells);
  info.origin.position.x = bounds.origin_x;
  info.origin.position.y = bounds.origin_y;
  info.origin.orientation.w = 1.0;
  return info;
}

} // namespace

std::vector<std::uint32_t>
rawObstacleChunkIndices(const GridBounds& bounds,
                        const std::span<const std::size_t> changed_cell_indices,
                        const std::uint32_t chunk_size_cells) {
  if (!validChunkSize(chunk_size_cells) || bounds.width_cells <= 0 ||
      bounds.height_cells <= 0) {
    return {};
  }
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t cell_count = width * static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunk_size = static_cast<std::size_t>(chunk_size_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  std::vector<std::uint32_t> result;
  result.reserve(changed_cell_indices.size());
  for (const std::size_t cell_index : changed_cell_indices) {
    if (cell_index >= cell_count) {
      continue;
    }
    const std::size_t x = cell_index % width;
    const std::size_t y = cell_index / width;
    const std::size_t chunk_index = (y / chunk_size) * chunks_x + x / chunk_size;
    if (chunk_index <= std::numeric_limits<std::uint32_t>::max()) {
      result.push_back(static_cast<std::uint32_t>(chunk_index));
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

std::optional<msg::RawObstacleDelta> makeRawObstacleDelta(
    const OccupancyGrid2D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id,
    const std::uint64_t base_snapshot_revision,
    const std::uint64_t obstacle_snapshot_revision,
    const std::uint64_t risk_policy_fingerprint, const double risk_critical_distance_m,
    const double risk_preferred_distance_m, const std::uint32_t chunk_size_cells,
    const std::span<const std::uint32_t> chunk_indices) {
  if (!validChunkSize(chunk_size_cells) || producer_instance_id == 0U ||
      base_snapshot_revision == 0U ||
      obstacle_snapshot_revision <= base_snapshot_revision) {
    return std::nullopt;
  }
  const GridBounds& bounds = grid.bounds();
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunk_size = static_cast<std::size_t>(chunk_size_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  const std::size_t chunks_y = divideRoundUp(height, chunk_size);
  const std::size_t chunk_count = chunks_x * chunks_y;
  const std::size_t cells_per_chunk = chunk_size * chunk_size;
  if (chunk_indices.size() >
      std::numeric_limits<std::size_t>::max() / cells_per_chunk) {
    return std::nullopt;
  }

  msg::RawObstacleDelta message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.base_snapshot_revision = base_snapshot_revision;
  message.obstacle_snapshot_revision = obstacle_snapshot_revision;
  message.risk_policy_fingerprint = risk_policy_fingerprint;
  message.risk_critical_distance_m = risk_critical_distance_m;
  message.risk_preferred_distance_m = risk_preferred_distance_m;
  message.map_info = mapInfo(bounds);
  message.chunk_size_cells = chunk_size_cells;
  message.chunk_indices.assign(chunk_indices.begin(), chunk_indices.end());
  message.chunk_data.reserve(chunk_indices.size() * cells_per_chunk);
  std::unordered_set<std::uint32_t> unique_chunks;
  for (const std::uint32_t chunk_index : chunk_indices) {
    if (static_cast<std::size_t>(chunk_index) >= chunk_count ||
        !unique_chunks.insert(chunk_index).second) {
      return std::nullopt;
    }
    const std::size_t chunk_x = static_cast<std::size_t>(chunk_index) % chunks_x;
    const std::size_t chunk_y = static_cast<std::size_t>(chunk_index) / chunks_x;
    for (std::size_t local_y = 0U; local_y < chunk_size; ++local_y) {
      for (std::size_t local_x = 0U; local_x < chunk_size; ++local_x) {
        const std::size_t x = chunk_x * chunk_size + local_x;
        const std::size_t y = chunk_y * chunk_size + local_y;
        const CellState state =
            x < width && y < height
                ? grid.state(GridIndex{static_cast<int>(x), static_cast<int>(y)})
                : CellState::kUnknown;
        message.chunk_data.push_back(static_cast<std::int8_t>(state));
      }
    }
  }
  return message;
}

RawObstacleGridUpdate
RawObstacleDeltaAccumulator::apply(const msg::RawObstacleSnapshot& snapshot) {
  if (snapshot.producer_instance_id == 0U ||
      snapshot.obstacle_snapshot_revision == 0U) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }
  if (state_.occupancy &&
      snapshot.producer_instance_id == state_.producer_instance_id &&
      snapshot.obstacle_snapshot_revision <= state_.obstacle_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kStale};
  }
  RawOccupancyGridFromRosResult converted =
      rawOccupancyGridFromRos(snapshot.grid, RawOccupancyGridFromRosConfig{100, 0});
  if (!converted.grid) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }
  state_ = RawObstacleGridState{
      .producer_instance_id = snapshot.producer_instance_id,
      .base_snapshot_revision = snapshot.obstacle_snapshot_revision,
      .obstacle_snapshot_revision = snapshot.obstacle_snapshot_revision,
      .risk_policy_fingerprint = snapshot.risk_policy_fingerprint,
      .risk_critical_distance_m = snapshot.risk_critical_distance_m,
      .risk_preferred_distance_m = snapshot.risk_preferred_distance_m,
      .occupancy = std::make_shared<const OccupancyGrid2D>(std::move(*converted.grid)),
  };
  return {.state = state_, .status = RawObstacleGridUpdateStatus::kAccepted};
}

RawObstacleGridUpdate
RawObstacleDeltaAccumulator::apply(const msg::RawObstacleDelta& delta) {
  if (!state_.occupancy || delta.producer_instance_id != state_.producer_instance_id ||
      delta.base_snapshot_revision != state_.base_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kBaseUnavailable};
  }
  if (delta.obstacle_snapshot_revision <= state_.obstacle_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kStale};
  }
  const GridBounds& bounds = state_.occupancy->bounds();
  if (!validChunkSize(delta.chunk_size_cells) ||
      !sameGeometry(bounds, delta.map_info) ||
      delta.risk_policy_fingerprint != state_.risk_policy_fingerprint ||
      delta.risk_critical_distance_m != state_.risk_critical_distance_m ||
      delta.risk_preferred_distance_m != state_.risk_preferred_distance_m) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }
  const std::size_t chunk_size = static_cast<std::size_t>(delta.chunk_size_cells);
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  const std::size_t chunks_y = divideRoundUp(height, chunk_size);
  const std::size_t chunk_count = chunks_x * chunks_y;
  const std::size_t cells_per_chunk = chunk_size * chunk_size;
  if (delta.chunk_indices.empty() ||
      delta.chunk_indices.size() >
          std::numeric_limits<std::size_t>::max() / cells_per_chunk ||
      delta.chunk_data.size() != delta.chunk_indices.size() * cells_per_chunk) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }

  OccupancyGrid2D updated = *state_.occupancy;
  std::unordered_set<std::uint32_t> unique_chunks;
  for (std::size_t message_chunk = 0U; message_chunk < delta.chunk_indices.size();
       ++message_chunk) {
    const std::uint32_t chunk_index = delta.chunk_indices[message_chunk];
    if (static_cast<std::size_t>(chunk_index) >= chunk_count ||
        !unique_chunks.insert(chunk_index).second) {
      return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
    }
    const std::size_t chunk_x = static_cast<std::size_t>(chunk_index) % chunks_x;
    const std::size_t chunk_y = static_cast<std::size_t>(chunk_index) / chunks_x;
    const std::size_t data_offset = message_chunk * cells_per_chunk;
    for (std::size_t local_y = 0U; local_y < chunk_size; ++local_y) {
      for (std::size_t local_x = 0U; local_x < chunk_size; ++local_x) {
        const std::size_t x = chunk_x * chunk_size + local_x;
        const std::size_t y = chunk_y * chunk_size + local_y;
        if (x >= width || y >= height) {
          continue;
        }
        const std::size_t data_index = data_offset + local_y * chunk_size + local_x;
        setCellState(updated, GridIndex{static_cast<int>(x), static_cast<int>(y)},
                     decodedCellState(delta.chunk_data[data_index]));
      }
    }
  }
  state_.obstacle_snapshot_revision = delta.obstacle_snapshot_revision;
  state_.occupancy = std::make_shared<const OccupancyGrid2D>(std::move(updated));
  return {.state = state_, .status = RawObstacleGridUpdateStatus::kAccepted};
}

const RawObstacleGridState& RawObstacleDeltaAccumulator::state() const noexcept {
  return state_;
}

const char*
rawObstacleGridUpdateStatusName(const RawObstacleGridUpdateStatus status) noexcept {
  switch (status) {
    case RawObstacleGridUpdateStatus::kAccepted:
      return "accepted";
    case RawObstacleGridUpdateStatus::kStale:
      return "stale";
    case RawObstacleGridUpdateStatus::kBaseUnavailable:
      return "base_unavailable";
    case RawObstacleGridUpdateStatus::kInvalidMessage:
      return "invalid_message";
  }
  return "unknown";
}

} // namespace drone_city_nav

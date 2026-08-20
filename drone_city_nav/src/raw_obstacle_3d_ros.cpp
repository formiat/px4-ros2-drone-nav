#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include "drone_city_nav/msg/observed_obstacle_chunk3_d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_set>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validBounds(const GridBounds3D& bounds) noexcept {
  return std::isfinite(bounds.origin_x) && std::isfinite(bounds.origin_y) &&
         std::isfinite(bounds.origin_z) && std::isfinite(bounds.resolution_m) &&
         bounds.resolution_m > 0.0 && bounds.width_cells > 0 &&
         bounds.height_cells > 0 && bounds.depth_cells > 0;
}

template<typename Message>
[[nodiscard]] std::optional<GridBounds3D> boundsFromMessage(const Message& message) {
  if (message.chunk_size_cells !=
          static_cast<std::uint32_t>(ObservedOccupancyGrid3D::kChunkSize) ||
      message.width_cells == 0U || message.height_cells == 0U ||
      message.depth_cells == 0U ||
      message.width_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      message.height_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      message.depth_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }
  const GridBounds3D bounds{
      .origin_x = message.origin_x_m,
      .origin_y = message.origin_y_m,
      .origin_z = message.origin_z_m,
      .resolution_m = message.resolution_m,
      .width_cells = static_cast<int>(message.width_cells),
      .height_cells = static_cast<int>(message.height_cells),
      .depth_cells = static_cast<int>(message.depth_cells),
  };
  return validBounds(bounds) ? std::optional<GridBounds3D>{bounds} : std::nullopt;
}

template<typename Message>
void setGeometry(Message& message, const GridBounds3D& bounds) {
  message.origin_x_m = bounds.origin_x;
  message.origin_y_m = bounds.origin_y;
  message.origin_z_m = bounds.origin_z;
  message.resolution_m = bounds.resolution_m;
  message.width_cells = static_cast<std::uint32_t>(bounds.width_cells);
  message.height_cells = static_cast<std::uint32_t>(bounds.height_cells);
  message.depth_cells = static_cast<std::uint32_t>(bounds.depth_cells);
  message.chunk_size_cells =
      static_cast<std::uint32_t>(ObservedOccupancyGrid3D::kChunkSize);
}

[[nodiscard]] msg::ObservedObstacleChunk3D
makeChunk(const OccupancyChunkIndex3D index,
          const ObservedOccupancyGrid3D::Chunk* const chunk) {
  msg::ObservedObstacleChunk3D message;
  message.x = index.x;
  message.y = index.y;
  message.z = index.z;
  if (chunk != nullptr) {
    message.observed_words = chunk->observed;
    message.occupied_words = chunk->occupied;
  }
  return message;
}

[[nodiscard]] bool chunkIndexInBounds(const OccupancyChunkIndex3D index,
                                      const GridBounds3D& bounds) noexcept {
  if (index.x < 0 || index.y < 0 || index.z < 0) {
    return false;
  }
  const int chunk_size = ObservedOccupancyGrid3D::kChunkSize;
  return index.x * chunk_size < bounds.width_cells &&
         index.y * chunk_size < bounds.height_cells &&
         index.z * chunk_size < bounds.depth_cells;
}

[[nodiscard]] bool
applyChunks(ObservedOccupancyGrid3D& grid,
            const std::span<const msg::ObservedObstacleChunk3D> chunks) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> unique;
  for (const msg::ObservedObstacleChunk3D& message : chunks) {
    const OccupancyChunkIndex3D index{message.x, message.y, message.z};
    if (!chunkIndexInBounds(index, grid.bounds()) || !unique.insert(index).second) {
      return false;
    }
    ObservedOccupancyGrid3D::Chunk chunk;
    chunk.observed = message.observed_words;
    chunk.occupied = message.occupied_words;
    static_cast<void>(grid.replaceChunk(index, chunk));
  }
  return true;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double tolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

} // namespace

msg::RawObstacleSnapshot3D makeRawObstacleSnapshot3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id, const std::uint64_t revision) {
  msg::RawObstacleSnapshot3D message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.obstacle_snapshot_revision = revision;
  setGeometry(message, grid.bounds());
  message.chunks.reserve(grid.chunks().size());
  for (const auto& [index, chunk] : grid.chunks()) {
    message.chunks.push_back(makeChunk(index, &chunk));
  }
  std::ranges::sort(message.chunks, [](const auto& first, const auto& second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return message;
}

msg::RawObstacleDelta3D makeRawObstacleDelta3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id,
    const std::uint64_t base_snapshot_revision, const std::uint64_t revision,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks) {
  msg::RawObstacleDelta3D message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.base_snapshot_revision = base_snapshot_revision;
  message.obstacle_snapshot_revision = revision;
  setGeometry(message, grid.bounds());
  message.chunks.reserve(dirty_chunks.size());
  for (const OccupancyChunkIndex3D index : dirty_chunks) {
    const auto found = grid.chunks().find(index);
    message.chunks.push_back(
        makeChunk(index, found == grid.chunks().end() ? nullptr : &found->second));
  }
  return message;
}

RawObstacleGridUpdate3D
RawObstacleDeltaAccumulator3D::apply(const msg::RawObstacleSnapshot3D& snapshot) {
  const std::optional<GridBounds3D> bounds = boundsFromMessage(snapshot);
  if (!bounds.has_value() || snapshot.producer_instance_id == 0U ||
      snapshot.obstacle_snapshot_revision == 0U) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
  }
  if (state_.occupancy &&
      snapshot.producer_instance_id == state_.producer_instance_id &&
      snapshot.obstacle_snapshot_revision <= state_.obstacle_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kStale};
  }
  ObservedOccupancyGrid3D grid{*bounds};
  if (!applyChunks(grid, snapshot.chunks)) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
  }
  state_ = RawObstacleGridState3D{
      .producer_instance_id = snapshot.producer_instance_id,
      .base_snapshot_revision = snapshot.obstacle_snapshot_revision,
      .obstacle_snapshot_revision = snapshot.obstacle_snapshot_revision,
      .occupancy = std::make_shared<const ObservedOccupancyGrid3D>(std::move(grid)),
  };
  return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kAccepted};
}

RawObstacleGridUpdate3D
RawObstacleDeltaAccumulator3D::apply(const msg::RawObstacleDelta3D& delta) {
  const std::optional<GridBounds3D> bounds = boundsFromMessage(delta);
  if (!state_.occupancy || !bounds.has_value() ||
      delta.producer_instance_id != state_.producer_instance_id ||
      delta.base_snapshot_revision != state_.base_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kBaseUnavailable};
  }
  if (delta.obstacle_snapshot_revision <= state_.obstacle_snapshot_revision) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kStale};
  }
  if (delta.chunks.empty() || !sameBounds(*bounds, state_.occupancy->bounds())) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
  }
  ObservedOccupancyGrid3D updated = *state_.occupancy;
  if (!applyChunks(updated, delta.chunks)) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
  }
  state_.obstacle_snapshot_revision = delta.obstacle_snapshot_revision;
  state_.occupancy =
      std::make_shared<const ObservedOccupancyGrid3D>(std::move(updated));
  return {.state = state_, .status = RawObstacleGridUpdateStatus3D::kAccepted};
}

const RawObstacleGridState3D& RawObstacleDeltaAccumulator3D::state() const noexcept {
  return state_;
}

const char*
rawObstacleGridUpdateStatus3DName(const RawObstacleGridUpdateStatus3D status) noexcept {
  switch (status) {
    case RawObstacleGridUpdateStatus3D::kAccepted:
      return "accepted";
    case RawObstacleGridUpdateStatus3D::kStale:
      return "stale";
    case RawObstacleGridUpdateStatus3D::kBaseUnavailable:
      return "base_unavailable";
    case RawObstacleGridUpdateStatus3D::kInvalidMessage:
      return "invalid_message";
  }
  return "unknown";
}

} // namespace drone_city_nav

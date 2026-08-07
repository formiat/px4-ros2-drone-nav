#pragma once

#include "drone_city_nav/msg/raw_obstacle_delta.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <std_msgs/msg/header.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RawObstacleGridState {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t base_snapshot_revision{0U};
  std::uint64_t obstacle_snapshot_revision{0U};
  std::uint64_t risk_policy_fingerprint{0U};
  double risk_critical_distance_m{0.0};
  double risk_preferred_distance_m{0.0};
  std::shared_ptr<const OccupancyGrid2D> occupancy;
};

enum class RawObstacleGridUpdateStatus : std::uint8_t {
  kAccepted,
  kStale,
  kBaseUnavailable,
  kInvalidMessage,
};

struct RawObstacleGridUpdate {
  RawObstacleGridState state{};
  RawObstacleGridUpdateStatus status{RawObstacleGridUpdateStatus::kInvalidMessage};

  [[nodiscard]] bool accepted() const noexcept {
    return status == RawObstacleGridUpdateStatus::kAccepted;
  }
};

[[nodiscard]] std::vector<std::uint32_t>
rawObstacleChunkIndices(const GridBounds& bounds,
                        std::span<const std::size_t> changed_cell_indices,
                        std::uint32_t chunk_size_cells);

[[nodiscard]] std::optional<msg::RawObstacleDelta> makeRawObstacleDelta(
    const OccupancyGrid2D& grid, const std_msgs::msg::Header& header,
    std::uint64_t producer_instance_id, std::uint64_t base_snapshot_revision,
    std::uint64_t obstacle_snapshot_revision, std::uint64_t risk_policy_fingerprint,
    double risk_critical_distance_m, double risk_preferred_distance_m,
    std::uint32_t chunk_size_cells, std::span<const std::uint32_t> chunk_indices);

class RawObstacleDeltaAccumulator final {
public:
  [[nodiscard]] RawObstacleGridUpdate apply(const msg::RawObstacleSnapshot& snapshot);
  [[nodiscard]] RawObstacleGridUpdate apply(const msg::RawObstacleDelta& delta);
  [[nodiscard]] const RawObstacleGridState& state() const noexcept;

private:
  RawObstacleGridState state_{};
};

[[nodiscard]] const char*
rawObstacleGridUpdateStatusName(RawObstacleGridUpdateStatus status) noexcept;

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RawObstacleGridState3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t base_snapshot_revision{0U};
  std::uint64_t obstacle_snapshot_revision{0U};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy;
};

enum class RawObstacleGridUpdateStatus3D : std::uint8_t {
  kAccepted,
  kStale,
  kBaseUnavailable,
  kInvalidMessage,
};

struct RawObstacleGridUpdate3D {
  RawObstacleGridState3D state{};
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  RawObstacleGridUpdateStatus3D status{RawObstacleGridUpdateStatus3D::kInvalidMessage};
  bool full_reset{false};

  [[nodiscard]] bool accepted() const noexcept {
    return status == RawObstacleGridUpdateStatus3D::kAccepted;
  }
};

[[nodiscard]] msg::RawObstacleSnapshot3D
makeRawObstacleSnapshot3D(const ObservedOccupancyGrid3D& grid,
                          const std_msgs::msg::Header& header,
                          std::uint64_t producer_instance_id, std::uint64_t revision);

[[nodiscard]] msg::RawObstacleDelta3D makeRawObstacleDelta3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    std::uint64_t producer_instance_id, std::uint64_t base_snapshot_revision,
    std::uint64_t revision, std::span<const OccupancyChunkIndex3D> dirty_chunks);

class RawObstacleDeltaAccumulator3D final {
public:
  [[nodiscard]] RawObstacleGridUpdate3D
  apply(const msg::RawObstacleSnapshot3D& snapshot);
  [[nodiscard]] RawObstacleGridUpdate3D apply(const msg::RawObstacleDelta3D& delta);
  [[nodiscard]] const RawObstacleGridState3D& state() const noexcept;

private:
  RawObstacleGridState3D state_{};
};

[[nodiscard]] const char*
rawObstacleGridUpdateStatus3DName(RawObstacleGridUpdateStatus3D status) noexcept;

} // namespace drone_city_nav

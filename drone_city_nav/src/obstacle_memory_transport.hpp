#pragma once

#include "drone_city_nav/msg/obstacle_memory_provenance.hpp"
#include "drone_city_nav/msg/obstacle_memory_snapshot.hpp"
#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>

namespace drone_city_nav {

class ObstacleMemoryTransport final {
public:
  ObstacleMemoryTransport(rclcpp::Node& node, std::string frame_id, bool use_static_map,
                          std::optional<OccupancyGrid2D> static_grid,
                          double risk_critical_distance_m,
                          double risk_preferred_distance_m);

  void publish(const OccupancyGrid2D& raw_grid,
               const std::unordered_map<std::size_t, MemoryCellProvenance>& provenance,
               const GridCellCounts& cell_counts, RawGridChanges changes,
               const rclcpp::Time& stamp);

  ObstacleMemoryTransport(const ObstacleMemoryTransport&) = delete;
  ObstacleMemoryTransport& operator=(const ObstacleMemoryTransport&) = delete;
  ObstacleMemoryTransport(ObstacleMemoryTransport&&) = delete;
  ObstacleMemoryTransport& operator=(ObstacleMemoryTransport&&) = delete;

private:
  [[nodiscard]] bool
  publishRawWorldSnapshot(const nav_msgs::msg::OccupancyGrid& memory_grid,
                          std::uint64_t producer_instance_id,
                          std::uint64_t snapshot_revision);

  rclcpp::Node& node_;
  std::string frame_id_;
  std::optional<OccupancyGrid2D> static_grid_;
  double debug_publish_period_s_{1.0};
  double diagnostic_period_s_{5.0};
  double maximum_assembly_time_ms_{100.0};
  double maximum_publish_interval_ms_{1500.0};
  double maximum_assembly_since_report_ms_{0.0};
  double maximum_publish_interval_since_report_ms_{0.0};
  double risk_critical_distance_m_{1.0};
  double risk_preferred_distance_m_{6.0};
  std::size_t maximum_serialized_bytes_{4'500'000U};
  std::uint32_t raw_delta_chunk_size_cells_{32U};
  std::uint64_t sequence_{0U};
  std::uint64_t producer_instance_id_{0U};
  std::uint64_t raw_base_revision_{0U};
  std::uint64_t risk_policy_fingerprint_{0U};
  std::uint64_t snapshot_publications_{0U};
  std::uint64_t status_publications_{0U};
  std::uint64_t raw_snapshot_publications_{0U};
  std::uint64_t raw_delta_publications_{0U};
  std::uint64_t debug_publications_{0U};
  std::uint64_t publications_at_last_diagnostic_{0U};
  std::int64_t last_snapshot_publish_stamp_ns_{0};
  std::int64_t last_status_publish_stamp_ns_{0};
  std::int64_t last_debug_publish_stamp_ns_{0};
  std::int64_t last_diagnostic_stamp_ns_{0};
  bool use_static_map_{true};
  std::set<std::uint32_t> dirty_chunks_since_base_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_memory_3d_pointcloud_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryProvenance>::SharedPtr provenance_pub_;
  rclcpp::Publisher<msg::ObstacleMemorySnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::RawObstacleSnapshot>::SharedPtr raw_obstacle_snapshot_pub_;
  rclcpp::Publisher<msg::RawObstacleDelta>::SharedPtr raw_obstacle_delta_pub_;
};

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>

namespace drone_city_nav {

class ObstacleMemoryTransport3D final {
public:
  ObstacleMemoryTransport3D(rclcpp::Node& node, std::string frame_id);

  void publish(const ObservedOccupancyGrid3D& grid,
               const ObstacleMemory3DChanges& changes, const rclcpp::Time& stamp,
               bool publish_debug);

private:
  rclcpp::Node& node_;
  std::string frame_id_;
  double snapshot_period_s_{1.0};
  std::size_t debug_stride_{1U};
  std::uint64_t producer_instance_id_{0U};
  std::uint64_t sequence_{0U};
  std::uint64_t base_snapshot_revision_{0U};
  std::int64_t last_snapshot_stamp_ns_{0};
  std::int64_t last_debug_stamp_ns_{0};
  std::set<OccupancyChunkIndex3D,
           bool (*)(const OccupancyChunkIndex3D&, const OccupancyChunkIndex3D&)>
      dirty_chunks_since_base_;

  rclcpp::Publisher<msg::RawObstacleSnapshot3D>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::RawObstacleDelta3D>::SharedPtr delta_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr memory_cloud_pub_;
};

} // namespace drone_city_nav

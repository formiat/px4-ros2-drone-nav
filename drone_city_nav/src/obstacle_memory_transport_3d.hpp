#pragma once

#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"
#include "drone_city_nav/obstacle_memory_transport_policy_3d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <stop_token>
#include <string>
#include <thread>

namespace drone_city_nav {

class ObstacleMemoryTransport3D final {
public:
  ObstacleMemoryTransport3D(rclcpp::Node& node, std::string frame_id,
                            bool gazebo_aligned_rviz_axes_swapped);
  ~ObstacleMemoryTransport3D();

  ObstacleMemoryTransport3D(const ObstacleMemoryTransport3D&) = delete;
  ObstacleMemoryTransport3D& operator=(const ObstacleMemoryTransport3D&) = delete;
  ObstacleMemoryTransport3D(ObstacleMemoryTransport3D&&) = delete;
  ObstacleMemoryTransport3D& operator=(ObstacleMemoryTransport3D&&) = delete;

  [[nodiscard]] bool enqueue(const ObservedOccupancyGrid3D& grid,
                             const ObstacleMemory3DChanges& changes,
                             const rclcpp::Time& stamp, bool publish_debug);

private:
  struct PendingUpdate {
    ObservedOccupancyGrid3D grid;
    ObstacleMemory3DChanges changes;
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point enqueued_at;
    std::uint64_t coalesced_updates{0U};
    bool publish_debug{false};
  };

  void workerLoop(std::stop_token stop_token);
  void publishUpdate(PendingUpdate update);

  rclcpp::Node& node_;
  std::string frame_id_;
  bool gazebo_aligned_rviz_axes_swapped_{true};
  ObstacleMemoryTransportPolicy3D policy_;
  double update_period_s_{0.5};
  double debug_period_s_{1.0};
  std::size_t debug_stride_{1U};
  std::uint64_t producer_instance_id_{0U};
  std::uint64_t sequence_{0U};
  std::int64_t last_debug_steady_ns_{0};
  std::chrono::steady_clock::time_point last_publish_time_{};
  std::set<OccupancyChunkIndex3D,
           bool (*)(const OccupancyChunkIndex3D&, const OccupancyChunkIndex3D&)>
      dirty_chunks_since_base_;

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::optional<PendingUpdate> pending_update_;
  std::atomic<std::uint64_t> total_coalesced_updates_{0U};
  std::jthread worker_;

  rclcpp::Publisher<msg::RawObstacleSnapshot3D>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::RawObstacleDelta3D>::SharedPtr delta_pub_;
  rclcpp::Publisher<msg::ObstacleMemoryStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr memory_cloud_pub_;
};

} // namespace drone_city_nav

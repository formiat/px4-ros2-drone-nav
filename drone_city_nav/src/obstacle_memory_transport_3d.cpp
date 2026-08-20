#include "obstacle_memory_transport_3d.hpp"

#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include <algorithm>
#include <cinttypes>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool chunkLess(const OccupancyChunkIndex3D& first,
                             const OccupancyChunkIndex3D& second) {
  return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
}

} // namespace

ObstacleMemoryTransport3D::ObstacleMemoryTransport3D(rclcpp::Node& node,
                                                     std::string frame_id)
    : node_{node},
      frame_id_{std::move(frame_id)},
      dirty_chunks_since_base_{chunkLess} {
  snapshot_period_s_ = std::clamp(
      node_.declare_parameter<double>("obstacle_memory_3d_snapshot_period_s", 1.0), 0.1,
      60.0);
  debug_stride_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
      node_.declare_parameter<std::int64_t>("obstacle_memory_3d_debug_stride", 1), 1,
      1000));
  snapshot_pub_ = node_.create_publisher<msg::RawObstacleSnapshot3D>(
      node_.declare_parameter<std::string>("raw_obstacle_snapshot_3d_topic",
                                           "/drone_city_nav/raw_obstacle_snapshot_3d"),
      rclcpp::QoS{1}.reliable().transient_local());
  delta_pub_ = node_.create_publisher<msg::RawObstacleDelta3D>(
      node_.declare_parameter<std::string>("raw_obstacle_delta_3d_topic",
                                           "/drone_city_nav/raw_obstacle_delta_3d"),
      rclcpp::QoS{1}.reliable().transient_local());
  status_pub_ = node_.create_publisher<msg::ObstacleMemoryStatus>(
      node_.declare_parameter<std::string>("obstacle_memory_status_topic",
                                           "/drone_city_nav/obstacle_memory_status"),
      rclcpp::QoS{1}.reliable().transient_local());
  memory_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      node_.declare_parameter<std::string>(
          "raw_memory_3d_pointcloud_topic",
          "/drone_city_nav/raw_memory_obstacle_points_3d"),
      rclcpp::QoS{1}.reliable().transient_local());
}

void ObstacleMemoryTransport3D::publish(const ObservedOccupancyGrid3D& grid,
                                        const ObstacleMemory3DChanges& changes,
                                        const rclcpp::Time& stamp,
                                        const bool publish_debug) {
  const std::int64_t stamp_ns = stamp.nanoseconds();
  if (producer_instance_id_ == 0U) {
    producer_instance_id_ =
        static_cast<std::uint64_t>(std::max<std::int64_t>(1, stamp_ns));
  }
  ++sequence_;
  if (changes.full_reset) {
    base_snapshot_revision_ = 0U;
    dirty_chunks_since_base_.clear();
  }
  dirty_chunks_since_base_.insert(changes.dirty_chunks.begin(),
                                  changes.dirty_chunks.end());
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;

  const bool snapshot_due =
      changes.revision > 0U && changes.revision != base_snapshot_revision_ &&
      (base_snapshot_revision_ == 0U || last_snapshot_stamp_ns_ <= 0 ||
       stamp_ns - last_snapshot_stamp_ns_ >=
           static_cast<std::int64_t>(snapshot_period_s_ * 1.0e9));
  bool snapshot_published{false};
  bool delta_published{false};
  if (snapshot_due) {
    snapshot_pub_->publish(makeRawObstacleSnapshot3D(
        grid, header, producer_instance_id_, changes.revision));
    base_snapshot_revision_ = changes.revision;
    dirty_chunks_since_base_.clear();
    last_snapshot_stamp_ns_ = stamp_ns;
    snapshot_published = true;
  } else if (base_snapshot_revision_ > 0U &&
             changes.revision > base_snapshot_revision_ &&
             !dirty_chunks_since_base_.empty()) {
    const std::vector<OccupancyChunkIndex3D> dirty{dirty_chunks_since_base_.begin(),
                                                   dirty_chunks_since_base_.end()};
    delta_pub_->publish(makeRawObstacleDelta3D(grid, header, producer_instance_id_,
                                               base_snapshot_revision_,
                                               changes.revision, dirty));
    delta_published = true;
  }

  if (publish_debug && (last_debug_stamp_ns_ <= 0 ||
                        stamp_ns - last_debug_stamp_ns_ >=
                            static_cast<std::int64_t>(snapshot_period_s_ * 1.0e9))) {
    memory_cloud_pub_->publish(buildObservedOccupancyPointCloud3D(
        grid, header.stamp, frame_id_, debug_stride_));
    last_debug_stamp_ns_ = stamp_ns;
  }

  msg::ObstacleMemoryStatus status;
  status.header = header;
  status.producer_instance_id = producer_instance_id_;
  status.sequence = sequence_;
  status.occupied_cell_count = static_cast<std::uint64_t>(grid.occupiedVoxelCount());
  status.raw_snapshot_published = snapshot_published;
  status.raw_delta_published = delta_published;
  status.full_snapshot_published = snapshot_published;
  status_pub_->publish(status);
  RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "ONLINE_OCCUPANCY3D_UPDATE producer=%" PRIu64 " sequence=%" PRIu64
      " revision=%" PRIu64 " known=%zu free=%zu occupied=%zu chunks=%zu "
      "dirty=%zu snapshot=%s delta=%s",
      producer_instance_id_, sequence_, changes.revision, grid.knownVoxelCount(),
      grid.freeVoxelCount(), grid.occupiedVoxelCount(), grid.chunks().size(),
      changes.dirty_chunks.size(), snapshot_published ? "true" : "false",
      delta_published ? "true" : "false");
}

} // namespace drone_city_nav

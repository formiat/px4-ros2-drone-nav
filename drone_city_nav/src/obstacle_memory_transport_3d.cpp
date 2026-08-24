#include "obstacle_memory_transport_3d.hpp"

#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/producer_instance_id.hpp"
#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <ranges>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool chunkLess(const OccupancyChunkIndex3D& first,
                             const OccupancyChunkIndex3D& second) {
  return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
}

void mergeDirtyChunks(std::vector<OccupancyChunkIndex3D>& destination,
                      const std::vector<OccupancyChunkIndex3D>& source) {
  destination.insert(destination.end(), source.begin(), source.end());
  std::ranges::sort(destination, chunkLess);
  const auto duplicates = std::ranges::unique(destination);
  destination.erase(duplicates.begin(), duplicates.end());
}

[[nodiscard]] std::int64_t steadyNowNanoseconds() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} // namespace

ObstacleMemoryTransport3D::ObstacleMemoryTransport3D(rclcpp::Node& node,
                                                     std::string frame_id)
    : node_{node},
      frame_id_{std::move(frame_id)},
      policy_{ObstacleMemoryTransportPolicy3DConfig{
          .minimum_snapshot_period_s =
              std::clamp(node_.declare_parameter<double>(
                             "obstacle_memory_3d_snapshot_minimum_period_s", 10.0),
                         0.0, 300.0),
          .maximum_snapshot_period_s =
              std::clamp(node_.declare_parameter<double>(
                             "obstacle_memory_3d_snapshot_maximum_period_s", 30.0),
                         0.1, 600.0),
          .snapshot_rebase_dirty_ratio =
              std::clamp(node_.declare_parameter<double>(
                             "obstacle_memory_3d_snapshot_rebase_dirty_ratio", 0.75),
                         0.05, 1.0),
      }},
      producer_instance_id_{createRawObstacleProducerInstanceId()},
      dirty_chunks_since_base_{chunkLess} {
  const double update_rate_hz = std::clamp(
      node_.declare_parameter<double>("obstacle_memory_3d_transport_rate_hz", 2.0), 0.1,
      20.0);
  update_period_s_ = 1.0 / update_rate_hz;
  debug_period_s_ = std::clamp(
      node_.declare_parameter<double>("obstacle_memory_3d_debug_period_s", 1.0), 0.1,
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
      rclcpp::QoS{1}.best_effort().transient_local());
  status_pub_ = node_.create_publisher<msg::ObstacleMemoryStatus>(
      node_.declare_parameter<std::string>("obstacle_memory_status_topic",
                                           "/drone_city_nav/obstacle_memory_status"),
      rclcpp::QoS{1}.reliable().transient_local());
  memory_cloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      node_.declare_parameter<std::string>(
          "raw_memory_3d_pointcloud_topic",
          "/drone_city_nav/raw_memory_obstacle_points_3d"),
      rclcpp::QoS{1}.best_effort().transient_local());
  worker_ = std::jthread([this](const std::stop_token token) { workerLoop(token); });

  RCLCPP_INFO(node_.get_logger(),
              "3D obstacle-memory transport ready: rate=%.2fHz "
              "snapshot_policy=adaptive debug_period=%.2fs "
              "delta_qos=best_effort cumulative=true",
              update_rate_hz, debug_period_s_);
}

ObstacleMemoryTransport3D::~ObstacleMemoryTransport3D() {
  if (worker_.joinable()) {
    worker_.request_stop();
    queue_condition_.notify_all();
    worker_.join();
  }
}

bool ObstacleMemoryTransport3D::enqueue(const ObservedOccupancyGrid3D& grid,
                                        const ObstacleMemory3DChanges& changes,
                                        const rclcpp::Time& stamp,
                                        const bool publish_debug) {
  PendingUpdate update{.grid = grid,
                       .changes = changes,
                       .stamp = stamp,
                       .enqueued_at = std::chrono::steady_clock::now(),
                       .coalesced_updates = 0U,
                       .publish_debug = publish_debug};
  bool replaced{false};
  {
    const std::scoped_lock lock{queue_mutex_};
    if (pending_update_.has_value()) {
      replaced = true;
      update.changes.full_reset =
          update.changes.full_reset || pending_update_->changes.full_reset;
      mergeDirtyChunks(update.changes.dirty_chunks,
                       pending_update_->changes.dirty_chunks);
      update.coalesced_updates = pending_update_->coalesced_updates + 1U;
      update.publish_debug = update.publish_debug || pending_update_->publish_debug;
    }
    pending_update_ = std::move(update);
  }
  if (replaced) {
    total_coalesced_updates_.fetch_add(1U, std::memory_order_relaxed);
  }
  queue_condition_.notify_one();
  return replaced;
}

void ObstacleMemoryTransport3D::workerLoop(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::optional<PendingUpdate> update;
    {
      std::unique_lock lock{queue_mutex_};
      queue_condition_.wait(lock, [this, &stop_token]() {
        return stop_token.stop_requested() || pending_update_.has_value();
      });
      if (stop_token.stop_requested()) {
        return;
      }
      if (last_publish_time_ != std::chrono::steady_clock::time_point{}) {
        const auto next_publish =
            last_publish_time_ + std::chrono::duration<double>{update_period_s_};
        queue_condition_.wait_until(lock, next_publish, [&stop_token]() {
          return stop_token.stop_requested();
        });
        if (stop_token.stop_requested()) {
          return;
        }
      }
      update = std::move(pending_update_);
      pending_update_.reset();
    }
    if (!update.has_value()) {
      continue;
    }
    try {
      publishUpdate(std::move(*update));
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_.get_logger(),
                   "ONLINE_OCCUPANCY3D_TRANSPORT failed=true error='%s'", error.what());
    }
    last_publish_time_ = std::chrono::steady_clock::now();
  }
}

void ObstacleMemoryTransport3D::publishUpdate(PendingUpdate update) {
  const auto started = std::chrono::steady_clock::now();
  ++sequence_;
  if (update.changes.full_reset) {
    dirty_chunks_since_base_.clear();
  }
  dirty_chunks_since_base_.insert(update.changes.dirty_chunks.begin(),
                                  update.changes.dirty_chunks.end());

  std_msgs::msg::Header header;
  header.stamp = update.stamp;
  header.frame_id = frame_id_;
  const std::int64_t now_steady_ns = steadyNowNanoseconds();
  const ObstacleMemoryTransportDecision3D decision =
      policy_.decide(ObstacleMemoryTransportPolicy3DInput{
          .now_steady_ns = now_steady_ns,
          .revision = update.changes.revision,
          .current_chunk_count = update.grid.chunks().size(),
          .dirty_chunk_count = dirty_chunks_since_base_.size(),
          .full_reset = update.changes.full_reset,
      });

  bool snapshot_published{false};
  bool delta_published{false};
  std::size_t published_chunks{0U};
  double assembly_ms{0.0};
  double publish_ms{0.0};
  if (decision == ObstacleMemoryTransportDecision3D::kSnapshot) {
    const auto assembly_started = std::chrono::steady_clock::now();
    msg::RawObstacleSnapshot3D message = makeRawObstacleSnapshot3D(
        update.grid, header, producer_instance_id_, update.changes.revision);
    assembly_ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - assembly_started)
                      .count();
    published_chunks = message.chunks.size();
    const auto publish_started = std::chrono::steady_clock::now();
    snapshot_pub_->publish(message);
    publish_ms = std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - publish_started)
                     .count();
    policy_.recordSnapshot(update.changes.revision, now_steady_ns);
    dirty_chunks_since_base_.clear();
    snapshot_published = true;
  } else if (decision == ObstacleMemoryTransportDecision3D::kDelta) {
    const std::vector<OccupancyChunkIndex3D> dirty{dirty_chunks_since_base_.begin(),
                                                   dirty_chunks_since_base_.end()};
    const auto assembly_started = std::chrono::steady_clock::now();
    msg::RawObstacleDelta3D message = makeRawObstacleDelta3D(
        update.grid, header, producer_instance_id_, policy_.baseSnapshotRevision(),
        update.changes.revision, dirty);
    assembly_ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - assembly_started)
                      .count();
    published_chunks = message.chunks.size();
    const auto publish_started = std::chrono::steady_clock::now();
    delta_pub_->publish(message);
    publish_ms = std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - publish_started)
                     .count();
    delta_published = true;
  }

  const bool debug_cloud_due =
      update.publish_debug && (last_debug_steady_ns_ <= 0 ||
                               now_steady_ns - last_debug_steady_ns_ >=
                                   static_cast<std::int64_t>(debug_period_s_ * 1.0e9));
  if (debug_cloud_due) {
    memory_cloud_pub_->publish(buildObservedOccupancyPointCloud3D(
        update.grid, header.stamp, frame_id_, debug_stride_));
    last_debug_steady_ns_ = now_steady_ns;
  }

  msg::ObstacleMemoryStatus status;
  status.header = header;
  status.producer_instance_id = producer_instance_id_;
  status.sequence = sequence_;
  status.occupied_cell_count =
      static_cast<std::uint64_t>(update.grid.occupiedVoxelCount());
  status.raw_snapshot_published = snapshot_published;
  status.raw_delta_published = delta_published;
  status.full_snapshot_published = snapshot_published;
  status_pub_->publish(status);

  const double queue_age_ms =
      std::chrono::duration<double, std::milli>(started - update.enqueued_at).count();
  const double total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "ONLINE_OCCUPANCY3D_UPDATE producer=%" PRIu64 " sequence=%" PRIu64
      " revision=%" PRIu64 " known=%zu free=%zu occupied=%zu chunks=%zu "
      "dirty_since_base=%zu published_chunks=%zu snapshot=%s delta=%s "
      "debug_cloud=%s queue_age_ms=%.3f assembly_ms=%.3f publish_ms=%.3f "
      "total_ms=%.3f coalesced_update=%" PRIu64 " coalesced_total=%" PRIu64,
      producer_instance_id_, sequence_, update.changes.revision,
      update.grid.knownVoxelCount(), update.grid.freeVoxelCount(),
      update.grid.occupiedVoxelCount(), update.grid.chunks().size(),
      dirty_chunks_since_base_.size(), published_chunks,
      snapshot_published ? "true" : "false", delta_published ? "true" : "false",
      debug_cloud_due ? "true" : "false", queue_age_ms, assembly_ms, publish_ms,
      total_ms, update.coalesced_updates,
      total_coalesced_updates_.load(std::memory_order_relaxed));
}

} // namespace drone_city_nav

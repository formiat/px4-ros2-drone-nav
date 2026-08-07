#include "obstacle_memory_transport.hpp"

#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/msg/obstacle_memory_provenance.hpp"
#include "drone_city_nav/msg/obstacle_memory_snapshot.hpp"
#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <utility>

#include "obstacle_memory_node_helpers.hpp"
#include "raw_world_snapshot.hpp"

namespace drone_city_nav {

ObstacleMemoryTransport::ObstacleMemoryTransport(
    rclcpp::Node& node, std::string frame_id, const bool use_static_map,
    std::optional<OccupancyGrid2D> static_grid, const double risk_critical_distance_m,
    const double risk_preferred_distance_m)
    : node_{node},
      frame_id_{std::move(frame_id)},
      static_grid_{std::move(static_grid)},
      risk_critical_distance_m_{risk_critical_distance_m},
      risk_preferred_distance_m_{risk_preferred_distance_m},
      use_static_map_{use_static_map} {
  debug_publish_period_s_ = std::clamp(
      node_.declare_parameter<double>("obstacle_memory_debug_publish_period_s", 1.0),
      0.0, 60.0);
  diagnostic_period_s_ =
      std::clamp(node_.declare_parameter<double>(
                     "obstacle_memory_snapshot_diagnostic_period_s", 5.0),
                 0.1, 60.0);
  maximum_serialized_bytes_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
      node_.declare_parameter<std::int64_t>(
          "obstacle_memory_snapshot_max_serialized_bytes", 4'500'000),
      1, 100'000'000));
  maximum_assembly_time_ms_ =
      std::clamp(node_.declare_parameter<double>(
                     "obstacle_memory_snapshot_max_assembly_time_ms", 100.0),
                 0.1, 10'000.0);
  maximum_publish_interval_ms_ =
      std::clamp(node_.declare_parameter<double>(
                     "obstacle_memory_snapshot_max_publish_interval_ms", 1500.0),
                 1.0, 60'000.0);

  raw_grid_pub_ = node_.create_publisher<nav_msgs::msg::OccupancyGrid>(
      node_.declare_parameter<std::string>("obstacle_memory_grid_topic",
                                           "/drone_city_nav/obstacle_memory_grid"),
      rclcpp::QoS{1}.transient_local());
  raw_memory_3d_pointcloud_pub_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      node_.declare_parameter<std::string>(
          "raw_memory_3d_pointcloud_topic",
          "/drone_city_nav/raw_memory_obstacle_points_3d"),
      rclcpp::QoS{1}.reliable().transient_local());
  provenance_pub_ = node_.create_publisher<msg::ObstacleMemoryProvenance>(
      node_.declare_parameter<std::string>(
          "obstacle_memory_provenance_topic",
          "/drone_city_nav/obstacle_memory_provenance"),
      rclcpp::QoS{1}.reliable().transient_local());
  snapshot_pub_ = node_.create_publisher<msg::ObstacleMemorySnapshot>(
      node_.declare_parameter<std::string>("obstacle_memory_snapshot_topic",
                                           "/drone_city_nav/obstacle_memory_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local());
  status_pub_ = node_.create_publisher<msg::ObstacleMemoryStatus>(
      node_.declare_parameter<std::string>("obstacle_memory_status_topic",
                                           "/drone_city_nav/obstacle_memory_status"),
      rclcpp::QoS{1}.reliable().transient_local());
  raw_obstacle_snapshot_pub_ = node_.create_publisher<msg::RawObstacleSnapshot>(
      node_.declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                           "/drone_city_nav/raw_obstacle_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local());

  RCLCPP_INFO(node_.get_logger(),
              "Obstacle memory transport: mode=%s status=every_update "
              "raw=%s full_snapshot=debug_period debug_period=%.2fs "
              "diagnostic_period=%.2fs budgets[serialized_bytes=%zu assembly=%.1fms "
              "publish_interval=%.1fms]",
              use_static_map_ ? "static" : "no_static",
              use_static_map_ ? "debug_period" : "every_update",
              debug_publish_period_s_, diagnostic_period_s_, maximum_serialized_bytes_,
              maximum_assembly_time_ms_, maximum_publish_interval_ms_);
}

void ObstacleMemoryTransport::publish(
    const OccupancyGrid2D& raw_grid,
    const std::unordered_map<std::size_t, MemoryCellProvenance>& provenance,
    const GridCellCounts& cell_counts, const rclcpp::Time& stamp) {
  const std::int64_t stamp_ns = stamp.nanoseconds();
  if (producer_instance_id_ == 0U) {
    producer_instance_id_ =
        static_cast<std::uint64_t>(std::max<std::int64_t>(1, stamp_ns));
  }
  ++sequence_;
  if (!rclcpp::ok(node_.get_node_base_interface()->get_context())) {
    return;
  }

  const bool publish_debug =
      debug_publish_period_s_ <= 0.0 || last_debug_publish_stamp_ns_ <= 0 ||
      stamp_ns - last_debug_publish_stamp_ns_ >=
          static_cast<std::int64_t>(debug_publish_period_s_ * 1.0e9);
  const bool publish_raw = !use_static_map_ || publish_debug;
  const auto assembly_started = std::chrono::steady_clock::now();
  std::optional<nav_msgs::msg::OccupancyGrid> grid_message;
  if (publish_raw || publish_debug) {
    grid_message = makeObstacleMemoryOccupancyGridMessage(raw_grid, stamp, frame_id_);
  }

  bool raw_snapshot_published{false};
  if (publish_raw && grid_message.has_value()) {
    raw_snapshot_published =
        publishRawWorldSnapshot(*grid_message, producer_instance_id_, sequence_);
    raw_snapshot_publications_ += raw_snapshot_published ? 1U : 0U;
  }

  std::optional<msg::ObstacleMemorySnapshot> snapshot_message;
  double assembly_ms{0.0};
  if (publish_debug) {
    snapshot_message = makeObstacleMemorySnapshotMessage(
        *grid_message, provenance, sequence_, producer_instance_id_);
    const auto assembly_duration = std::chrono::steady_clock::now() - assembly_started;
    snapshot_message->producer_assembly_duration_ns =
        static_cast<std::uint64_t>(std::max<std::int64_t>(
            0, std::chrono::duration_cast<std::chrono::nanoseconds>(assembly_duration)
                   .count()));
    assembly_ms =
        static_cast<double>(snapshot_message->producer_assembly_duration_ns) / 1.0e6;
    const double publish_interval_ms =
        last_snapshot_publish_stamp_ns_ > 0 &&
                stamp_ns > last_snapshot_publish_stamp_ns_
            ? static_cast<double>(stamp_ns - last_snapshot_publish_stamp_ns_) / 1.0e6
            : 0.0;
    last_snapshot_publish_stamp_ns_ = stamp_ns;
    ++snapshot_publications_;
    maximum_assembly_since_report_ms_ =
        std::max(maximum_assembly_since_report_ms_, assembly_ms);
    maximum_publish_interval_since_report_ms_ =
        std::max(maximum_publish_interval_since_report_ms_, publish_interval_ms);
    snapshot_pub_->publish(*snapshot_message);
    raw_grid_pub_->publish(snapshot_message->grid);
    provenance_pub_->publish(snapshot_message->provenance);
    raw_memory_3d_pointcloud_pub_->publish(buildObstacleMemoryTriggerPointCloud(
        provenance, snapshot_message->grid.header.stamp, frame_id_));
    last_debug_publish_stamp_ns_ = stamp_ns;
    ++debug_publications_;
  }

  msg::ObstacleMemoryStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = frame_id_;
  status.producer_instance_id = producer_instance_id_;
  status.sequence = sequence_;
  status.occupied_cell_count = static_cast<std::uint64_t>(cell_counts.occupied_cells);
  status.raw_snapshot_published = raw_snapshot_published;
  status.full_snapshot_published = snapshot_message.has_value();
  status_pub_->publish(status);
  ++status_publications_;

  const double status_interval_ms =
      last_status_publish_stamp_ns_ > 0 && stamp_ns > last_status_publish_stamp_ns_
          ? static_cast<double>(stamp_ns - last_status_publish_stamp_ns_) / 1.0e6
          : 0.0;
  last_status_publish_stamp_ns_ = stamp_ns;
  RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                       "Obstacle memory update published: producer_instance=%" PRIu64
                       " sequence=%" PRIu64 " stamp_ns=%" PRId64
                       " status_interval_ms=%.3f full_assembly_ms=%.3f occupied=%zu "
                       "raw_published=%s full_published=%s",
                       status.producer_instance_id, status.sequence, stamp_ns,
                       status_interval_ms, assembly_ms, cell_counts.occupied_cells,
                       raw_snapshot_published ? "true" : "false",
                       snapshot_message.has_value() ? "true" : "false");

  const bool report_transport =
      snapshot_message.has_value() &&
      (last_diagnostic_stamp_ns_ <= 0 ||
       stamp_ns - last_diagnostic_stamp_ns_ >=
           static_cast<std::int64_t>(diagnostic_period_s_ * 1.0e9));
  if (report_transport) {
    const std::size_t snapshot_bytes =
        serializedObstacleMemorySnapshotSize(*snapshot_message);
    const std::size_t provenance_bytes =
        serializedObstacleMemoryProvenanceSize(snapshot_message->provenance);
    const bool within_budget =
        snapshot_bytes <= maximum_serialized_bytes_ &&
        maximum_assembly_since_report_ms_ <= maximum_assembly_time_ms_ &&
        maximum_publish_interval_since_report_ms_ <= maximum_publish_interval_ms_;
    const double report_elapsed_s =
        last_diagnostic_stamp_ns_ > 0 && stamp_ns > last_diagnostic_stamp_ns_
            ? static_cast<double>(stamp_ns - last_diagnostic_stamp_ns_) / 1.0e9
            : 0.0;
    const std::uint64_t report_publications =
        snapshot_publications_ - publications_at_last_diagnostic_;
    const double publish_rate_hz =
        report_elapsed_s > 0.0
            ? static_cast<double>(report_publications) / report_elapsed_s
            : 0.0;
    const char* status_name = within_budget ? "within_budget" : "exceeded";
    if (within_budget) {
      RCLCPP_INFO(
          node_.get_logger(),
          "Obstacle memory snapshot budget: status=%s sequence=%" PRIu64
          " full_serialized_bytes=%zu provenance_serialized_bytes=%zu "
          "grid_cells=%zu max_assembly_ms=%.3f max_publish_interval_ms=%.3f "
          "publish_rate_hz=%.3f publications=%" PRIu64 " status_publications=%" PRIu64
          " raw_publications=%" PRIu64 " debug_publications=%" PRIu64,
          status_name, snapshot_message->sequence, snapshot_bytes, provenance_bytes,
          snapshot_message->grid.data.size(), maximum_assembly_since_report_ms_,
          maximum_publish_interval_since_report_ms_, publish_rate_hz,
          snapshot_publications_, status_publications_, raw_snapshot_publications_,
          debug_publications_);
    } else {
      RCLCPP_WARN(
          node_.get_logger(),
          "Obstacle memory snapshot budget: status=%s sequence=%" PRIu64
          " full_serialized_bytes=%zu max_serialized_bytes=%zu "
          "observed_max_assembly_ms=%.3f assembly_budget_ms=%.3f "
          "observed_max_publish_interval_ms=%.3f publish_interval_budget_ms=%.3f "
          "publish_rate_hz=%.3f",
          status_name, snapshot_message->sequence, snapshot_bytes,
          maximum_serialized_bytes_, maximum_assembly_since_report_ms_,
          maximum_assembly_time_ms_, maximum_publish_interval_since_report_ms_,
          maximum_publish_interval_ms_, publish_rate_hz);
    }
    last_diagnostic_stamp_ns_ = stamp_ns;
    publications_at_last_diagnostic_ = snapshot_publications_;
    maximum_assembly_since_report_ms_ = 0.0;
    maximum_publish_interval_since_report_ms_ = 0.0;
  }

  if (snapshot_message.has_value()) {
    const std::size_t invalid_z_count = static_cast<std::size_t>(
        std::count_if(provenance.begin(), provenance.end(), [](const auto& item) {
          return !item.second.min_endpoint_z_m.has_value() ||
                 !item.second.max_endpoint_z_m.has_value();
        }));
    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "Obstacle memory provenance snapshot: occupied=%zu records=%zu invalid_z=%zu",
        cell_counts.occupied_cells, snapshot_message->provenance.cells.size(),
        invalid_z_count);
  }
}

bool ObstacleMemoryTransport::publishRawWorldSnapshot(
    const nav_msgs::msg::OccupancyGrid& memory_grid,
    const std::uint64_t producer_instance_id, const std::uint64_t snapshot_revision) {
  const std::optional<msg::RawObstacleSnapshot> message = composeRawObstacleSnapshot(
      memory_grid, producer_instance_id, snapshot_revision, static_grid_,
      risk_critical_distance_m_, risk_preferred_distance_m_);
  if (!message.has_value()) {
    RCLCPP_ERROR(node_.get_logger(),
                 "RAW_WORLD_SNAPSHOT rejected reason=invalid_grid_composition");
    return false;
  }
  raw_obstacle_snapshot_pub_->publish(*message);
  return true;
}

} // namespace drone_city_nav

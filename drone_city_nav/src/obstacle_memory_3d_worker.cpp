#include "obstacle_memory_3d_worker.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {

ObstacleMemory3DWorker::ObstacleMemory3DWorker(
    rclcpp::Node& node, const GridBounds3D& bounds,
    const ObstacleMemory3DConfig& memory_config,
    const double minimum_mapping_altitude_m, std::string frame_id,
    const bool gazebo_aligned_rviz_axes_swapped, const std::size_t scan_queue_capacity)
    : node_{node},
      memory_{bounds, memory_config},
      mapping_lifecycle_{minimum_mapping_altitude_m},
      transport_{node, frame_id, gazebo_aligned_rviz_axes_swapped},
      moving_body_pub_{node.create_publisher<msg::MovingBodyEvidence3D>(
          node.declare_parameter<std::string>(
              "moving_body_evidence_topic", "/drone_city_nav/moving_body_evidence_3d"),
          rclcpp::SensorDataQoS{})},
      frame_id_{std::move(frame_id)},
      mailbox_{scan_queue_capacity},
      worker_{[this](const std::stop_token token) { workerLoop(token); }} {
}

ObstacleMemory3DWorker::~ObstacleMemory3DWorker() {
  if (worker_.joinable()) {
    worker_.request_stop();
    mailbox_.notifyAll();
    worker_.join();
  }
}

void ObstacleMemory3DWorker::updateArmed(const bool armed) noexcept {
  armed_.store(armed, std::memory_order_release);
  armed_seen_.store(true, std::memory_order_release);
}

bool ObstacleMemory3DWorker::enqueue(PersistentLidarScan3D scan) {
  const bool dropped = mailbox_.push(std::move(scan));
  if (dropped) {
    dropped_scans_.fetch_add(1U, std::memory_order_relaxed);
  }
  return dropped;
}

void ObstacleMemory3DWorker::logNearestBodies(const Point3& origin) {
  // The bodies the memory follows, the five nearest the vehicle, once a
  // second: what the planner's law and the sweep stand on.
  const std::vector<MovingBody3D>& bodies = memory_.movingBodies();
  if (bodies.empty()) {
    return;
  }
  std::vector<const MovingBody3D*> nearest;
  nearest.reserve(bodies.size());
  for (const MovingBody3D& body : bodies) {
    nearest.push_back(&body);
  }
  std::ranges::sort(nearest,
                    [&](const MovingBody3D* first, const MovingBody3D* second) {
                      return distance3D(first->centre_map, origin) <
                             distance3D(second->centre_map, origin);
                    });
  std::string text;
  std::size_t listed = 0U;
  for (const MovingBody3D* body : nearest) {
    if (listed++ >= 5U) {
      break;
    }
    std::array<char, 160> item{};
    std::snprintf(item.data(), item.size(),
                  " [%.1f %.1f %.1f r%.2f h%.2f v%.2f,%.2f n%zu a%.1f d%.1f]",
                  body->centre_map.x, body->centre_map.y, body->centre_map.z,
                  body->radius_m, body->top_m - body->bottom_m,
                  body->velocity_map_mps.x, body->velocity_map_mps.y, body->voxels,
                  body->age_s, distance3D(body->centre_map, origin));
    text += item.data();
  }
  RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                       "FOLLOWED_BODIES followed=%zu nearest=%s", bodies.size(),
                       text.c_str());
}

void ObstacleMemory3DWorker::publishMovingBodies(
    const std::int64_t acquisition_stamp_ns) {
  msg::MovingBodyEvidence3D message;
  message.header.stamp = rclcpp::Time{acquisition_stamp_ns, RCL_ROS_TIME};
  message.header.frame_id = frame_id_;
  message.sequence = ++moving_body_sequence_;
  for (const MovingBody3D& body : memory_.movingBodies()) {
    msg::MovingBody3D output;
    output.centre_map.x = body.centre_map.x;
    output.centre_map.y = body.centre_map.y;
    output.centre_map.z = body.centre_map.z;
    output.velocity_map_mps.x = body.velocity_map_mps.x;
    output.velocity_map_mps.y = body.velocity_map_mps.y;
    output.velocity_map_mps.z = body.velocity_map_mps.z;
    output.radius_m = body.radius_m;
    output.bottom_m = body.bottom_m;
    output.top_m = body.top_m;
    output.age_s = body.age_s;
    output.voxels = static_cast<std::uint32_t>(
        std::min<std::size_t>(body.voxels, std::numeric_limits<std::uint32_t>::max()));
    message.bodies.push_back(output);
  }
  moving_body_pub_->publish(message);
}

void ObstacleMemory3DWorker::workerLoop(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::optional<PersistentLidarScan3D> scan = mailbox_.waitPop(stop_token);
    if (!scan.has_value()) {
      continue;
    }
    process(std::move(*scan));
  }
}

void ObstacleMemory3DWorker::process(PersistentLidarScan3D scan) {
  const auto started = std::chrono::steady_clock::now();
  if (armed_seen_.load(std::memory_order_acquire)) {
    mapping_lifecycle_.updateArmed(armed_.load(std::memory_order_acquire));
  }
  if (!mapping_lifecycle_.updateAltitude(scan.altitude_m, scan.altitude_valid)) {
    return;
  }

  const std::size_t forgotten_cooperative_voxels = memory_.forgetDynamicVolumes(
      scan.dynamic_filter_plan.cooperative_memory_exclusions);
  if (!scan.dynamic_filter_plan.cooperative_memory_exclusions.empty()) {
    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "COOPERATIVE_PEER_LIDAR_FILTER3D filtered_beams=%zu known_peers=%zu "
        "forgotten_voxels=%zu",
        scan.cooperative_filtered,
        scan.dynamic_filter_plan.cooperative_memory_exclusions.size(),
        forgotten_cooperative_voxels);
  }
  const ObstacleMemory3DStats stats = memory_.integrateScan(
      LidarScan3DView{.origin_map = scan.origin_map,
                      .beams = scan.beams,
                      .acquisition_stamp_ns = scan.acquisition_stamp_ns});
  const ObstacleMemory3DChanges changes = memory_.takeChanges();
  publishMovingBodies(scan.acquisition_stamp_ns);
  logNearestBodies(scan.origin_map);
  const auto integration_finished = std::chrono::steady_clock::now();
  const bool transport_coalesced = transport_.enqueue(
      memory_.grid(), changes, rclcpp::Time{scan.acquisition_stamp_ns, RCL_ROS_TIME},
      scan.publish_debug);
  const auto finished = std::chrono::steady_clock::now();

  const double queue_age_ms =
      scan.enqueued_at == std::chrono::steady_clock::time_point{}
          ? -1.0
          : std::chrono::duration<double, std::milli>(started - scan.enqueued_at)
                .count();
  const double integration_ms =
      std::chrono::duration<double, std::milli>(integration_finished - started).count();
  const double transport_enqueue_ms =
      std::chrono::duration<double, std::milli>(finished - integration_finished)
          .count();
  RCLCPP_INFO_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 1000,
      "LIDAR3D_MEMORY accepted=true stamp_ns=%" PRId64
      " source=%zu processed=%zu hits=%zu misses=%zu surface=%zu invalid=%zu "
      "self_filtered=%zu persistent_self_filtered=%zu unobservable_on_path=%zu "
      "aerosol=%zu thin=%zu dropout=%zu dynamic_filtered=%zu "
      "dynamic_forgotten=%zu transitions=%zu decayed=%zu decayed_total=%zu "
      "moving_hits=%zu moving_cleared=%zu moving_expired=%zu moving_promoted=%zu "
      "moving_joined=%zu "
      "moving_voxels=%zu followed_bodies=%zu swept_voxels=%zu revision=%" PRIu64
      " queue_age_ms=%.3f integration_ms=%.3f transport_enqueue_ms=%.3f "
      "evidence_interval_ms=%.3f stale_acquisition=%s scan_dropped_total=%" PRIu64
      " transport_coalesced=%s debug=%s",
      scan.acquisition_stamp_ns, scan.source_beams, stats.processed_beams,
      stats.hit_beams, stats.miss_beams, stats.surface_beams,
      stats.invalid_beams + scan.projection_invalid, scan.self_filtered,
      scan.persistent_self_filtered, scan.unobservable_on_path, scan.aerosol_beams,
      scan.thin_aerosol_beams, scan.dropout_beams, scan.cooperative_filtered,
      forgotten_cooperative_voxels, stats.state_transitions, stats.decayed_voxels,
      memory_.decayedVoxelTotal(), stats.moving_hits, stats.moving_cleared,
      stats.moving_expired, stats.moving_promoted, stats.moving_joined,
      stats.moving_voxels, stats.followed_bodies, stats.swept_voxels,
      memory_.revision(), queue_age_ms, integration_ms, transport_enqueue_ms,
      1000.0 * stats.evidence_interval_s, stats.stale_acquisition ? "true" : "false",
      dropped_scans_.load(std::memory_order_relaxed),
      transport_coalesced ? "true" : "false", scan.publish_debug ? "true" : "false");
}

} // namespace drone_city_nav

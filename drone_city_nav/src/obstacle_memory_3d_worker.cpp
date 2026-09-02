#include "obstacle_memory_3d_worker.hpp"

#include <chrono>
#include <cinttypes>
#include <optional>
#include <utility>

namespace drone_city_nav {

ObstacleMemory3DWorker::ObstacleMemory3DWorker(
    rclcpp::Node& node, const GridBounds3D& bounds,
    const ObstacleMemory3DConfig& memory_config,
    const double minimum_mapping_altitude_m, std::string frame_id,
    const std::size_t scan_queue_capacity)
    : node_{node},
      memory_{bounds, memory_config},
      mapping_lifecycle_{minimum_mapping_altitude_m},
      transport_{node, std::move(frame_id)},
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

  const std::size_t forgotten_tracked_voxels =
      memory_.forgetDynamicVolumes(scan.dynamic_filter_plan.tracked_agent_exclusions);
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
      "self_filtered=%zu persistent_self_filtered=%zu dynamic_filtered=%zu "
      "dynamic_forgotten=%zu transitions=%zu revision=%" PRIu64
      " queue_age_ms=%.3f integration_ms=%.3f transport_enqueue_ms=%.3f "
      "evidence_interval_ms=%.3f stale_acquisition=%s scan_dropped_total=%" PRIu64
      " transport_coalesced=%s debug=%s",
      scan.acquisition_stamp_ns, scan.source_beams, stats.processed_beams,
      stats.hit_beams, stats.miss_beams, stats.surface_beams,
      stats.invalid_beams + scan.projection_invalid, scan.self_filtered,
      scan.persistent_self_filtered,
      scan.tracked_agent_filtered + scan.cooperative_filtered,
      forgotten_tracked_voxels + forgotten_cooperative_voxels, stats.state_transitions,
      memory_.revision(), queue_age_ms, integration_ms, transport_enqueue_ms,
      1000.0 * stats.evidence_interval_s, stats.stale_acquisition ? "true" : "false",
      dropped_scans_.load(std::memory_order_relaxed),
      transport_coalesced ? "true" : "false", scan.publish_debug ? "true" : "false");
}

} // namespace drone_city_nav

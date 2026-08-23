#pragma once

#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/mapping_lifecycle.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "obstacle_memory_transport_3d.hpp"

namespace drone_city_nav {

struct PersistentLidarScan3D {
  Point3 origin_map{};
  std::vector<LidarBeam3D> beams;
  DynamicAgentLidarFilterPlan dynamic_filter_plan;
  std::int64_t acquisition_stamp_ns{0};
  std::chrono::steady_clock::time_point enqueued_at{};
  double altitude_m{0.0};
  std::size_t source_beams{0U};
  std::size_t projection_invalid{0U};
  std::size_t self_filtered{0U};
  std::size_t persistent_self_filtered{0U};
  std::size_t tracked_agent_filtered{0U};
  std::size_t cooperative_filtered{0U};
  bool altitude_valid{false};
  bool publish_debug{false};
};

class ObstacleMemory3DWorker final {
public:
  ObstacleMemory3DWorker(rclcpp::Node& node, const GridBounds3D& bounds,
                         const ObstacleMemory3DConfig& memory_config,
                         double minimum_mapping_altitude_m, std::string frame_id);
  ~ObstacleMemory3DWorker();

  ObstacleMemory3DWorker(const ObstacleMemory3DWorker&) = delete;
  ObstacleMemory3DWorker& operator=(const ObstacleMemory3DWorker&) = delete;
  ObstacleMemory3DWorker(ObstacleMemory3DWorker&&) = delete;
  ObstacleMemory3DWorker& operator=(ObstacleMemory3DWorker&&) = delete;

  void updateArmed(bool armed) noexcept;
  [[nodiscard]] bool enqueue(PersistentLidarScan3D scan);

private:
  void workerLoop(std::stop_token stop_token);
  void process(PersistentLidarScan3D scan);

  rclcpp::Node& node_;
  ObstacleMemory3D memory_;
  MappingLifecycle mapping_lifecycle_;
  ObstacleMemoryTransport3D transport_;
  LatestValueMailbox<PersistentLidarScan3D> mailbox_;
  std::atomic<bool> armed_seen_{false};
  std::atomic<bool> armed_{false};
  std::atomic<std::uint64_t> coalesced_scans_{0U};
  std::jthread worker_;
};

} // namespace drone_city_nav

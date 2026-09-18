#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct DynamicAgentLidarStateConfig {
  bool cooperative_enabled{false};
  std::string own_vehicle_id;
  double cooperative_peer_horizontal_margin_m{0.0};
  double cooperative_peer_vertical_margin_m{0.0};
  double cooperative_alignment_extrapolation_s{0.5};
  CooperativePeerStoreConfig peer_store{};
};

struct DynamicAgentLidarFilterPlan {
  std::vector<DynamicAgentLidarVolume> cooperative_memory_exclusions;
};

class DynamicAgentLidarState final {
public:
  explicit DynamicAgentLidarState(DynamicAgentLidarStateConfig config);

  [[nodiscard]] CooperativePeerUpdateStatus
  updateCooperativeIntent(const CooperativeFlightIntentData& intent,
                          std::int64_t now_ns);

  [[nodiscard]] DynamicAgentLidarFilterPlan
  makeFilterPlan(std::int64_t now_ns, std::int64_t acquisition_stamp_ns);

private:
  DynamicAgentLidarStateConfig config_{};
  std::mutex mutex_;
  std::unique_ptr<CooperativePeerStore> peer_store_;
};

} // namespace drone_city_nav

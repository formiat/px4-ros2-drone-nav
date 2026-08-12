#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct DynamicAgentLidarStateConfig {
  bool cooperative_enabled{false};
  std::string own_vehicle_id;
  double tracked_agent_radius_m{1.0};
  double tracked_agent_vertical_tolerance_m{1.0};
  double tracked_agent_maximum_age_s{0.5};
  bool tracked_agent_excluded_from_latest_safety{true};
  double cooperative_peer_horizontal_margin_m{0.0};
  double cooperative_peer_vertical_margin_m{0.0};
  double cooperative_alignment_extrapolation_s{0.5};
  CooperativePeerStoreConfig peer_store{};
};

struct DynamicAgentLidarFilterPlan {
  std::vector<DynamicAgentLidarVolume> tracked_agent_exclusions;
  std::vector<DynamicAgentLidarVolume> cooperative_memory_exclusions;
  bool tracked_agent_excluded_from_latest_safety{false};
};

class DynamicAgentLidarState final {
public:
  explicit DynamicAgentLidarState(DynamicAgentLidarStateConfig config);

  void updateTrackedAgent(const Point3& position, const Vec3& velocity,
                          bool position_valid, bool velocity_valid,
                          std::int64_t stamp_ns) noexcept;

  [[nodiscard]] CooperativePeerUpdateStatus
  updateCooperativeIntent(const CooperativeFlightIntentData& intent,
                          std::int64_t now_ns);

  [[nodiscard]] DynamicAgentLidarFilterPlan
  makeFilterPlan(std::int64_t now_ns, std::int64_t acquisition_stamp_ns);

private:
  struct TrackedAgentState {
    Point3 position{};
    Vec3 velocity{};
    std::int64_t stamp_ns{0};
    bool position_valid{false};
    bool velocity_valid{false};
  };

  DynamicAgentLidarStateConfig config_{};
  TrackedAgentState tracked_agent_{};
  std::unique_ptr<CooperativePeerStore> peer_store_;
};

} // namespace drone_city_nav

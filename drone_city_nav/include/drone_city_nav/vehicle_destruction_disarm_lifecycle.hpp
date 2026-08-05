#pragma once

#include <cstdint>

namespace drone_city_nav {

struct VehicleDestructionDisarmConfig {
  double retry_period_s{0.2};
};

struct VehicleDestructionDisarmUpdate {
  bool latched{false};
  bool confirmed{false};
  bool force_disarm_requested{false};
};

class VehicleDestructionDisarmLifecycle final {
public:
  explicit VehicleDestructionDisarmLifecycle(
      const VehicleDestructionDisarmConfig& config = {});

  void latch(std::int64_t stamp_ns) noexcept;

  [[nodiscard]] VehicleDestructionDisarmUpdate
  update(std::int64_t stamp_ns, bool vehicle_status_seen, bool armed) noexcept;

  [[nodiscard]] bool latched() const noexcept;

private:
  VehicleDestructionDisarmConfig config_{};
  std::int64_t latched_stamp_ns_{0};
  std::int64_t last_request_stamp_ns_{0};
  bool latched_{false};
  bool command_sent_{false};
  bool confirmed_{false};
};

} // namespace drone_city_nav

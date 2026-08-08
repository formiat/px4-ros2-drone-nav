#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>

namespace drone_city_nav {

enum class DirectTrackingReseedReason : std::uint8_t {
  kNone,
  kBearingChange,
  kNoClosing,
};

struct DirectTrackingManeuverConfig {
  double bearing_change_threshold_rad{0.5235987755982988};
  double minimum_closing_speed_mps{0.5};
  double closing_recovery_speed_mps{1.5};
  double no_closing_duration_s{1.0};
  double minimum_reseed_interval_s{0.5};
};

struct DirectTrackingManeuverObservation {
  Point3 interceptor_position{};
  Vec3 interceptor_velocity{};
  Point3 target_position{};
  Vec3 target_velocity{};
  std::int64_t stamp_ns{0};
  std::uint64_t line_of_sight_generation{0U};
  bool active{false};
};

struct DirectTrackingManeuverUpdate {
  DirectTrackingReseedReason reason{DirectTrackingReseedReason::kNone};
  std::uint64_t reseed_generation{0U};
  double bearing_change_rad{0.0};
  double closing_speed_mps{0.0};
  double no_closing_duration_s{0.0};
  bool reseed_requested{false};
};

class DirectTrackingManeuverLifecycle final {
public:
  explicit DirectTrackingManeuverLifecycle(
      const DirectTrackingManeuverConfig& config = {});

  [[nodiscard]] DirectTrackingManeuverUpdate
  update(const DirectTrackingManeuverObservation& observation) noexcept;
  void resetEpisode() noexcept;

private:
  DirectTrackingManeuverConfig config_{};
  std::uint64_t line_of_sight_generation_{0U};
  std::uint64_t reseed_generation_{0U};
  std::int64_t no_closing_since_ns_{0};
  std::int64_t last_reseed_stamp_ns_{0};
  double reference_bearing_rad_{0.0};
  bool reference_bearing_valid_{false};
  bool no_closing_reseeded_{false};
};

[[nodiscard]] const char*
directTrackingReseedReasonName(DirectTrackingReseedReason reason) noexcept;

} // namespace drone_city_nav

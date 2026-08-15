#pragma once

#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

struct CooperativePassageTimingConfig {
  double minimum_prediction_speed_mps{1.0};
  double maximum_prediction_horizon_s{30.0};
};

[[nodiscard]] CooperativePassageUse
makeCooperativePassageUse(const ConstrainedRouteObservation& observation,
                          const CooperativePassageAssignment& assignment,
                          std::int64_t now_ns, double planned_speed_mps,
                          const CooperativePassageTimingConfig& config) noexcept;

enum class CooperativePassageYieldStatus : std::uint8_t {
  kDisabled,
  kNotRequired,
  kStaleCommand,
  kVehicleMismatch,
  kRouteMismatch,
  kPassageMismatch,
  kCorridorMismatch,
  kEntryTimeSatisfied,
  kNotApproaching,
  kAccepted,
};

struct CooperativePassageYieldConfig {
  double stopping_buffer_m{2.0};
  double reaction_latency_s{0.1};
  double maximum_braking_acceleration_mps2{8.0};
};

struct CooperativePassageYieldDecision {
  CooperativePassageYieldStatus status{CooperativePassageYieldStatus::kDisabled};
  bool active{false};
  bool hold_at_entry{false};
  double hold_station_m{0.0};
  double maximum_speed_mps{0.0};
  std::int64_t entry_not_before_ns{0};
};

[[nodiscard]] CooperativePassageYieldDecision evaluateCooperativePassageYield(
    const CooperativeManeuverCommandData& command, const CooperativePassageUse& passage,
    const ConstrainedRouteObservation& observation, std::string_view vehicle_id,
    std::int64_t now_ns, double current_speed_mps,
    const CooperativePassageYieldConfig& config) noexcept;

[[nodiscard]] const char*
cooperativePassageYieldStatusName(CooperativePassageYieldStatus status) noexcept;

} // namespace drone_city_nav

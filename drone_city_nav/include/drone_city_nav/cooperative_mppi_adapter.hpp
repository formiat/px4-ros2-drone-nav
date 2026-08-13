#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class CooperativeMppiAdapterStatus : std::uint8_t {
  kDisabled,
  kAccepted,
  kVehicleMismatch,
  kInvalid,
  kStale,
  kNoTrajectoryCoverage,
};

struct CooperativeMppiAdapterResult {
  CooperativeMppiAdapterStatus status{CooperativeMppiAdapterStatus::kDisabled};
  std::vector<mppi::CooperativePeerTrajectory> conflicting_peers;
  std::optional<mppi::CooperativeManeuverPreference> maneuver;
  std::optional<mppi::CooperativeSeparationAcquisition> acquisition;
  bool avoidance_active{false};
  bool space_time_plan_active{false};
  double space_time_lateral_offset_m{0.0};
  double space_time_vertical_offset_m{0.0};
  double space_time_shift_s{0.0};
  double space_time_predicted_minimum_separation_m{0.0};
  double space_time_integrated_shortfall_m2_s{0.0};
  std::size_t space_time_evaluated_candidate_count{0U};

  [[nodiscard]] bool accepted() const noexcept {
    return status == CooperativeMppiAdapterStatus::kAccepted;
  }
};

[[nodiscard]] CooperativeMppiAdapterResult
adaptCooperativeMppiCommand(const CooperativeManeuverCommandData& command,
                            std::string_view vehicle_id, std::int64_t planning_stamp_ns,
                            std::size_t steps, float dt_s);

[[nodiscard]] const char*
cooperativeMppiAdapterStatusName(CooperativeMppiAdapterStatus status) noexcept;

} // namespace drone_city_nav

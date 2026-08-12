#include "drone_city_nav/cooperative_mppi_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Vec3& value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] mppi::CooperativeManeuver
mppiManeuver(const CooperativeManeuver maneuver) noexcept {
  return static_cast<mppi::CooperativeManeuver>(maneuver);
}

} // namespace

CooperativeMppiAdapterResult adaptCooperativeMppiCommand(
    const CooperativeManeuverCommandData& command, const std::string_view vehicle_id,
    const std::int64_t planning_stamp_ns, const std::size_t steps, const float dt_s) {
  CooperativeMppiAdapterResult result;
  if (command.command_generation == 0U) {
    return result;
  }
  if (vehicle_id.empty() || command.vehicle_id != vehicle_id) {
    result.status = CooperativeMppiAdapterStatus::kVehicleMismatch;
    return result;
  }
  if (planning_stamp_ns <= 0 || steps == 0U || !std::isfinite(dt_s) || !(dt_s > 0.0F) ||
      command.stamp_ns <= 0 || command.valid_until_ns < command.stamp_ns ||
      !finite(command.preferred_acceleration_direction)) {
    result.status = CooperativeMppiAdapterStatus::kInvalid;
    return result;
  }
  if (planning_stamp_ns > command.valid_until_ns) {
    result.status = CooperativeMppiAdapterStatus::kStale;
    return result;
  }
  if (!command.avoidance_active) {
    result.status = CooperativeMppiAdapterStatus::kAccepted;
    return result;
  }

  result.conflicting_peers.reserve(command.conflicting_peers.size());
  for (const CooperativePeerTrajectoryData& peer : command.conflicting_peers) {
    if (peer.vehicle_id.empty() || !(peer.footprint_radius_m >= 0.0) ||
        peer.footprint_radius_m > std::numeric_limits<float>::max() ||
        peer.valid_from_ns <= 0 || peer.valid_until_ns < peer.valid_from_ns ||
        peer.trajectory.empty()) {
      continue;
    }
    auto samples = std::make_shared<std::vector<mppi::CooperativePeerSample>>(steps);
    std::size_t active_steps = 0U;
    for (std::size_t step = 0U; step < steps; ++step) {
      const std::int64_t offset_ns = static_cast<std::int64_t>(std::llround(
          static_cast<double>(step + 1U) * static_cast<double>(dt_s) * 1.0e9));
      const std::int64_t sample_time_ns = planning_stamp_ns + offset_ns;
      if (sample_time_ns > peer.valid_until_ns) {
        break;
      }
      const std::optional<CooperativeTrajectorySample> sample =
          sampleCooperativeTrajectory(peer, sample_time_ns);
      if (!sample.has_value()) {
        break;
      }
      (*samples)[step] = mppi::CooperativePeerSample{
          .x = static_cast<float>(sample->position.x),
          .y = static_cast<float>(sample->position.y),
          .z = static_cast<float>(sample->position.z),
      };
      ++active_steps;
    }
    if (active_steps == 0U) {
      continue;
    }
    std::fill(samples->begin() + static_cast<std::ptrdiff_t>(active_steps),
              samples->end(), (*samples)[active_steps - 1U]);
    result.conflicting_peers.push_back(mppi::CooperativePeerTrajectory{
        .samples = std::move(samples),
        .footprint_radius_m = static_cast<float>(peer.footprint_radius_m),
        .active_steps = active_steps,
    });
  }
  if (result.conflicting_peers.empty()) {
    result.status = CooperativeMppiAdapterStatus::kNoTrajectoryCoverage;
    return result;
  }
  result.maneuver = mppi::CooperativeManeuverPreference{
      .maneuver = mppiManeuver(command.preferred_maneuver),
      .direction_x = static_cast<float>(command.preferred_acceleration_direction.x),
      .direction_y = static_cast<float>(command.preferred_acceleration_direction.y),
      .direction_z = static_cast<float>(command.preferred_acceleration_direction.z),
      .generation = command.conflict_generation,
  };
  result.status = CooperativeMppiAdapterStatus::kAccepted;
  return result;
}

const char*
cooperativeMppiAdapterStatusName(const CooperativeMppiAdapterStatus status) noexcept {
  switch (status) {
    case CooperativeMppiAdapterStatus::kDisabled:
      return "disabled";
    case CooperativeMppiAdapterStatus::kAccepted:
      return "accepted";
    case CooperativeMppiAdapterStatus::kVehicleMismatch:
      return "vehicle_mismatch";
    case CooperativeMppiAdapterStatus::kInvalid:
      return "invalid";
    case CooperativeMppiAdapterStatus::kStale:
      return "stale";
    case CooperativeMppiAdapterStatus::kNoTrajectoryCoverage:
      return "no_trajectory_coverage";
  }
  return "unknown";
}

} // namespace drone_city_nav

#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

} // namespace

FiniteExecutionPathValidation3D validateRemainingFiniteExecutionAgainstObservedWorld3D(
    const FiniteExecutionState3D& execution,
    const VersionedExecutionInput3D& current_input,
    const VersionedObservedRawWorld3D& current_world,
    const std::int64_t validation_stamp_ns) noexcept {
  if (execution.horizon == nullptr || execution.validation_policy == nullptr ||
      execution.execution_input == nullptr || execution.observed_raw_world == nullptr ||
      execution.static_world != nullptr || !current_input.valid() ||
      !current_input.nominalStateAuthoritative() || !current_world.valid() ||
      validation_stamp_ns <= 0 ||
      current_input.effectiveStampNs() != validation_stamp_ns ||
      current_world.version().producer_instance_id !=
          execution.observed_raw_world->version().producer_instance_id ||
      current_world.version().revision <
          execution.observed_raw_world->version().revision) {
    return {};
  }
  const std::vector<TimedExecutionPathPoint3D> points = timedExecutionPathPoints(
      *execution.horizon, execution.execution_input->previousControl(),
      execution.control_interval_ns);
  if (points.empty()) {
    return {};
  }
  // Contact evidence is the vehicle's pose now: the question is whether the
  // published path continues from where the vehicle stands.
  const std::optional<ProprioceptiveFreeSpaceSeed3D> live_seed =
      proprioceptiveContactSeed3D(Point3{current_input.state().x,
                                         current_input.state().y,
                                         current_input.state().z},
                                  current_input.previousControl(),
                                  execution.validation_policy->sweptFootprint(),
                                  std::addressof(current_world.occupancy()));
  const FiniteExecutionPathWorld3D validation_world{
      .flight_envelope = &execution.validation_policy->flightEnvelope(),
      .dynamics = &execution.validation_policy->dynamics(),
      .altitude_envelope = &execution.validation_policy->altitudeEnvelope(),
      .footprint = &execution.validation_policy->sweptFootprint(),
      .static_occupancy = nullptr,
      .observed_occupancy = &current_world.occupancy(),
      .launch_support_contact = optionalAddress(current_world.launchSupportContact()),
      .proprioceptive_free_space_seed = optionalAddress(live_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = {},
      .terminal_boundary = std::nullopt,
  };
  return validateFiniteExecutionTrajectoryContinuation3D(
      points, execution.valid_from_ns, execution.valid_until_ns, validation_stamp_ns,
      current_input.state(), current_input.previousControl(), validation_world);
}

FiniteExecutionPathValidation3D validateRemainingFiniteExecutionAgainstLatestLidar3D(
    const FiniteExecutionState3D& execution,
    const VersionedExecutionInput3D& current_input,
    const VersionedLatestLidarEvidence3D& current_lidar,
    const std::int64_t validation_stamp_ns) noexcept {
  const bool observed_mode = execution.observed_raw_world != nullptr;
  const bool static_mode = execution.static_world != nullptr;
  if (execution.horizon == nullptr || execution.validation_policy == nullptr ||
      execution.execution_input == nullptr ||
      execution.latest_lidar_evidence == nullptr || observed_mode == static_mode ||
      !current_input.valid() || !current_input.nominalStateAuthoritative() ||
      !current_lidar.valid() || validation_stamp_ns <= 0 ||
      current_input.effectiveStampNs() != validation_stamp_ns ||
      !latestLidarEvidenceFreshAt(current_lidar, *execution.validation_policy,
                                  validation_stamp_ns) ||
      (current_lidar.producerInstanceId() ==
           execution.latest_lidar_evidence->producerInstanceId() &&
       !latestLidarEvidenceNotOlder(current_lidar, *execution.latest_lidar_evidence)) ||
      (observed_mode && !execution.observed_raw_world->valid()) ||
      (static_mode && !execution.static_world->valid())) {
    return {};
  }
  const std::vector<TimedExecutionPathPoint3D> points = timedExecutionPathPoints(
      *execution.horizon, execution.execution_input->previousControl(),
      execution.control_interval_ns);
  if (points.empty()) {
    return {};
  }
  const VersionedObservedRawWorld3D* const observed_world =
      execution.observed_raw_world.get();
  const std::optional<ProprioceptiveFreeSpaceSeed3D> live_seed =
      proprioceptiveContactSeed3D(
          Point3{execution.execution_input->state().x,
                 execution.execution_input->state().y,
                 execution.execution_input->state().z},
          execution.execution_input->previousControl(),
          execution.validation_policy->sweptFootprint(),
          observed_mode ? std::addressof(observed_world->occupancy()) : nullptr);
  const FiniteExecutionPathWorld3D validation_world{
      .flight_envelope = &execution.validation_policy->flightEnvelope(),
      .dynamics = &execution.validation_policy->dynamics(),
      .altitude_envelope = &execution.validation_policy->altitudeEnvelope(),
      .footprint = &execution.validation_policy->sweptFootprint(),
      .static_occupancy = static_mode ? &execution.static_world->occupancy() : nullptr,
      .observed_occupancy = observed_mode ? &observed_world->occupancy() : nullptr,
      .launch_support_contact =
          observed_mode ? optionalAddress(observed_world->launchSupportContact())
                        : nullptr,
      .proprioceptive_free_space_seed = optionalAddress(live_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points =
          std::span<const Point3>{current_lidar.hitPointsMapM()},
      .terminal_boundary = std::nullopt,
  };
  return validateFiniteExecutionTrajectoryContinuation3D(
      points, execution.valid_from_ns, execution.valid_until_ns, validation_stamp_ns,
      current_input.state(), current_input.previousControl(), validation_world);
}

} // namespace drone_city_nav

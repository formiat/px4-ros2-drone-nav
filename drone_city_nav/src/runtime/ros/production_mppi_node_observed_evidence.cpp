#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <optional>
#include <ranges>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupiedCollisionResult3D validateObservedPoint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& footprint,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = std::addressof(occupancy),
      .static_occupancy = nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = launch_support_contact,
      .footprint = footprint,
      .flight_envelope = std::nullopt,
  }};
  return oracle.validatePoint(position, body_axis);
}

} // namespace

std::optional<ProprioceptiveFreeSpaceSeed3D>
ProductionMppiNode::prepareObservedExecutionEvidence3D(
    const ProductionMppiRawWorld3D& raw_world,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority) {
  if (!raw_world.valid()) {
    return std::nullopt;
  }
  const std::shared_ptr<const ObservedOccupancyGrid3D>& occupancy =
      raw_world.occupancyOwner();
  const AppliedControlEvidence3D applied_control = execution_authority != nullptr
                                                       ? execution_authority->control()
                                                       : AppliedControlEvidence3D{};
  const ExecutionOwnerIdentity3D execution_horizon_owner =
      execution_authority != nullptr ? execution_authority->owner()
                                     : ExecutionOwnerIdentity3D{};
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  const std::optional<FootprintBodyAxis> current_body_axis =
      authoritativeBodyAxisForExecution(
          applied_control, execution_horizon_owner, navigation,
          get_clock()->now().nanoseconds(),
          config_.execution.maximum_control_feedback_age_ms,
          config_.execution.maximum_pose_age_ms);
  const std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed =
      current_body_axis.has_value()
          ? std::optional<ProprioceptiveFreeSpaceSeed3D>{ProprioceptiveFreeSpaceSeed3D{
                .position = position,
                .body_axis = *current_body_axis,
                .footprint = config_.world.physical_footprint,
            }}
          : std::nullopt;
  if (!launch_support_seed_ && free_space_seed.has_value()) {
    launch_support_seed_ = free_space_seed;
  }
  if (!launch_support_evaluated_ && launch_support_seed_.has_value()) {
    const bool vehicle_land_contact_received =
        vehicle_land_contact_received_.load(std::memory_order_acquire);
    const bool vehicle_launch_support_confirmed =
        launch_support_confirmed_by_land_detector_.load(std::memory_order_acquire);
    if (vehicle_launch_support_confirmed) {
      launch_support_contact_ =
          makeVehicleLandedSupportContact3D(occupancy->bounds(), *launch_support_seed_);
    } else {
      launch_support_contact_ =
          detectLaunchSupportContact3D(*occupancy, *launch_support_seed_);
      if (!launch_support_contact_ && vehicle_land_contact_received) {
        const OccupiedCollisionResult3D without_support = validateObservedPoint(
            *occupancy, launch_support_seed_->position, launch_support_seed_->body_axis,
            config_.world.physical_footprint, nullptr);
        if (without_support.clear()) {
          launch_support_evaluated_ = true;
          RCLCPP_INFO(get_logger(),
                      "LAUNCH_SUPPORT_CONTACT state=not_present source="
                      "observed_known_free revision=%" PRIu64,
                      raw_world.version().revision);
        }
      }
    }
    if (launch_support_contact_) {
      launch_support_evaluated_ = true;
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=active revision=%" PRIu64
                  " source=%s cells=%zu occupied_evidence=%zu"
                  " anchor=(%.3f,%.3f,%.3f)",
                  raw_world.version().revision,
                  launch_support_contact_->evidence_source ==
                          LaunchSupportEvidenceSource::kVehicleLandDetector
                      ? "vehicle_land_detector"
                      : "observed_occupancy",
                  launch_support_contact_->contact_cells.size(),
                  launch_support_contact_->occupied_evidence_cells,
                  launch_support_seed_->position.x, launch_support_seed_->position.y,
                  launch_support_seed_->position.z);
    } else if (!launch_support_evaluated_ &&
               distance3D(position, launch_support_seed_->position) >
                   std::max(0.5, config_.world.physical_footprint.radius_m)) {
      launch_support_evaluated_ = true;
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=not_detected_after_departure"
                  " revision=%" PRIu64 " position=(%.3f,%.3f,%.3f)",
                  raw_world.version().revision, position.x, position.y, position.z);
    } else if (!launch_support_evaluated_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LAUNCH_SUPPORT_CONTACT state=awaiting_evidence revision=%" PRIu64
          " anchor=(%.3f,%.3f,%.3f)",
          raw_world.version().revision, launch_support_seed_->position.x,
          launch_support_seed_->position.y, launch_support_seed_->position.z);
    }
  }
  if (launch_support_contact_) {
    if (updateLaunchSupportSettling(*launch_support_contact_, position)) {
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=settled revision=%" PRIu64
                  " minimum_axial_departure_m=%.3f position=(%.3f,%.3f,%.3f)",
                  raw_world.version().revision,
                  launch_support_contact_->minimum_axial_departure_m, position.x,
                  position.y, position.z);
    }
    if (current_body_axis.has_value() && free_space_seed.has_value()) {
      const OccupiedCollisionResult3D without_support =
          validateObservedPoint(*occupancy, position, *current_body_axis,
                                config_.world.physical_footprint, nullptr);
      const FootprintBodyAxis support_axis = launch_support_contact_->seed.body_axis;
      const Point3 support_delta{
          position.x - launch_support_contact_->seed.position.x,
          position.y - launch_support_contact_->seed.position.y,
          position.z - launch_support_contact_->seed.position.z,
      };
      const double support_axial_departure_m = support_delta.x * support_axis.x +
                                               support_delta.y * support_axis.y +
                                               support_delta.z * support_axis.z;
      if (without_support.clear() &&
          support_axial_departure_m > occupancy->bounds().resolution_m) {
        RCLCPP_INFO(get_logger(),
                    "LAUNCH_SUPPORT_CONTACT state=released revision=%" PRIu64
                    " axial_departure_m=%.3f position=(%.3f,%.3f,%.3f)",
                    raw_world.version().revision, support_axial_departure_m, position.x,
                    position.y, position.z);
        launch_support_contact_.reset();
      }
    }
  }
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_contact_ ? &*launch_support_contact_ : nullptr;
  const std::optional<OccupiedCollisionResult3D> current_footprint =
      current_body_axis.has_value() && free_space_seed.has_value()
          ? std::optional<OccupiedCollisionResult3D>{validateObservedPoint(
                *occupancy, position, *current_body_axis,
                config_.world.physical_footprint, launch_support_contact)}
          : std::nullopt;
  double support_axial_departure_m{0.0};
  double support_lateral_departure_m{0.0};
  bool failure_is_launch_support_cell{false};
  if (launch_support_contact != nullptr && current_footprint.has_value()) {
    const FootprintBodyAxis axis = launch_support_contact->seed.body_axis;
    const Point3 delta{position.x - launch_support_contact->seed.position.x,
                       position.y - launch_support_contact->seed.position.y,
                       position.z - launch_support_contact->seed.position.z};
    support_axial_departure_m = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
    support_lateral_departure_m = std::sqrt(
        std::max(0.0, delta.x * delta.x + delta.y * delta.y + delta.z * delta.z -
                          support_axial_departure_m * support_axial_departure_m));
    constexpr double kCellContainmentToleranceM{1.0e-6};
    failure_is_launch_support_cell = std::ranges::any_of(
        launch_support_contact->contact_cells, [&](const AxisAlignedBox3D& cell) {
          return current_footprint->failure_point.x >=
                     cell.minimum.x - kCellContainmentToleranceM &&
                 current_footprint->failure_point.x <=
                     cell.maximum.x + kCellContainmentToleranceM &&
                 current_footprint->failure_point.y >=
                     cell.minimum.y - kCellContainmentToleranceM &&
                 current_footprint->failure_point.y <=
                     cell.maximum.y + kCellContainmentToleranceM &&
                 current_footprint->failure_point.z >=
                     cell.minimum.z - kCellContainmentToleranceM &&
                 current_footprint->failure_point.z <=
                     cell.maximum.z + kCellContainmentToleranceM;
        });
  }
  if (current_footprint.has_value()) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OBSERVED_FOOTPRINT_READINESS revision=%" PRIu64
        " status=%s position=(%.3f,%.3f,%.3f) failure_point=(%.3f,%.3f,%.3f)"
        " launch_support_active=%s support_failure_cell=%s"
        " support_axial_departure_m=%.3f support_lateral_departure_m=%.3f"
        " support_maximum_lateral_departure_m=%.3f"
        " support_minimum_axial_departure_m=%.3f"
        " support_maximum_axial_settling_m=%.3f",
        raw_world.version().revision,
        occupiedCollisionStatus3DName(current_footprint->status), position.x,
        position.y, position.z, current_footprint->failure_point.x,
        current_footprint->failure_point.y, current_footprint->failure_point.z,
        launch_support_contact != nullptr ? "true" : "false",
        failure_is_launch_support_cell ? "true" : "false", support_axial_departure_m,
        support_lateral_departure_m,
        launch_support_contact != nullptr
            ? launch_support_contact->maximum_lateral_departure_m
            : 0.0,
        launch_support_contact != nullptr
            ? launch_support_contact->minimum_axial_departure_m
            : 0.0,
        launch_support_contact != nullptr
            ? launch_support_contact->maximum_axial_settling_m
            : 0.0);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "OBSERVED_FOOTPRINT_READINESS revision=%" PRIu64
                         " status=body_axis_unavailable position=(%.3f,%.3f,%.3f)"
                         " proprioceptive_free_space=false",
                         raw_world.version().revision, position.x, position.y,
                         position.z);
  }
  return free_space_seed;
}

} // namespace drone_city_nav

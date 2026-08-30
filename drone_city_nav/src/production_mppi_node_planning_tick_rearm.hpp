#pragma once

#include <cstdint>
#include <memory>

#include "production_mppi_node_planning_tick_context.hpp"

namespace drone_city_nav {

struct ProductionMppiStationaryCaptureRearmContext {
  const ProductionNavigationObjective* objective{nullptr};
  const MissionWaypointSequence* mission_waypoint_sequence{nullptr};
  const ProductionMppiNavigation* navigation{nullptr};
  const ProductionMppiVehicleStatus* vehicle_status{nullptr};
  std::shared_ptr<const CommittedExecutionAuthority3D> execution_authority;
  const OffboardSessionAdmissionState* offboard_session{nullptr};
  const WorldSnapshot3D* world{nullptr};
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  std::shared_ptr<const OccupancyGrid3D> static_occupancy_3d;
  MissionWaypointCaptureGateConfig capture_gate_config{};
  Point3 mission_goal{};
  std::int64_t now_ns{0};
  std::int64_t offboard_session_receive_stamp_ns{0};
  double maximum_pose_age_ms{0.0};
  double maximum_control_feedback_age_ms{0.0};
  double maximum_esdf_age_ms{0.0};
  double observation_age_ms{0.0};
  bool vehicle_status_epoch_stable{false};
  bool terminal_hold_enabled{false};
  bool goal_capture_latched{false};
  bool use_static_map{false};
  bool observed_3d_world{false};
};

[[nodiscard]] bool stationaryCaptureRearmEligibleForPlanningTick(
    const ProductionMppiStationaryCaptureRearmContext& context);

struct ProductionMppiExecutionInputPreparation {
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  ProductionMppiPreviousControlSource previous_control_source{
      ProductionMppiPreviousControlSource::kUnavailable};
  bool control_feedback_fresh{false};
  bool measured_control_available{false};
  bool previous_control_available{false};
};

[[nodiscard]] ProductionMppiExecutionInputPreparation
prepareExecutionInputForPlanningTick(
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& execution_authority,
    std::uint64_t execution_input_sequence, std::int64_t now_ns,
    double maximum_control_feedback_age_ms, bool pose_predicted,
    bool stationary_capture_rearm);

} // namespace drone_city_nav

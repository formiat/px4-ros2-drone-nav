#pragma once

#include "drone_city_nav/intercept_guidance.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/msg/radar_track_mode_command.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/tracking_objective.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct ProductionMppiNavigation {
  mppi::State state{};
  mppi::Control measured_equivalent_control{};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t source_timestamp_us{0U};
  std::uint64_t revision{0U};
  std::uint8_t xy_reset_counter{0U};
  std::uint8_t z_reset_counter{0U};
  std::uint8_t vxy_reset_counter{0U};
  std::uint8_t vz_reset_counter{0U};
  std::uint8_t heading_reset_counter{0U};
  bool source_timestamp_from_sample{false};
  bool position_velocity_authoritative{false};
  bool heading_authoritative{false};
  bool yaw_rate_authoritative{false};
  bool linear_acceleration_authoritative{false};
  bool yaw_acceleration_authoritative{false};
  bool full_state_authoritative{false};
  bool measured_acceleration_valid{false};
  bool valid{false};
};

struct ProductionMppiVehicleStatus {
  std::int64_t receive_stamp_ns{0};
  std::uint64_t source_timestamp_us{0U};
  std::uint64_t revision{0U};
  bool armed{false};
  bool valid{false};
};

struct ProductionTrackingObjective {
  Point3 observed_position{};
  Point3 current_target_position{};
  Point3 unconstrained_predicted_position{};
  Vec3 observed_velocity{};
  std::int64_t observation_stamp_ns{0};
  double prediction_horizon_s{0.0};
  double resolved_fraction{0.0};
  InterceptGuidanceMode guidance_mode{InterceptGuidanceMode::kDirect};
  TrackingObjectiveResolutionStatus resolution_status{
      TrackingObjectiveResolutionStatus::kInvalidInput};
  DirectTrackingTargetStatus direct_target_status{
      DirectTrackingTargetStatus::kWorldUnavailable};
  std::uint8_t radar_cadence_reason{
      msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE};
  bool vertical_prediction_clipped{false};
  bool observed_target_visible{false};
  bool predicted_intercept_path_clear{false};
  bool direct_interception_active{false};
  std::uint64_t line_of_sight_generation{0U};
  std::uint64_t target_track_id{0U};
};

struct ProductionNavigationObjective {
  Point3 goal{};
  std::optional<ProductionTrackingObjective> tracking;
  std::uint64_t mission_epoch{0U};
  std::uint64_t sample_sequence{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
  std::int64_t stamp_ns{0};
  bool continuous_tracking{false};
  bool immediate_hold{false};
};

[[nodiscard]] inline StaticRouteObjective
makeStaticRouteObjective(const ProductionNavigationObjective& objective) noexcept {
  return StaticRouteObjective{
      .goal = objective.goal,
      .mission_epoch = objective.mission_epoch,
      .sample_sequence = objective.sample_sequence,
      .assignment_generation = objective.assignment_generation,
      .target_detection_id = objective.target_detection_id,
      .target_track_id = objective.target_track_id,
      .continuous_tracking = objective.continuous_tracking,
      .available = true,
  };
}

} // namespace drone_city_nav

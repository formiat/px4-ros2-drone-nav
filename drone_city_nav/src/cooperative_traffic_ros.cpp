#include "drone_city_nav/cooperative_traffic_ros.hpp"

#include "drone_city_nav/msg/cooperative_conflict_resource_use.hpp"
#include "drone_city_nav/msg/cooperative_passage_intent.hpp"
#include "drone_city_nav/msg/cooperative_trajectory_point.hpp"

#include <algorithm>
#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kNanosecondsPerSecond{1'000'000'000LL};

[[nodiscard]] Point3 point(const geometry_msgs::msg::Point& value) noexcept {
  return Point3{value.x, value.y, value.z};
}

[[nodiscard]] Vec3 vector(const geometry_msgs::msg::Vector3& value) noexcept {
  return Vec3{value.x, value.y, value.z};
}

void assign(geometry_msgs::msg::Point& target, const Point3& source) noexcept {
  target.x = source.x;
  target.y = source.y;
  target.z = source.z;
}

void assign(geometry_msgs::msg::Vector3& target, const Vec3& source) noexcept {
  target.x = source.x;
  target.y = source.y;
  target.z = source.z;
}

[[nodiscard]] CooperativeManeuver maneuver(const std::uint8_t value) noexcept {
  if (value <= static_cast<std::uint8_t>(CooperativeManeuver::kSlow)) {
    return static_cast<CooperativeManeuver>(value);
  }
  return CooperativeManeuver::kKeep;
}

[[nodiscard]] CooperativePassagePhase passagePhase(const std::uint8_t value) noexcept {
  if (value <= static_cast<std::uint8_t>(CooperativePassagePhase::kDeparture)) {
    return static_cast<CooperativePassagePhase>(value);
  }
  return CooperativePassagePhase::kNone;
}

[[nodiscard]] CooperativePassageUse
passageUseImpl(const msg::CooperativePassageIntent& message) {
  CooperativePassageUse result{
      .passage_traversal_id = PassageTraversalId{message.passage_traversal_id},
      .route_generation = message.route_generation,
      .phase = passagePhase(message.phase),
      .lateral_offset_m = message.lateral_offset_m,
      .minimum_lateral_offset_m = message.minimum_lateral_offset_m,
      .maximum_lateral_offset_m = message.maximum_lateral_offset_m,
      .desired_center_separation_m = message.desired_center_separation_m,
      .direction_sign = message.direction_sign,
      .station_m = message.station_m,
      .distance_to_entry_m = message.distance_to_entry_m,
      .distance_to_exit_m = message.distance_to_exit_m,
      .predicted_entry_ns = cooperativeTimeNanoseconds(message.predicted_entry),
      .predicted_exit_ns = cooperativeTimeNanoseconds(message.predicted_exit),
      .conflict_resources = {},
  };
  result.conflict_resources.reserve(message.conflict_resources.size());
  for (const msg::CooperativeConflictResourceUse& resource :
       message.conflict_resources) {
    result.conflict_resources.push_back(CooperativeConflictResourceUse{
        .conflict_resource_id =
            CooperativeConflictResourceId{resource.conflict_resource_id},
        .begin_station_m = resource.begin_station_m,
        .end_station_m = resource.end_station_m,
        .predicted_entry_ns = cooperativeTimeNanoseconds(resource.predicted_entry),
        .predicted_exit_ns = cooperativeTimeNanoseconds(resource.predicted_exit),
    });
  }
  return result;
}

void assignPassage(msg::CooperativePassageIntent& message,
                   const CooperativePassageUse& passage) {
  message.passage_traversal_id = passage.passage_traversal_id.value();
  message.route_generation = passage.route_generation;
  message.phase = static_cast<std::uint8_t>(passage.phase);
  message.lateral_offset_m = passage.lateral_offset_m;
  message.minimum_lateral_offset_m = passage.minimum_lateral_offset_m;
  message.maximum_lateral_offset_m = passage.maximum_lateral_offset_m;
  message.desired_center_separation_m = passage.desired_center_separation_m;
  message.direction_sign =
      static_cast<std::int8_t>(std::clamp(passage.direction_sign, -1, 1));
  message.station_m = passage.station_m;
  message.distance_to_entry_m = passage.distance_to_entry_m;
  message.distance_to_exit_m = passage.distance_to_exit_m;
  message.predicted_entry = cooperativeTimeMessage(passage.predicted_entry_ns);
  message.predicted_exit = cooperativeTimeMessage(passage.predicted_exit_ns);
  message.conflict_resources.reserve(passage.conflict_resources.size());
  for (const CooperativeConflictResourceUse& resource : passage.conflict_resources) {
    msg::CooperativeConflictResourceUse resource_message;
    resource_message.conflict_resource_id = resource.conflict_resource_id.value();
    resource_message.begin_station_m = resource.begin_station_m;
    resource_message.end_station_m = resource.end_station_m;
    resource_message.predicted_entry =
        cooperativeTimeMessage(resource.predicted_entry_ns);
    resource_message.predicted_exit =
        cooperativeTimeMessage(resource.predicted_exit_ns);
    message.conflict_resources.push_back(std::move(resource_message));
  }
}

[[nodiscard]] msg::CooperativeTrajectoryPoint
trajectoryPointMessage(const CooperativeTrajectorySample& sample,
                       const std::int64_t valid_from_ns) {
  msg::CooperativeTrajectoryPoint message;
  message.time_from_valid_from_s =
      static_cast<float>(static_cast<double>(sample.time_ns - valid_from_ns) * 1.0e-9);
  assign(message.position, sample.position);
  assign(message.velocity, sample.velocity);
  return message;
}

} // namespace

std::int64_t
cooperativeTimeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept {
  return static_cast<std::int64_t>(time.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(time.nanosec);
}

builtin_interfaces::msg::Time
cooperativeTimeMessage(const std::int64_t nanoseconds) noexcept {
  builtin_interfaces::msg::Time result;
  if (nanoseconds <= 0) {
    return result;
  }
  result.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return result;
}

CooperativePassageUse
cooperativePassageUseData(const msg::CooperativePassageIntent& message) {
  return passageUseImpl(message);
}

msg::CooperativePassageIntent
cooperativePassageIntentMessage(const CooperativePassageUse& passage) {
  msg::CooperativePassageIntent message;
  assignPassage(message, passage);
  return message;
}

CooperativeFlightIntentData
cooperativeFlightIntentData(const msg::CooperativeFlightIntent& message) {
  CooperativeFlightIntentData result{
      .vehicle_id = message.vehicle_id,
      .frame_id = message.header.frame_id,
      .stamp_ns = cooperativeTimeNanoseconds(message.header.stamp),
      .intent_generation = message.intent_generation,
      .valid_from_ns = cooperativeTimeNanoseconds(message.valid_from),
      .valid_until_ns = cooperativeTimeNanoseconds(message.valid_until),
      .footprint_radius_m = message.footprint_radius_m,
      .footprint_lower_extent_m = message.footprint_lower_extent_m,
      .footprint_upper_extent_m = message.footprint_upper_extent_m,
      .current_position = point(message.current_position),
      .current_velocity = vector(message.current_velocity),
      .maneuver_state = maneuver(message.maneuver_state),
      .conflict_generation = message.conflict_generation,
      .conflicting_vehicle_ids = message.conflicting_vehicle_ids,
      .passage = cooperativePassageUseData(message.passage),
      .trajectory = {},
  };
  result.trajectory.reserve(message.trajectory.size());
  for (const msg::CooperativeTrajectoryPoint& sample : message.trajectory) {
    result.trajectory.push_back(CooperativeTrajectorySample{
        .time_ns = result.valid_from_ns +
                   static_cast<std::int64_t>(
                       static_cast<double>(sample.time_from_valid_from_s) * 1.0e9),
        .position = point(sample.position),
        .velocity = vector(sample.velocity),
    });
  }
  return result;
}

msg::CooperativeFlightIntent
cooperativeFlightIntentMessage(const CooperativeFlightIntentData& intent) {
  msg::CooperativeFlightIntent message;
  message.header.stamp = cooperativeTimeMessage(intent.stamp_ns);
  message.header.frame_id = intent.frame_id;
  message.vehicle_id = intent.vehicle_id;
  message.intent_generation = intent.intent_generation;
  message.valid_from = cooperativeTimeMessage(intent.valid_from_ns);
  message.valid_until = cooperativeTimeMessage(intent.valid_until_ns);
  message.footprint_radius_m = static_cast<float>(intent.footprint_radius_m);
  message.footprint_lower_extent_m =
      static_cast<float>(intent.footprint_lower_extent_m);
  message.footprint_upper_extent_m =
      static_cast<float>(intent.footprint_upper_extent_m);
  assign(message.current_position, intent.current_position);
  assign(message.current_velocity, intent.current_velocity);
  message.maneuver_state = static_cast<std::uint8_t>(intent.maneuver_state);
  message.conflict_generation = intent.conflict_generation;
  message.conflicting_vehicle_ids = intent.conflicting_vehicle_ids;
  assignPassage(message.passage, intent.passage);
  message.trajectory.reserve(intent.trajectory.size());
  for (const CooperativeTrajectorySample& sample : intent.trajectory) {
    message.trajectory.push_back(trajectoryPointMessage(sample, intent.valid_from_ns));
  }
  return message;
}

msg::CooperativePeerTrajectory
cooperativePeerTrajectoryMessage(const CooperativeFlightIntentData& intent) {
  msg::CooperativePeerTrajectory message;
  message.vehicle_id = intent.vehicle_id;
  message.footprint_radius_m = static_cast<float>(intent.footprint_radius_m);
  message.footprint_lower_extent_m =
      static_cast<float>(intent.footprint_lower_extent_m);
  message.footprint_upper_extent_m =
      static_cast<float>(intent.footprint_upper_extent_m);
  message.valid_from = cooperativeTimeMessage(intent.valid_from_ns);
  message.valid_until = cooperativeTimeMessage(intent.valid_until_ns);
  message.trajectory.reserve(intent.trajectory.size());
  for (const CooperativeTrajectorySample& sample : intent.trajectory) {
    message.trajectory.push_back(trajectoryPointMessage(sample, intent.valid_from_ns));
  }
  return message;
}

CooperativePeerTrajectoryData
cooperativePeerTrajectoryData(const msg::CooperativePeerTrajectory& message) {
  CooperativePeerTrajectoryData result{
      .vehicle_id = message.vehicle_id,
      .valid_from_ns = cooperativeTimeNanoseconds(message.valid_from),
      .valid_until_ns = cooperativeTimeNanoseconds(message.valid_until),
      .footprint_radius_m = message.footprint_radius_m,
      .footprint_lower_extent_m = message.footprint_lower_extent_m,
      .footprint_upper_extent_m = message.footprint_upper_extent_m,
      .trajectory = {},
  };
  result.trajectory.reserve(message.trajectory.size());
  for (const msg::CooperativeTrajectoryPoint& sample : message.trajectory) {
    result.trajectory.push_back(CooperativeTrajectorySample{
        .time_ns = result.valid_from_ns +
                   static_cast<std::int64_t>(
                       static_cast<double>(sample.time_from_valid_from_s) * 1.0e9),
        .position = point(sample.position),
        .velocity = vector(sample.velocity),
    });
  }
  return result;
}

CooperativeManeuverCommandData
cooperativeManeuverCommandData(const msg::CooperativeManeuverCommand& message) {
  CooperativeManeuverCommandData result{
      .vehicle_id = message.vehicle_id,
      .stamp_ns = cooperativeTimeNanoseconds(message.header.stamp),
      .command_generation = message.command_generation,
      .valid_until_ns = cooperativeTimeNanoseconds(message.valid_until),
      .avoidance_active = message.avoidance_active,
      .preferred_maneuver = maneuver(message.preferred_maneuver),
      .preferred_acceleration_direction =
          vector(message.preferred_acceleration_direction),
      .conflict_generation = message.conflict_generation,
      .space_time_plan_active = message.space_time_plan_active,
      .space_time_lateral_offset_m = message.space_time_lateral_offset_m,
      .space_time_vertical_offset_m = message.space_time_vertical_offset_m,
      .space_time_shift_s = message.space_time_shift_s,
      .space_time_predicted_minimum_separation_m =
          message.space_time_predicted_minimum_separation_m,
      .space_time_integrated_shortfall_m2_s =
          message.space_time_integrated_shortfall_m2_s,
      .space_time_evaluated_candidate_count =
          message.space_time_evaluated_candidate_count,
      .passage_yield_required = message.passage_yield_required,
      .passage_yield_to_vehicle_id = message.passage_yield_to_vehicle_id,
      .passage_traversal_id = PassageTraversalId{message.passage_traversal_id},
      .passage_conflict_resource_id =
          CooperativeConflictResourceId{message.passage_conflict_resource_id},
      .passage_route_generation = message.passage_route_generation,
      .passage_lateral_offset_m = message.passage_lateral_offset_m,
      .passage_minimum_lateral_offset_m = message.passage_minimum_lateral_offset_m,
      .passage_maximum_lateral_offset_m = message.passage_maximum_lateral_offset_m,
      .passage_entry_not_before_ns =
          cooperativeTimeNanoseconds(message.passage_entry_not_before),
      .conflicting_peers = {},
  };
  result.conflicting_peers.reserve(message.conflicting_peers.size());
  for (const msg::CooperativePeerTrajectory& peer : message.conflicting_peers) {
    result.conflicting_peers.push_back(cooperativePeerTrajectoryData(peer));
  }
  return result;
}

} // namespace drone_city_nav

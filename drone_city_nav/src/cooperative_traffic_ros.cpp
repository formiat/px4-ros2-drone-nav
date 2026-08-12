#include "drone_city_nav/cooperative_traffic_ros.hpp"

#include "drone_city_nav/msg/cooperative_channel_intent.hpp"
#include "drone_city_nav/msg/cooperative_trajectory_point.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

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

[[nodiscard]] CooperativeChannelPhase channelPhase(const std::uint8_t value) noexcept {
  if (value <= static_cast<std::uint8_t>(CooperativeChannelPhase::kDeparture)) {
    return static_cast<CooperativeChannelPhase>(value);
  }
  return CooperativeChannelPhase::kNone;
}

[[nodiscard]] CooperativeChannelUse
channelUseImpl(const msg::CooperativeChannelIntent& message) {
  return CooperativeChannelUse{
      .channel_id = message.channel_id,
      .conflict_resource_id = message.conflict_resource_id,
      .route_generation = message.route_generation,
      .phase = channelPhase(message.phase),
      .lane_index = message.lane_index,
      .lane_count = message.lane_count,
      .direction_sign = message.direction_sign,
      .station_m = message.station_m,
      .distance_to_entry_m = message.distance_to_entry_m,
      .distance_to_exit_m = message.distance_to_exit_m,
      .predicted_entry_ns = cooperativeTimeNanoseconds(message.predicted_entry),
      .predicted_exit_ns = cooperativeTimeNanoseconds(message.predicted_exit),
  };
}

void assignChannel(msg::CooperativeChannelIntent& message,
                   const CooperativeChannelUse& channel) {
  message.channel_id = channel.channel_id;
  message.conflict_resource_id = channel.conflict_resource_id;
  message.route_generation = channel.route_generation;
  message.phase = static_cast<std::uint8_t>(channel.phase);
  message.lane_index = static_cast<std::uint16_t>(
      std::min(channel.lane_index,
               static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max())));
  message.lane_count = static_cast<std::uint16_t>(
      std::min(channel.lane_count,
               static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max())));
  message.direction_sign =
      static_cast<std::int8_t>(std::clamp(channel.direction_sign, -1, 1));
  message.station_m = channel.station_m;
  message.distance_to_entry_m = channel.distance_to_entry_m;
  message.distance_to_exit_m = channel.distance_to_exit_m;
  message.predicted_entry = cooperativeTimeMessage(channel.predicted_entry_ns);
  message.predicted_exit = cooperativeTimeMessage(channel.predicted_exit_ns);
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

CooperativeChannelUse
cooperativeChannelUseData(const msg::CooperativeChannelIntent& message) {
  return channelUseImpl(message);
}

msg::CooperativeChannelIntent
cooperativeChannelIntentMessage(const CooperativeChannelUse& channel) {
  msg::CooperativeChannelIntent message;
  assignChannel(message, channel);
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
      .channel = cooperativeChannelUseData(message.channel),
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
  assignChannel(message.channel, intent.channel);
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

} // namespace drone_city_nav

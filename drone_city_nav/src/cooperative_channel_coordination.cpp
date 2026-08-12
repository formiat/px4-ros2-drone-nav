#include "drone_city_nav/cooperative_channel_coordination.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) noexcept {
  return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

[[nodiscard]] bool timeWindowsOverlap(const CooperativeChannelUse& first,
                                      const CooperativeChannelUse& second,
                                      const double margin_s) noexcept {
  if (first.predicted_entry_ns <= 0 || first.predicted_exit_ns <= 0 ||
      second.predicted_entry_ns <= 0 || second.predicted_exit_ns <= 0) {
    return false;
  }
  const std::int64_t margin_ns = secondsToNanoseconds(margin_s);
  return first.predicted_entry_ns - margin_ns <= second.predicted_exit_ns + margin_ns &&
         second.predicted_entry_ns - margin_ns <= first.predicted_exit_ns + margin_ns;
}

[[nodiscard]] bool
ownshipWinsRightOfWay(const CooperativeFlightIntentData& ownship,
                      const CooperativeFlightIntentData& peer,
                      const CooperativeChannelCoordinationConfig& config) noexcept {
  if (ownship.channel.phase == CooperativeChannelPhase::kTraversal) {
    return true;
  }
  if (peer.channel.phase == CooperativeChannelPhase::kTraversal) {
    return false;
  }
  const std::int64_t headway_ns =
      secondsToNanoseconds(config.same_lane_entry_headway_s);
  if (ownship.channel.predicted_entry_ns + headway_ns <
      peer.channel.predicted_entry_ns) {
    return true;
  }
  if (peer.channel.predicted_entry_ns + headway_ns <
      ownship.channel.predicted_entry_ns) {
    return false;
  }
  return ownship.vehicle_id < peer.vehicle_id;
}

} // namespace

std::size_t assignCooperativeChannelLane(const int direction_sign,
                                         const std::size_t lane_count) noexcept {
  if (lane_count <= 1U || direction_sign >= 0) {
    return 0U;
  }
  return lane_count - 1U;
}

CooperativeChannelDecision coordinateCooperativeChannel(
    const CooperativeFlightIntentData& ownship,
    const std::span<const CooperativeFlightIntentData> peers,
    const CooperativeChannelCoordinationConfig& config) noexcept {
  CooperativeChannelDecision result;
  if (!ownship.channel.active() || !(config.reservation_time_margin_s >= 0.0) ||
      !(config.same_lane_entry_headway_s >= 0.0)) {
    return result;
  }
  result.active = true;
  result.lane_count = ownship.channel.lane_count;
  result.lane_index = assignCooperativeChannelLane(ownship.channel.direction_sign,
                                                   ownship.channel.lane_count);
  if (ownship.channel.phase != CooperativeChannelPhase::kApproach) {
    return result;
  }

  for (const CooperativeFlightIntentData& peer : peers) {
    if (peer.vehicle_id.empty() || peer.vehicle_id == ownship.vehicle_id ||
        !peer.channel.active() ||
        peer.channel.conflict_resource_id != ownship.channel.conflict_resource_id ||
        !timeWindowsOverlap(ownship.channel, peer.channel,
                            config.reservation_time_margin_s)) {
      continue;
    }
    const std::size_t shared_lane_count =
        std::min(ownship.channel.lane_count, peer.channel.lane_count);
    const bool exclusive_resource = shared_lane_count <= 1U;
    const bool different_movement =
        ownship.channel.channel_id != peer.channel.channel_id;
    const std::size_t peer_lane =
        assignCooperativeChannelLane(peer.channel.direction_sign, shared_lane_count);
    const std::size_t own_lane =
        assignCooperativeChannelLane(ownship.channel.direction_sign, shared_lane_count);
    const bool same_direction =
        ownship.channel.direction_sign == peer.channel.direction_sign;
    const bool separate_opposite_lanes =
        !same_direction && shared_lane_count > 1U && own_lane != peer_lane;
    if (!exclusive_resource && !different_movement && separate_opposite_lanes) {
      continue;
    }

    const CooperativeConflictPrediction prediction =
        predictCooperativeConflict(ownship, peer, ownship.stamp_ns, config.conflict);
    const bool same_lane_following =
        !different_movement && same_direction && own_lane == peer_lane;
    const bool conflict_relevant =
        exclusive_resource || ((different_movement || same_lane_following) &&
                               prediction.valid && prediction.conflict_predicted);
    if (!conflict_relevant || ownshipWinsRightOfWay(ownship, peer, config)) {
      continue;
    }
    result.yield_before_entry = true;
    result.conflict_zone_only = different_movement && !exclusive_resource;
    if (result.yield_to_vehicle_id.empty() ||
        peer.vehicle_id < result.yield_to_vehicle_id) {
      result.yield_to_vehicle_id = peer.vehicle_id;
    }
  }
  return result;
}

} // namespace drone_city_nav

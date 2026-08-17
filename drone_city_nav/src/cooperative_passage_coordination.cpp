#include "drone_city_nav/cooperative_passage_coordination.hpp"

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

[[nodiscard]] bool timeWindowsOverlap(const CooperativeConflictResourceUse& first,
                                      const CooperativeConflictResourceUse& second,
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
                      const CooperativeConflictResourceUse& ownship_resource,
                      const CooperativeConflictResourceUse& peer_resource,
                      const CooperativePassageCoordinationConfig& config) noexcept {
  const bool ownship_occupies_resource =
      ownship.passage.phase == CooperativePassagePhase::kTraversal &&
      ownship.passage.station_m + 1.0e-6 >= ownship_resource.begin_station_m &&
      ownship.passage.station_m - 1.0e-6 <= ownship_resource.end_station_m;
  const bool peer_occupies_resource =
      peer.passage.phase == CooperativePassagePhase::kTraversal &&
      peer.passage.station_m + 1.0e-6 >= peer_resource.begin_station_m &&
      peer.passage.station_m - 1.0e-6 <= peer_resource.end_station_m;
  if (ownship_occupies_resource != peer_occupies_resource) {
    return ownship_occupies_resource;
  }
  if (ownship_occupies_resource) {
    return ownship.vehicle_id < peer.vehicle_id;
  }
  const std::int64_t headway_ns =
      secondsToNanoseconds(config.same_path_entry_headway_s);
  if (ownship_resource.predicted_entry_ns + headway_ns <
      peer_resource.predicted_entry_ns) {
    return true;
  }
  if (peer_resource.predicted_entry_ns + headway_ns <
      ownship_resource.predicted_entry_ns) {
    return false;
  }
  return ownship.vehicle_id < peer.vehicle_id;
}

[[nodiscard]] double sharedUsableWidthM(const CooperativePassageUse& first,
                                        const CooperativePassageUse& second) noexcept {
  return std::max(
      0.0,
      std::min(first.maximum_lateral_offset_m, second.maximum_lateral_offset_m) -
          std::max(first.minimum_lateral_offset_m, second.minimum_lateral_offset_m));
}

[[nodiscard]] std::int64_t
requestedEntryTime(const bool same_direction,
                   const CooperativeConflictResourceUse& peer_resource,
                   const CooperativePassageCoordinationConfig& config) noexcept {
  if (same_direction) {
    return peer_resource.predicted_entry_ns +
           secondsToNanoseconds(config.same_path_entry_headway_s);
  }
  return peer_resource.predicted_exit_ns +
         secondsToNanoseconds(config.reservation_time_margin_s);
}

} // namespace

double passageLateralSeparationM(const CooperativePassageUse& first,
                                 const CooperativePassageUse& second) noexcept {
  if (first.passage_traversal_id != second.passage_traversal_id) {
    return 0.0;
  }
  return std::abs(first.lateral_offset_m - second.lateral_offset_m);
}

CooperativePassageDecision coordinateCooperativePassage(
    const CooperativeFlightIntentData& ownship,
    const std::span<const CooperativeFlightIntentData> peers,
    const CooperativePassageCoordinationConfig& config) noexcept {
  CooperativePassageDecision result;
  if (!ownship.passage.active() || !(config.reservation_time_margin_s >= 0.0) ||
      !(config.same_path_entry_headway_s >= 0.0) ||
      !(config.lateral_separation_tolerance_m >= 0.0)) {
    return result;
  }
  result.active = true;
  result.lateral_offset_m = ownship.passage.lateral_offset_m;
  if (ownship.passage.phase != CooperativePassagePhase::kApproach) {
    return result;
  }

  for (const CooperativeFlightIntentData& peer : peers) {
    if (peer.vehicle_id.empty() || peer.vehicle_id == ownship.vehicle_id ||
        !peer.passage.active()) {
      continue;
    }
    const bool different_movement =
        ownship.passage.passage_traversal_id != peer.passage.passage_traversal_id;
    const bool same_direction = !different_movement && ownship.passage.direction_sign ==
                                                           peer.passage.direction_sign;
    const double required_separation_m =
        std::max(ownship.passage.desired_center_separation_m,
                 peer.passage.desired_center_separation_m);
    const bool offsets_separated =
        !different_movement &&
        passageLateralSeparationM(ownship.passage, peer.passage) +
                config.lateral_separation_tolerance_m >=
            required_separation_m;
    if (same_direction && !offsets_separated &&
        peer.passage.station_m > ownship.passage.station_m) {
      const double candidate_hold_station_m = std::max(
          ownship.passage.station_m, peer.passage.station_m - required_separation_m);
      if (!result.queue_hold_station_valid ||
          candidate_hold_station_m < result.queue_hold_station_m) {
        result.queue_hold_station_valid = true;
        result.queue_hold_station_m = candidate_hold_station_m;
      }
    }
    const CooperativeConflictPrediction prediction =
        predictCooperativeConflict(ownship, peer, ownship.stamp_ns, config.conflict);
    for (const CooperativeConflictResourceUse& ownship_resource :
         ownship.passage.conflict_resources) {
      for (const CooperativeConflictResourceUse& peer_resource :
           peer.passage.conflict_resources) {
        if (!ownship_resource.active() || !peer_resource.active() ||
            ownship_resource.conflict_resource_id !=
                peer_resource.conflict_resource_id ||
            !timeWindowsOverlap(ownship_resource, peer_resource,
                                config.reservation_time_margin_s)) {
          continue;
        }
        const bool exclusive_resource =
            sharedUsableWidthM(ownship.passage, peer.passage) +
                config.lateral_separation_tolerance_m <
            required_separation_m;
        if (!exclusive_resource && offsets_separated) {
          continue;
        }
        const bool conflict_relevant =
            exclusive_resource ||
            ((different_movement || same_direction || !offsets_separated) &&
             prediction.valid && prediction.conflict_predicted);
        if (!conflict_relevant || ownshipWinsRightOfWay(ownship, peer, ownship_resource,
                                                        peer_resource, config)) {
          continue;
        }
        const bool conflict_zone_only = different_movement && !exclusive_resource;
        if (!result.yield_before_entry) {
          result.conflict_zone_only = conflict_zone_only;
        } else {
          result.conflict_zone_only = result.conflict_zone_only && conflict_zone_only;
        }
        result.yield_before_entry = true;
        result.entry_not_before_ns =
            std::max(result.entry_not_before_ns,
                     requestedEntryTime(same_direction, peer_resource, config));
        if (result.yield_to_vehicle_id.empty() ||
            peer.vehicle_id < result.yield_to_vehicle_id) {
          result.yield_to_vehicle_id = peer.vehicle_id;
        }
        if (result.conflict_resource_id.empty() ||
            ownship_resource.conflict_resource_id.value() <
                result.conflict_resource_id.value()) {
          result.conflict_resource_id = ownship_resource.conflict_resource_id;
        }
      }
    }
  }
  return result;
}

} // namespace drone_city_nav

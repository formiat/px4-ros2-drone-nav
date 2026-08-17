#include "drone_city_nav/cooperative_traffic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <set>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool validIntent(const CooperativeFlightIntentData& intent) noexcept {
  if (intent.vehicle_id.empty() || intent.frame_id.empty() || intent.stamp_ns <= 0 ||
      intent.intent_generation == 0U || intent.valid_from_ns <= 0 ||
      intent.valid_until_ns < intent.valid_from_ns ||
      !(intent.footprint_radius_m > 0.0) || !(intent.footprint_lower_extent_m >= 0.0) ||
      !(intent.footprint_upper_extent_m >= 0.0) || !finite(intent.current_position) ||
      !finite(intent.current_velocity) || intent.trajectory.empty() ||
      intent.trajectory.size() > 4096U) {
    return false;
  }
  std::int64_t previous_time_ns = 0;
  for (const CooperativeTrajectorySample& sample : intent.trajectory) {
    if (sample.time_ns < intent.valid_from_ns ||
        sample.time_ns > intent.valid_until_ns ||
        (previous_time_ns > 0 && sample.time_ns <= previous_time_ns) ||
        !finite(sample.position) || !finite(sample.velocity)) {
      return false;
    }
    previous_time_ns = sample.time_ns;
  }
  return true;
}

[[nodiscard]] Point3 interpolate(const Point3& first, const Point3& second,
                                 const double fraction) noexcept {
  return Point3{first.x + fraction * (second.x - first.x),
                first.y + fraction * (second.y - first.y),
                first.z + fraction * (second.z - first.z)};
}

[[nodiscard]] Vec3 interpolate(const Vec3& first, const Vec3& second,
                               const double fraction) noexcept {
  return Vec3{first.x + fraction * (second.x - first.x),
              first.y + fraction * (second.y - first.y),
              first.z + fraction * (second.z - first.z)};
}

[[nodiscard]] Vec3 subtract(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] Vec3 subtract(const Vec3& first, const Vec3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] double squaredNorm(const Vec3& vector) noexcept {
  return dot(vector, vector);
}

[[nodiscard]] double norm(const Vec3& vector) noexcept {
  return std::sqrt(squaredNorm(vector));
}

void appendTimesWithin(const CooperativeFlightIntentData& intent,
                       const std::int64_t begin_ns, const std::int64_t end_ns,
                       std::vector<std::int64_t>& times) {
  for (const CooperativeTrajectorySample& sample : intent.trajectory) {
    if (sample.time_ns > begin_ns && sample.time_ns < end_ns) {
      times.push_back(sample.time_ns);
    }
  }
}

[[nodiscard]] std::uint64_t stablePairHash(const std::string_view first,
                                           const std::string_view second) noexcept {
  constexpr std::uint64_t kOffset{1469598103934665603ULL};
  constexpr std::uint64_t kPrime{1099511628211ULL};
  std::uint64_t hash = kOffset;
  const auto append = [&](const std::string_view value) {
    for (const char character : value) {
      hash ^= static_cast<std::uint8_t>(character);
      hash *= kPrime;
    }
    hash ^= 0xffU;
    hash *= kPrime;
  };
  if (first < second) {
    append(first);
    append(second);
  } else {
    append(second);
    append(first);
  }
  return hash;
}

} // namespace

std::vector<CooperativeTrajectorySample> makeStationaryCooperativeTrajectory(
    const Point3 hold_position, const std::int64_t valid_from_ns,
    const std::int64_t valid_until_ns, const std::int64_t now_ns) {
  if (valid_from_ns <= 0 || valid_until_ns < valid_from_ns) {
    return {};
  }
  const std::int64_t first_sample_ns = std::max(valid_from_ns, now_ns);
  if (first_sample_ns > valid_until_ns) {
    return {};
  }

  std::vector<CooperativeTrajectorySample> result;
  result.reserve(2U);
  result.push_back(CooperativeTrajectorySample{
      .time_ns = first_sample_ns,
      .position = hold_position,
      .velocity = Vec3{},
  });
  if (valid_until_ns > first_sample_ns) {
    result.push_back(CooperativeTrajectorySample{
        .time_ns = valid_until_ns,
        .position = hold_position,
        .velocity = Vec3{},
    });
  }
  return result;
}

CooperativePairManeuverPreference
preferredCooperativePairManeuver(const CooperativeFlightIntentData& ownship,
                                 const CooperativeFlightIntentData& peer) noexcept {
  const bool ownship_is_first = ownship.vehicle_id < peer.vehicle_id;
  if ((stablePairHash(ownship.vehicle_id, peer.vehicle_id) & 1U) == 0U) {
    return ownship_is_first
               ? CooperativePairManeuverPreference{CooperativeManeuver::kClimb,
                                                   Vec3{0.0, 0.0, 1.0}}
               : CooperativePairManeuverPreference{CooperativeManeuver::kDescend,
                                                   Vec3{0.0, 0.0, -1.0}};
  }

  const Point3 first_position =
      ownship_is_first ? ownship.current_position : peer.current_position;
  const Point3 second_position =
      ownship_is_first ? peer.current_position : ownship.current_position;
  const double dx = second_position.x - first_position.x;
  const double dy = second_position.y - first_position.y;
  const double length = std::hypot(dx, dy);
  Vec3 first_direction{0.0, 1.0, 0.0};
  if (length > 1.0e-6) {
    first_direction = Vec3{-dy / length, dx / length, 0.0};
  }
  const Vec3 own_direction =
      ownship_is_first
          ? first_direction
          : Vec3{-first_direction.x, -first_direction.y, -first_direction.z};
  const double own_speed =
      std::hypot(ownship.current_velocity.x, ownship.current_velocity.y);
  if (own_speed <= 1.0e-6) {
    return CooperativePairManeuverPreference{
        ownship_is_first ? CooperativeManeuver::kLeft : CooperativeManeuver::kRight,
        own_direction};
  }
  const Vec3 own_left{-ownship.current_velocity.y / own_speed,
                      ownship.current_velocity.x / own_speed, 0.0};
  return CooperativePairManeuverPreference{dot(own_direction, own_left) >= 0.0
                                               ? CooperativeManeuver::kLeft
                                               : CooperativeManeuver::kRight,
                                           own_direction};
}

namespace {

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) noexcept {
  return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

[[nodiscard]] std::optional<CooperativeTrajectorySample>
sampleTrajectory(const std::span<const CooperativeTrajectorySample> trajectory,
                 const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
                 const std::int64_t time_ns) noexcept {
  if (trajectory.empty() || time_ns < valid_from_ns || time_ns > valid_until_ns ||
      time_ns < trajectory.front().time_ns || time_ns > trajectory.back().time_ns) {
    return std::nullopt;
  }
  const auto next = std::ranges::lower_bound(trajectory, time_ns, {},
                                             &CooperativeTrajectorySample::time_ns);
  if (next == trajectory.end()) {
    return trajectory.back();
  }
  if (next->time_ns == time_ns || next == trajectory.begin()) {
    return *next;
  }
  const auto previous = std::prev(next);
  const double fraction = static_cast<double>(time_ns - previous->time_ns) /
                          static_cast<double>(next->time_ns - previous->time_ns);
  return CooperativeTrajectorySample{
      .time_ns = time_ns,
      .position = interpolate(previous->position, next->position, fraction),
      .velocity = interpolate(previous->velocity, next->velocity, fraction),
  };
}

} // namespace

CooperativePeerStore::CooperativePeerStore(std::string own_vehicle_id,
                                           const CooperativePeerStoreConfig& config)
    : own_vehicle_id_{std::move(own_vehicle_id)},
      config_{config} {
  if (own_vehicle_id_.empty() || !(config_.maximum_publication_age_s > 0.0) ||
      config_.maximum_peers == 0U || config_.maximum_peers > 1024U) {
    throw std::invalid_argument{"invalid cooperative peer store configuration"};
  }
}

CooperativePeerUpdateStatus
CooperativePeerStore::update(const CooperativeFlightIntentData& intent,
                             const std::int64_t now_ns) {
  if (intent.vehicle_id == own_vehicle_id_) {
    return CooperativePeerUpdateStatus::kIgnoredOwnship;
  }
  if (now_ns <= 0 || !validIntent(intent)) {
    return CooperativePeerUpdateStatus::kInvalid;
  }
  const std::int64_t maximum_age_ns =
      secondsToNanoseconds(config_.maximum_publication_age_s);
  if (now_ns > intent.valid_until_ns || now_ns - intent.stamp_ns > maximum_age_ns) {
    return CooperativePeerUpdateStatus::kStale;
  }
  static_cast<void>(activeIntents(now_ns));
  const auto existing = intents_.find(intent.vehicle_id);
  if (existing != intents_.end() &&
      (intent.intent_generation < existing->second.intent_generation ||
       (intent.intent_generation == existing->second.intent_generation &&
        intent.stamp_ns <= existing->second.stamp_ns))) {
    return CooperativePeerUpdateStatus::kOutOfOrder;
  }
  if (existing == intents_.end() && intents_.size() >= config_.maximum_peers) {
    return CooperativePeerUpdateStatus::kInvalid;
  }
  intents_.insert_or_assign(intent.vehicle_id, intent);
  return CooperativePeerUpdateStatus::kAccepted;
}

std::vector<CooperativeFlightIntentData>
CooperativePeerStore::activeIntents(const std::int64_t now_ns) {
  const std::int64_t maximum_age_ns =
      secondsToNanoseconds(config_.maximum_publication_age_s);
  std::erase_if(intents_, [&](const auto& entry) {
    const CooperativeFlightIntentData& intent = entry.second;
    return now_ns <= 0 || now_ns > intent.valid_until_ns ||
           now_ns - intent.stamp_ns > maximum_age_ns;
  });
  std::vector<CooperativeFlightIntentData> result;
  result.reserve(intents_.size());
  for (const auto& [vehicle_id, intent] : intents_) {
    static_cast<void>(vehicle_id);
    result.push_back(intent);
  }
  return result;
}

void CooperativePeerStore::clear() noexcept {
  intents_.clear();
}

std::optional<CooperativeTrajectorySample>
sampleCooperativeTrajectory(const CooperativeFlightIntentData& intent,
                            const std::int64_t time_ns) noexcept {
  return sampleTrajectory(intent.trajectory, intent.valid_from_ns,
                          intent.valid_until_ns, time_ns);
}

std::optional<CooperativeTrajectorySample>
sampleCooperativeTrajectory(const CooperativePeerTrajectoryData& trajectory,
                            const std::int64_t time_ns) noexcept {
  return sampleTrajectory(trajectory.trajectory, trajectory.valid_from_ns,
                          trajectory.valid_until_ns, time_ns);
}

CooperativeConflictPrediction predictCooperativeConflict(
    const CooperativeFlightIntentData& ownship, const CooperativeFlightIntentData& peer,
    const std::int64_t now_ns, const CooperativeConflictConfig& config) noexcept {
  CooperativeConflictPrediction result;
  if (now_ns <= 0 || ownship.vehicle_id.empty() || peer.vehicle_id.empty() ||
      ownship.vehicle_id == peer.vehicle_id || ownship.frame_id != peer.frame_id ||
      !(config.prediction_horizon_s > 0.0) ||
      !(config.desired_minimum_separation_m > 0.0) ||
      !(config.release_separation_m > config.desired_minimum_separation_m) ||
      ownship.trajectory.empty() || peer.trajectory.empty()) {
    return result;
  }
  const Vec3 current_relative_position =
      subtract(peer.current_position, ownship.current_position);
  const Vec3 current_relative_velocity =
      subtract(peer.current_velocity, ownship.current_velocity);
  result.current_separation_m = norm(current_relative_position);
  if (result.current_separation_m > 1.0e-9) {
    const double distance_rate_mps =
        dot(current_relative_position, current_relative_velocity) /
        result.current_separation_m;
    result.current_closing_speed_mps = -distance_rate_mps;
    result.separating = distance_rate_mps > 0.0;
  }

  double minimum_squared_m = result.current_separation_m * result.current_separation_m;
  std::int64_t minimum_time_ns = now_ns;
  if (!std::isfinite(minimum_squared_m)) {
    return result;
  }

  const std::int64_t horizon_end_ns =
      now_ns + secondsToNanoseconds(config.prediction_horizon_s);
  const std::int64_t begin_ns =
      std::max({now_ns, ownship.valid_from_ns, peer.valid_from_ns,
                ownship.trajectory.front().time_ns, peer.trajectory.front().time_ns});
  const std::int64_t end_ns =
      std::min({horizon_end_ns, ownship.valid_until_ns, peer.valid_until_ns,
                ownship.trajectory.back().time_ns, peer.trajectory.back().time_ns});
  if (end_ns < begin_ns) {
    result.valid = true;
    result.minimum_separation_m = result.current_separation_m;
    result.time_to_minimum_s = 0.0;
    result.conflict_predicted =
        result.current_separation_m < config.desired_minimum_separation_m;
    return result;
  }

  std::vector<std::int64_t> times{begin_ns, end_ns};
  appendTimesWithin(ownship, begin_ns, end_ns, times);
  appendTimesWithin(peer, begin_ns, end_ns, times);
  std::ranges::sort(times);
  times.erase(std::unique(times.begin(), times.end()), times.end());

  if (times.size() == 1U) {
    const auto own_sample = sampleCooperativeTrajectory(ownship, times.front());
    const auto peer_sample = sampleCooperativeTrajectory(peer, times.front());
    if (!own_sample.has_value() || !peer_sample.has_value()) {
      return result;
    }
    const double squared_m =
        squaredNorm(subtract(peer_sample->position, own_sample->position));
    if (squared_m < minimum_squared_m) {
      minimum_squared_m = squared_m;
      minimum_time_ns = times.front();
    }
  } else {
    for (std::size_t index = 0U; index + 1U < times.size(); ++index) {
      const auto own_begin = sampleCooperativeTrajectory(ownship, times[index]);
      const auto own_end = sampleCooperativeTrajectory(ownship, times[index + 1U]);
      const auto peer_begin = sampleCooperativeTrajectory(peer, times[index]);
      const auto peer_end = sampleCooperativeTrajectory(peer, times[index + 1U]);
      if (!own_begin.has_value() || !own_end.has_value() || !peer_begin.has_value() ||
          !peer_end.has_value()) {
        return result;
      }
      const Vec3 relative_begin = subtract(peer_begin->position, own_begin->position);
      const Vec3 relative_end = subtract(peer_end->position, own_end->position);
      const Vec3 relative_delta = subtract(relative_end, relative_begin);
      const double denominator = squaredNorm(relative_delta);
      const double fraction =
          denominator > 1.0e-12
              ? std::clamp(-dot(relative_begin, relative_delta) / denominator, 0.0, 1.0)
              : 0.0;
      const Vec3 closest{
          relative_begin.x + fraction * relative_delta.x,
          relative_begin.y + fraction * relative_delta.y,
          relative_begin.z + fraction * relative_delta.z,
      };
      const double squared_m = squaredNorm(closest);
      if (squared_m < minimum_squared_m) {
        minimum_squared_m = squared_m;
        minimum_time_ns =
            times[index] +
            static_cast<std::int64_t>(std::llround(
                fraction * static_cast<double>(times[index + 1U] - times[index])));
      }
    }
  }
  result.valid = std::isfinite(minimum_squared_m);
  if (!result.valid) {
    return result;
  }
  result.minimum_separation_m = std::sqrt(std::max(0.0, minimum_squared_m));
  result.time_to_minimum_s = std::max(
      0.0, static_cast<double>(minimum_time_ns - now_ns) / kNanosecondsPerSecond);
  result.conflict_predicted =
      result.minimum_separation_m < config.desired_minimum_separation_m;
  return result;
}

CooperativeConflictLifecycle::CooperativeConflictLifecycle(
    const CooperativeConflictConfig& config)
    : config_{config} {
  if (!(config_.prediction_horizon_s > 0.0) ||
      !(config_.desired_minimum_separation_m > 0.0) ||
      !(config_.release_separation_m > config_.desired_minimum_separation_m) ||
      !(config_.minimum_maneuver_latch_s >= 0.0) ||
      !(config_.release_confirmation_s >= 0.0)) {
    throw std::invalid_argument{"invalid cooperative conflict configuration"};
  }
}

CooperativeAvoidanceDecision CooperativeConflictLifecycle::update(
    const std::int64_t now_ns, const CooperativeFlightIntentData& ownship,
    const std::span<const CooperativeFlightIntentData> peers) {
  CooperativeAvoidanceDecision result;
  if (now_ns <= 0 || ownship.vehicle_id.empty()) {
    return result;
  }
  bool changed = false;
  std::set<std::string> observed_peer_ids;
  for (const CooperativeFlightIntentData& peer : peers) {
    if (peer.vehicle_id.empty() || peer.vehicle_id == ownship.vehicle_id) {
      continue;
    }
    observed_peer_ids.insert(peer.vehicle_id);
    const CooperativeConflictPrediction prediction =
        predictCooperativeConflict(ownship, peer, now_ns, config_);
    if (!prediction.valid) {
      continue;
    }
    auto conflict = conflicts_.find(peer.vehicle_id);
    if (conflict == conflicts_.end() && prediction.conflict_predicted) {
      const CooperativePairManeuverPreference preference =
          preferredCooperativePairManeuver(ownship, peer);
      conflict =
          conflicts_
              .emplace(peer.vehicle_id,
                       LatchedConflict{
                           .maneuver = preference.maneuver,
                           .acceleration_direction = preference.acceleration_direction,
                           .latched_until_ns =
                               now_ns +
                               secondsToNanoseconds(config_.minimum_maneuver_latch_s),
                           .release_candidate_since_ns = std::nullopt,
                       })
              .first;
      changed = true;
    }
    if (conflict == conflicts_.end()) {
      continue;
    }

    const bool releasable =
        now_ns >= conflict->second.latched_until_ns && prediction.separating &&
        prediction.current_separation_m >= config_.release_separation_m &&
        prediction.minimum_separation_m >= config_.desired_minimum_separation_m;
    if (releasable) {
      if (!conflict->second.release_candidate_since_ns.has_value()) {
        conflict->second.release_candidate_since_ns = now_ns;
      }
      const std::int64_t release_candidate_since_ns =
          conflict->second.release_candidate_since_ns.value_or(now_ns);
      if (now_ns - release_candidate_since_ns >=
          secondsToNanoseconds(config_.release_confirmation_s)) {
        conflicts_.erase(conflict);
        changed = true;
        continue;
      }
    } else {
      conflict->second.release_candidate_since_ns.reset();
    }
    result.peers.push_back(
        CooperativeConflictPeer{.intent = peer, .prediction = prediction});
  }

  for (auto conflict = conflicts_.begin(); conflict != conflicts_.end();) {
    if (!observed_peer_ids.contains(conflict->first)) {
      conflict = conflicts_.erase(conflict);
      changed = true;
    } else {
      ++conflict;
    }
  }
  if (changed) {
    ++generation_;
  }
  result.changed = changed;
  result.conflict_generation = generation_;
  std::ranges::sort(result.peers, [](const CooperativeConflictPeer& first,
                                     const CooperativeConflictPeer& second) {
    if (first.prediction.minimum_separation_m !=
        second.prediction.minimum_separation_m) {
      return first.prediction.minimum_separation_m <
             second.prediction.minimum_separation_m;
    }
    if (first.prediction.time_to_minimum_s != second.prediction.time_to_minimum_s) {
      return first.prediction.time_to_minimum_s < second.prediction.time_to_minimum_s;
    }
    return first.intent.vehicle_id < second.intent.vehicle_id;
  });
  result.active = !result.peers.empty();
  if (!result.active) {
    primary_peer_id_.clear();
    return result;
  }
  const auto latched_primary =
      std::ranges::find(result.peers, primary_peer_id_,
                        [](const CooperativeConflictPeer& peer) -> const std::string& {
                          return peer.intent.vehicle_id;
                        });
  const CooperativeConflictPeer& primary =
      latched_primary != result.peers.end() ? *latched_primary : result.peers.front();
  primary_peer_id_ = primary.intent.vehicle_id;
  const auto latch = conflicts_.find(primary.intent.vehicle_id);
  if (latch == conflicts_.end()) {
    return result;
  }
  result.primary_peer_id = primary.intent.vehicle_id;
  result.preferred_maneuver = latch->second.maneuver;
  result.preferred_acceleration_direction = latch->second.acceleration_direction;
  result.predicted_minimum_separation_m = primary.prediction.minimum_separation_m;
  result.time_to_minimum_s = primary.prediction.time_to_minimum_s;
  return result;
}

void CooperativeConflictLifecycle::reset() noexcept {
  conflicts_.clear();
  primary_peer_id_.clear();
  generation_ = 0U;
}

std::string_view cooperativeManeuverName(const CooperativeManeuver maneuver) noexcept {
  switch (maneuver) {
    case CooperativeManeuver::kKeep:
      return "keep";
    case CooperativeManeuver::kClimb:
      return "climb";
    case CooperativeManeuver::kDescend:
      return "descend";
    case CooperativeManeuver::kLeft:
      return "left";
    case CooperativeManeuver::kRight:
      return "right";
    case CooperativeManeuver::kSlow:
      return "slow";
  }
  return "unknown";
}

std::string_view
cooperativePassagePhaseName(const CooperativePassagePhase phase) noexcept {
  switch (phase) {
    case CooperativePassagePhase::kNone:
      return "none";
    case CooperativePassagePhase::kApproach:
      return "approach";
    case CooperativePassagePhase::kTraversal:
      return "traversal";
    case CooperativePassagePhase::kDeparture:
      return "departure";
  }
  return "unknown";
}

} // namespace drone_city_nav

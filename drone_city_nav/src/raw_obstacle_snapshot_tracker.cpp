#include "drone_city_nav/raw_obstacle_snapshot_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool metadataValid(const RawObstacleSnapshotMetadata& snapshot) noexcept {
  return snapshot.grid_valid && snapshot.identity.producer_instance_id != 0U &&
         snapshot.identity.revision != 0U &&
         snapshot.identity.policy_fingerprint != 0U &&
         std::isfinite(snapshot.policy.critical_distance_m) &&
         std::isfinite(snapshot.policy.preferred_distance_m) &&
         snapshot.policy.critical_distance_m >= 0.0 &&
         snapshot.policy.preferred_distance_m >= snapshot.policy.critical_distance_m;
}

} // namespace

bool RawObstacleSnapshotTracker::accept(const RawObstacleSnapshotMetadata& snapshot) {
  if (!metadataValid(snapshot)) {
    return false;
  }
  if (std::ranges::find(retired_producers_, snapshot.identity.producer_instance_id) !=
      retired_producers_.end()) {
    return false;
  }
  if (current_valid_ &&
      snapshot.identity.producer_instance_id ==
          current_.identity.producer_instance_id &&
      snapshot.identity.revision <= current_.identity.revision) {
    return false;
  }
  if (current_valid_ && snapshot.identity.producer_instance_id !=
                            current_.identity.producer_instance_id) {
    if (std::ranges::find(retired_producers_, current_.identity.producer_instance_id) ==
        retired_producers_.end()) {
      retired_producers_.push_back(current_.identity.producer_instance_id);
    }
  }
  current_ = snapshot;
  current_valid_ = true;
  return true;
}

RawSnapshotRelation RawObstacleSnapshotTracker::relation(
    const RawObstacleSnapshotIdentity& trajectory) const noexcept {
  if (!current_valid_ || trajectory.producer_instance_id == 0U ||
      trajectory.revision == 0U || trajectory.policy_fingerprint == 0U) {
    return RawSnapshotRelation::kMalformed;
  }
  if (trajectory.producer_instance_id != current_.identity.producer_instance_id) {
    if (std::ranges::find(retired_producers_, trajectory.producer_instance_id) !=
        retired_producers_.end()) {
      return RawSnapshotRelation::kRetiredProducer;
    }
    return RawSnapshotRelation::kDifferentProducer;
  }
  if (trajectory.policy_fingerprint != current_.identity.policy_fingerprint) {
    return RawSnapshotRelation::kPolicyMismatch;
  }
  if (current_.identity.revision == trajectory.revision) {
    return RawSnapshotRelation::kExact;
  }
  return current_.identity.revision > trajectory.revision
             ? RawSnapshotRelation::kRuntimeNewer
             : RawSnapshotRelation::kRuntimeOlder;
}

const RawObstacleSnapshotMetadata*
RawObstacleSnapshotTracker::current() const noexcept {
  return current_valid_ ? &current_ : nullptr;
}

const char* rawSnapshotRelationName(const RawSnapshotRelation relation) noexcept {
  switch (relation) {
    case RawSnapshotRelation::kExact:
      return "exact";
    case RawSnapshotRelation::kRuntimeNewer:
      return "runtime_newer";
    case RawSnapshotRelation::kRuntimeOlder:
      return "runtime_older";
    case RawSnapshotRelation::kDifferentProducer:
      return "different_producer";
    case RawSnapshotRelation::kRetiredProducer:
      return "retired_producer";
    case RawSnapshotRelation::kPolicyMismatch:
      return "policy_mismatch";
    case RawSnapshotRelation::kMalformed:
      return "malformed";
  }
  return "unknown";
}

} // namespace drone_city_nav

#include "drone_city_nav/raw_obstacle_snapshot_tracker.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <unistd.h>

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

[[nodiscard]] std::uint64_t mixIdentity(std::uint64_t value) noexcept {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
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
  if (trajectory.producer_instance_id == 0U || trajectory.revision == 0U ||
      trajectory.policy_fingerprint == 0U) {
    return RawSnapshotRelation::kMalformed;
  }
  if (!current_valid_) {
    return RawSnapshotRelation::kNoSnapshot;
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
    case RawSnapshotRelation::kNoSnapshot:
      return "no_snapshot";
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

std::uint64_t generateRawObstacleProducerInstanceId() noexcept {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto system_ticks = static_cast<std::uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  const auto steady_ticks = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const auto process_id = static_cast<std::uint64_t>(::getpid());
  const auto thread_id = static_cast<std::uint64_t>(
      std::hash<std::thread::id>{}(std::this_thread::get_id()));
  const std::uint64_t ordinal = sequence.fetch_add(1U, std::memory_order_relaxed) + 1U;
  const std::uint64_t mixed = mixIdentity(system_ticks) ^ mixIdentity(steady_ticks) ^
                              mixIdentity(process_id << 32U) ^ mixIdentity(thread_id) ^
                              mixIdentity(ordinal);
  const std::uint64_t identity = mixIdentity(mixed);
  return identity != 0U ? identity : ordinal;
}

} // namespace drone_city_nav

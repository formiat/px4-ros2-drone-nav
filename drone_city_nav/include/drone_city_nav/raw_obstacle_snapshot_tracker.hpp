#pragma once

#include "drone_city_nav/obstacle_risk_field.hpp"

#include <cstdint>
#include <vector>

namespace drone_city_nav {

struct RawObstacleSnapshotIdentity {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  std::uint64_t policy_fingerprint{0U};
};

enum class RawSnapshotRelation {
  kExact,
  kRuntimeNewer,
  kRuntimeOlder,
  kNoSnapshot,
  kDifferentProducer,
  kRetiredProducer,
  kPolicyMismatch,
  kMalformed,
};

struct RawObstacleSnapshotMetadata {
  RawObstacleSnapshotIdentity identity{};
  ObstacleRiskPolicy policy{};
  bool grid_valid{false};
};

class RawObstacleSnapshotTracker {
public:
  [[nodiscard]] bool accept(const RawObstacleSnapshotMetadata& snapshot);
  [[nodiscard]] RawSnapshotRelation
  relation(const RawObstacleSnapshotIdentity& trajectory) const noexcept;
  [[nodiscard]] const RawObstacleSnapshotMetadata* current() const noexcept;

private:
  RawObstacleSnapshotMetadata current_{};
  bool current_valid_{false};
  std::vector<std::uint64_t> retired_producers_;
};

class PendingRawObstacleSnapshotDeadline {
public:
  void startIfIdle(std::int64_t now_ns) noexcept;
  void complete() noexcept;
  [[nodiscard]] bool expired(std::int64_t now_ns,
                             std::int64_t timeout_ns) const noexcept;
  [[nodiscard]] std::int64_t startedAtNs() const noexcept;

private:
  std::int64_t started_at_ns_{0};
};

[[nodiscard]] const char*
rawSnapshotRelationName(RawSnapshotRelation relation) noexcept;
[[nodiscard]] std::uint64_t generateRawObstacleProducerInstanceId() noexcept;

} // namespace drone_city_nav

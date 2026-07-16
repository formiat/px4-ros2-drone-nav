#pragma once

#include "drone_city_nav/msg/obstacle_memory_provenance.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

// Grid and provenance are published on separate ROS topics. Retain enough reliable
// provenance history to cover planner callbacks that are temporarily occupied by a
// trajectory rebuild; depth one can discard the exact snapshot needed for a later
// blocker audit before the planner has a chance to consume it.
inline constexpr std::size_t kMemoryProvenanceTransportDepth = 32U;

enum class MemoryProvenanceUnavailableReason {
  kNone,
  kNotApplicable,
  kNotReceived,
  kStampMismatch,
  kFrameMismatch,
  kSchemaInvalid,
  kGeometryMismatch,
  kContentMismatch,
  kCellMissing,
  kMalformed,
};

struct MemoryGridSnapshotIdentity {
  std::int64_t stamp_ns{0};
  bool stamp_valid{false};
  std::string frame_id;
  nav_msgs::msg::MapMetaData grid_info;
  std::uint64_t raw_grid_data_hash{0U};
  std::uint64_t occupied_cell_count{0U};
};

struct MemoryProvenanceSnapshot {
  MemoryGridSnapshotIdentity identity;
  std::unordered_map<std::size_t, MemoryCellProvenance> cells;
};

struct MemoryProvenanceParseResult {
  std::optional<MemoryProvenanceSnapshot> snapshot;
  MemoryProvenanceUnavailableReason reason{
      MemoryProvenanceUnavailableReason::kMalformed};
  std::string detail{"unclassified"};
};

struct MemoryProvenanceMatchResult {
  const MemoryProvenanceSnapshot* snapshot{nullptr};
  MemoryProvenanceUnavailableReason reason{
      MemoryProvenanceUnavailableReason::kNotReceived};
};

[[nodiscard]] std::uint64_t rawGridDataHash(std::span<const std::int8_t> data) noexcept;

[[nodiscard]] MemoryGridSnapshotIdentity
memoryGridSnapshotIdentity(const nav_msgs::msg::OccupancyGrid& grid);

[[nodiscard]] msg::ObstacleMemoryProvenance makeObstacleMemoryProvenanceMessage(
    const nav_msgs::msg::OccupancyGrid& grid,
    const std::unordered_map<std::size_t, MemoryCellProvenance>& provenance);

[[nodiscard]] std::size_t
serializedObstacleMemoryProvenanceSize(const msg::ObstacleMemoryProvenance& message);

[[nodiscard]] MemoryProvenanceParseResult
parseObstacleMemoryProvenanceMessage(const msg::ObstacleMemoryProvenance& message);

class MemoryProvenanceCache {
public:
  explicit MemoryProvenanceCache(std::size_t capacity = 4U);

  void insert(MemoryProvenanceSnapshot snapshot);
  void clear() noexcept;

  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] MemoryProvenanceMatchResult
  match(const nav_msgs::msg::OccupancyGrid& grid) const;
  [[nodiscard]] MemoryProvenanceMatchResult
  match(const nav_msgs::msg::OccupancyGrid& grid,
        const MemoryGridSnapshotIdentity& grid_identity) const;

private:
  std::size_t capacity_{4U};
  std::deque<MemoryProvenanceSnapshot> snapshots_;
};

struct MemoryProvenanceAuditResult {
  std::string diagnostic;
  std::optional<std::uint64_t> pending_audit_id;
  std::optional<std::uint64_t> evicted_audit_id;
};

struct MemoryProvenanceAuditEnrichment {
  std::uint64_t audit_id{0U};
  MemoryGridSnapshotIdentity identity;
  GridIndex cell{};
  std::string diagnostic;
};

class MemoryProvenanceAuditTracker {
public:
  explicit MemoryProvenanceAuditTracker(std::size_t cache_capacity = 4U,
                                        std::size_t pending_capacity = 256U);

  [[nodiscard]] std::vector<MemoryProvenanceAuditEnrichment>
  insert(MemoryProvenanceSnapshot snapshot);

  [[nodiscard]] MemoryProvenanceAuditResult
  audit(const nav_msgs::msg::OccupancyGrid& grid, std::optional<GridIndex> cell,
        MemoryProvenanceUnavailableReason unavailable_override =
            MemoryProvenanceUnavailableReason::kNone);

  void clear() noexcept;

  [[nodiscard]] std::size_t cachedSnapshotCount() const noexcept;
  [[nodiscard]] std::size_t pendingAuditCount() const noexcept;

private:
  struct PendingAudit {
    std::uint64_t audit_id{0U};
    MemoryGridSnapshotIdentity identity;
    GridIndex cell{};
    std::shared_ptr<const std::vector<std::size_t>> occupied_cells;
  };

  std::size_t pending_capacity_{256U};
  std::uint64_t next_audit_id_{1U};
  MemoryProvenanceCache cache_;
  std::deque<PendingAudit> pending_audits_;
};

[[nodiscard]] const char* memoryProvenanceUnavailableReasonName(
    MemoryProvenanceUnavailableReason reason) noexcept;

[[nodiscard]] std::string
formatMemoryProvenanceDiagnostic(const MemoryProvenanceMatchResult& match,
                                 std::optional<GridIndex> cell);

} // namespace drone_city_nav

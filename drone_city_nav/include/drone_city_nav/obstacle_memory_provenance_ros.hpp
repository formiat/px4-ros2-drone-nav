#pragma once

#include "drone_city_nav/msg/obstacle_memory_provenance.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>

namespace drone_city_nav {

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

private:
  std::size_t capacity_{4U};
  std::deque<MemoryProvenanceSnapshot> snapshots_;
};

[[nodiscard]] const char* memoryProvenanceUnavailableReasonName(
    MemoryProvenanceUnavailableReason reason) noexcept;

[[nodiscard]] std::string
formatMemoryProvenanceDiagnostic(const MemoryProvenanceMatchResult& match,
                                 std::optional<GridIndex> cell);

} // namespace drone_city_nav

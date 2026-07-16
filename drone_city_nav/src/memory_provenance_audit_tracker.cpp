#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include <algorithm>
#include <ranges>
#include <sstream>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool identitiesEqual(const MemoryGridSnapshotIdentity& lhs,
                                   const MemoryGridSnapshotIdentity& rhs) noexcept {
  return lhs.stamp_valid == rhs.stamp_valid && lhs.stamp_ns == rhs.stamp_ns &&
         lhs.frame_id == rhs.frame_id && lhs.grid_info == rhs.grid_info &&
         lhs.raw_grid_data_hash == rhs.raw_grid_data_hash &&
         lhs.occupied_cell_count == rhs.occupied_cell_count;
}

[[nodiscard]] std::vector<std::size_t>
occupiedCellIndices(const nav_msgs::msg::OccupancyGrid& grid,
                    const std::uint64_t occupied_cell_count) {
  std::vector<std::size_t> occupied;
  occupied.reserve(static_cast<std::size_t>(occupied_cell_count));
  for (std::size_t index = 0U; index < grid.data.size(); ++index) {
    if (grid.data[index] == 100) {
      occupied.push_back(index);
    }
  }
  return occupied;
}

[[nodiscard]] bool
snapshotMatchesOccupiedCells(const MemoryProvenanceSnapshot& snapshot,
                             const std::vector<std::size_t>& occupied_cells) {
  return snapshot.cells.size() == occupied_cells.size() &&
         std::ranges::all_of(occupied_cells, [&snapshot](const std::size_t index) {
           return snapshot.cells.contains(index);
         });
}

[[nodiscard]] MemoryProvenanceAuditOutcome
makeUnavailableOutcome(const std::uint64_t audit_id,
                       const MemoryGridSnapshotIdentity& identity, const GridIndex cell,
                       const MemoryProvenanceUnavailableReason reason) {
  return MemoryProvenanceAuditOutcome{
      .audit_id = audit_id,
      .identity = identity,
      .cell = cell,
      .reason = reason,
      .diagnostic = formatMemoryProvenanceDiagnostic(
          MemoryProvenanceMatchResult{nullptr, reason}, cell),
  };
}

} // namespace

MemoryProvenanceAuditTracker::MemoryProvenanceAuditTracker(
    const std::size_t cache_capacity, const std::size_t pending_capacity,
    const std::size_t retention_horizon)
    : pending_capacity_{std::max<std::size_t>(1U, pending_capacity)},
      retention_horizon_{std::max<std::size_t>(1U, retention_horizon)},
      cache_{cache_capacity} {
}

std::vector<MemoryProvenanceAuditOutcome>
MemoryProvenanceAuditTracker::insert(MemoryProvenanceSnapshot snapshot) {
  std::vector<MemoryProvenanceAuditOutcome> outcomes;
  auto pending = pending_audits_.begin();
  while (pending != pending_audits_.end()) {
    if (identitiesEqual(pending->identity, snapshot.identity)) {
      if (pending->occupied_cells != nullptr &&
          snapshotMatchesOccupiedCells(snapshot, *pending->occupied_cells)) {
        outcomes.push_back(MemoryProvenanceAuditOutcome{
            .audit_id = pending->audit_id,
            .identity = snapshot.identity,
            .cell = pending->cell,
            .reason = MemoryProvenanceUnavailableReason::kNone,
            .diagnostic = formatMemoryProvenanceDiagnostic(
                MemoryProvenanceMatchResult{&snapshot,
                                            MemoryProvenanceUnavailableReason::kNone},
                pending->cell),
        });
      } else {
        outcomes.push_back(makeUnavailableOutcome(
            pending->audit_id, pending->identity, pending->cell,
            MemoryProvenanceUnavailableReason::kContentMismatch));
      }
      pending = pending_audits_.erase(pending);
      continue;
    }

    const bool newer_identity = snapshot.identity.stamp_valid &&
                                pending->identity.stamp_valid &&
                                snapshot.identity.stamp_ns > pending->identity.stamp_ns;
    const bool distinct_newer_identity =
        newer_identity &&
        (!pending->last_newer_snapshot_identity.has_value() ||
         !identitiesEqual(pending->last_newer_snapshot_identity.value(), // NOLINT
                          snapshot.identity));
    if (distinct_newer_identity) {
      pending->last_newer_snapshot_identity = snapshot.identity;
      ++pending->newer_snapshot_count;
    }
    if (pending->newer_snapshot_count < retention_horizon_) {
      ++pending;
      continue;
    }
    outcomes.push_back(
        makeUnavailableOutcome(pending->audit_id, pending->identity, pending->cell,
                               MemoryProvenanceUnavailableReason::kHistoryExpired));
    pending = pending_audits_.erase(pending);
  }
  cache_.insert(std::move(snapshot));
  return outcomes;
}

std::vector<MemoryProvenanceAuditOutcome> MemoryProvenanceAuditTracker::terminate(
    const MemoryGridSnapshotIdentity& identity,
    const MemoryProvenanceUnavailableReason reason) {
  std::vector<MemoryProvenanceAuditOutcome> outcomes;
  auto pending = pending_audits_.begin();
  while (pending != pending_audits_.end()) {
    if (!identitiesEqual(pending->identity, identity)) {
      ++pending;
      continue;
    }
    outcomes.push_back(makeUnavailableOutcome(pending->audit_id, pending->identity,
                                              pending->cell, reason));
    pending = pending_audits_.erase(pending);
  }
  return outcomes;
}

MemoryProvenanceAuditResult MemoryProvenanceAuditTracker::audit(
    const nav_msgs::msg::OccupancyGrid& grid, const std::optional<GridIndex> cell,
    const MemoryProvenanceUnavailableReason unavailable_override) {
  MemoryProvenanceAuditResult result;
  if (!cell.has_value()) {
    result.diagnostic = "memory_provenance[status=not_applicable]";
    return result;
  }

  const MemoryGridSnapshotIdentity identity = memoryGridSnapshotIdentity(grid);
  MemoryProvenanceMatchResult match = cache_.match(grid, identity);
  if (match.snapshot != nullptr) {
    result.diagnostic = formatMemoryProvenanceDiagnostic(match, cell);
    return result;
  }
  if (match.reason == MemoryProvenanceUnavailableReason::kNotReceived &&
      unavailable_override != MemoryProvenanceUnavailableReason::kNone) {
    match.reason = unavailable_override;
  }
  if (!identity.stamp_valid) {
    result.diagnostic = formatMemoryProvenanceDiagnostic(match, cell);
    return result;
  }

  const auto existing =
      std::find_if(pending_audits_.begin(), pending_audits_.end(),
                   [&identity, &cell](const PendingAudit& pending) {
                     return identitiesEqual(pending.identity, identity) &&
                            pending.cell.x == cell->x && pending.cell.y == cell->y;
                   });
  if (existing != pending_audits_.end()) {
    result.pending_audit_id = existing->audit_id;
  } else {
    if (pending_audits_.size() >= pending_capacity_) {
      result.evicted_audit_id = pending_audits_.front().audit_id;
      pending_audits_.pop_front();
    }
    const std::uint64_t audit_id = next_audit_id_++;
    if (next_audit_id_ == 0U) {
      next_audit_id_ = 1U;
    }
    const auto same_snapshot =
        std::find_if(pending_audits_.begin(), pending_audits_.end(),
                     [&identity](const PendingAudit& pending) {
                       return identitiesEqual(pending.identity, identity);
                     });
    std::shared_ptr<const std::vector<std::size_t>> occupied_cells;
    if (same_snapshot != pending_audits_.end()) {
      occupied_cells = same_snapshot->occupied_cells;
    } else {
      occupied_cells = std::make_shared<const std::vector<std::size_t>>(
          occupiedCellIndices(grid, identity.occupied_cell_count));
    }
    pending_audits_.push_back(PendingAudit{
        .audit_id = audit_id,
        .identity = identity,
        .cell = *cell,
        .occupied_cells = std::move(occupied_cells),
        .newer_snapshot_count = 0U,
        .last_newer_snapshot_identity = std::nullopt,
    });
    result.pending_audit_id = audit_id;
  }

  std::ostringstream stream;
  stream << "memory_provenance[status=pending audit_id="
         << result.pending_audit_id.value_or(0U)
         << " reason=" << memoryProvenanceUnavailableReasonName(match.reason)
         << " snapshot_stamp_ns=" << identity.stamp_ns
         << " grid_hash=" << identity.raw_grid_data_hash
         << " occupied=" << identity.occupied_cell_count << " cell=(" << cell->x << ','
         << cell->y << ")]";
  if (result.evicted_audit_id.has_value()) {
    stream << " memory_provenance[status=evicted audit_id="
           << result.evicted_audit_id.value_or(0U) << " reason="
           << memoryProvenanceUnavailableReasonName(
                  MemoryProvenanceUnavailableReason::kCapacityEvicted)
           << ']';
  }
  result.diagnostic = stream.str();
  return result;
}

void MemoryProvenanceAuditTracker::clear() noexcept {
  cache_.clear();
  pending_audits_.clear();
  next_audit_id_ = 1U;
}

std::size_t MemoryProvenanceAuditTracker::cachedSnapshotCount() const noexcept {
  return cache_.size();
}

std::size_t MemoryProvenanceAuditTracker::pendingAuditCount() const noexcept {
  return pending_audits_.size();
}

} // namespace drone_city_nav

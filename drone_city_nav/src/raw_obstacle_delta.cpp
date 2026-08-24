#include "drone_city_nav/raw_obstacle_delta.hpp"

#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace drone_city_nav {
namespace {

constexpr std::uint32_t kMaximumChunkSizeCells{256U};
constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::uint64_t kRawObstacle2DSnapshotWireDomain{0x32534e4150574952ULL};
constexpr std::uint64_t kRawObstacle2DDeltaWireDomain{0x3244454c54415749ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

[[nodiscard]] std::uint64_t canonicalFloatBits(const float value) noexcept {
  return value == 0.0F ? 0U : std::bit_cast<std::uint32_t>(value);
}

void hashString(std::uint64_t& hash, const std::string& value) noexcept {
  hashValue(hash, value.size());
  for (const char character : value) {
    hashValue(hash, static_cast<unsigned char>(character));
  }
}

void hashTime(std::uint64_t& hash,
              const builtin_interfaces::msg::Time& stamp) noexcept {
  hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(stamp.sec)));
  hashValue(hash, stamp.nanosec);
}

void hashHeader(std::uint64_t& hash, const std_msgs::msg::Header& header) noexcept {
  hashTime(hash, header.stamp);
  hashString(hash, header.frame_id);
}

void hashMapInfo(std::uint64_t& hash, const nav_msgs::msg::MapMetaData& info) noexcept {
  hashTime(hash, info.map_load_time);
  hashValue(hash, canonicalFloatBits(info.resolution));
  hashValue(hash, info.width);
  hashValue(hash, info.height);
  hashValue(hash, canonicalDoubleBits(info.origin.position.x));
  hashValue(hash, canonicalDoubleBits(info.origin.position.y));
  hashValue(hash, canonicalDoubleBits(info.origin.position.z));
  hashValue(hash, canonicalDoubleBits(info.origin.orientation.x));
  hashValue(hash, canonicalDoubleBits(info.origin.orientation.y));
  hashValue(hash, canonicalDoubleBits(info.origin.orientation.z));
  hashValue(hash, canonicalDoubleBits(info.origin.orientation.w));
}

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] bool validRiskPolicy(const std::uint64_t fingerprint,
                                   const double critical_distance_m,
                                   const double preferred_distance_m) noexcept {
  return fingerprint != 0U && std::isfinite(critical_distance_m) &&
         std::isfinite(preferred_distance_m) && critical_distance_m >= 0.0 &&
         preferred_distance_m >= critical_distance_m;
}

[[nodiscard]] RawObstacleGridUpdateStatus
updateStatus(const ProducerEvidenceAdmissionStatus status) noexcept {
  switch (status) {
    case ProducerEvidenceAdmissionStatus::kAcceptedInitial:
    case ProducerEvidenceAdmissionStatus::kAcceptedNewer:
    case ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff:
      return RawObstacleGridUpdateStatus::kAccepted;
    case ProducerEvidenceAdmissionStatus::kIdempotentReplay:
      return RawObstacleGridUpdateStatus::kIdempotentReplay;
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict:
      return RawObstacleGridUpdateStatus::kIdentityConflict;
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity:
      return RawObstacleGridUpdateStatus::kProducerEpochCapacity;
    case ProducerEvidenceAdmissionStatus::kRejectedRegression:
      return RawObstacleGridUpdateStatus::kStale;
    case ProducerEvidenceAdmissionStatus::kRejectedAuthorityMismatch:
    case ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired:
    case ProducerEvidenceAdmissionStatus::kRejectedClockDiscontinuity:
      return RawObstacleGridUpdateStatus::kProducerEpochUnavailable;
    case ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate:
      return RawObstacleGridUpdateStatus::kStale;
    case ProducerEvidenceAdmissionStatus::kRejectedInvalid:
      return RawObstacleGridUpdateStatus::kInvalidMessage;
  }
  return RawObstacleGridUpdateStatus::kInvalidMessage;
}

[[nodiscard]] bool validChunkSize(const std::uint32_t chunk_size_cells) noexcept {
  return chunk_size_cells > 0U && chunk_size_cells <= kMaximumChunkSizeCells;
}

[[nodiscard]] bool axisAlignedMapInfo(const nav_msgs::msg::MapMetaData& info) noexcept {
  constexpr double kPoseTolerance{1.0e-9};
  const auto& position = info.origin.position;
  const auto& orientation = info.origin.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) && std::abs(position.z) <= kPoseTolerance &&
         std::abs(orientation.x) <= kPoseTolerance &&
         std::abs(orientation.y) <= kPoseTolerance &&
         std::abs(orientation.z) <= kPoseTolerance &&
         std::abs(std::abs(orientation.w) - 1.0) <= kPoseTolerance;
}

[[nodiscard]] std::size_t divideRoundUp(const std::size_t value,
                                        const std::size_t divisor) noexcept {
  return (value + divisor - 1U) / divisor;
}

[[nodiscard]] bool sameGeometry(const GridBounds& bounds,
                                const nav_msgs::msg::MapMetaData& info) noexcept {
  return axisAlignedMapInfo(info) &&
         info.width == static_cast<std::uint32_t>(bounds.width_cells) &&
         info.height == static_cast<std::uint32_t>(bounds.height_cells) &&
         std::abs(static_cast<double>(info.resolution) - bounds.resolution_m) <=
             1.0e-9 &&
         std::abs(info.origin.position.x - bounds.origin_x) <= 1.0e-9 &&
         std::abs(info.origin.position.y - bounds.origin_y) <= 1.0e-9;
}

[[nodiscard]] CellState decodedCellState(const std::int8_t value) noexcept {
  if (value >= static_cast<std::int8_t>(CellState::kOccupied)) {
    return CellState::kOccupied;
  }
  if (value == static_cast<std::int8_t>(CellState::kFree)) {
    return CellState::kFree;
  }
  return CellState::kUnknown;
}

void setCellState(OccupancyGrid2D& grid, const GridIndex cell, const CellState state) {
  switch (state) {
    case CellState::kUnknown:
      grid.setUnknown(cell);
      return;
    case CellState::kFree:
      grid.setFree(cell);
      return;
    case CellState::kOccupied:
      grid.setOccupied(cell);
      return;
  }
}

[[nodiscard]] nav_msgs::msg::MapMetaData mapInfo(const GridBounds& bounds) {
  nav_msgs::msg::MapMetaData info;
  info.resolution = static_cast<float>(bounds.resolution_m);
  info.width = static_cast<std::uint32_t>(bounds.width_cells);
  info.height = static_cast<std::uint32_t>(bounds.height_cells);
  info.origin.position.x = bounds.origin_x;
  info.origin.position.y = bounds.origin_y;
  info.origin.orientation.w = 1.0;
  return info;
}

} // namespace

std::uint64_t
rawObstacleSnapshotWireFingerprint(const msg::RawObstacleSnapshot& snapshot) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kRawObstacle2DSnapshotWireDomain);
  hashValue(hash, snapshot.producer_instance_id);
  hashValue(hash, snapshot.obstacle_snapshot_revision);
  hashValue(hash, snapshot.risk_policy_fingerprint);
  hashValue(hash, canonicalDoubleBits(snapshot.risk_critical_distance_m));
  hashValue(hash, canonicalDoubleBits(snapshot.risk_preferred_distance_m));
  hashHeader(hash, snapshot.grid.header);
  hashMapInfo(hash, snapshot.grid.info);
  hashValue(hash, snapshot.grid.data.size());
  for (const std::int8_t value : snapshot.grid.data) {
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
  }
  return hash == 0U ? 1U : hash;
}

std::uint64_t
rawObstacleDeltaWireFingerprint(const msg::RawObstacleDelta& delta) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kRawObstacle2DDeltaWireDomain);
  hashHeader(hash, delta.header);
  hashValue(hash, delta.producer_instance_id);
  hashValue(hash, delta.base_snapshot_revision);
  hashValue(hash, delta.obstacle_snapshot_revision);
  hashValue(hash, delta.risk_policy_fingerprint);
  hashValue(hash, canonicalDoubleBits(delta.risk_critical_distance_m));
  hashValue(hash, canonicalDoubleBits(delta.risk_preferred_distance_m));
  hashMapInfo(hash, delta.map_info);
  hashValue(hash, delta.chunk_size_cells);
  hashValue(hash, delta.chunk_indices.size());
  for (const std::uint32_t index : delta.chunk_indices) {
    hashValue(hash, index);
  }
  hashValue(hash, delta.chunk_data.size());
  for (const std::int8_t value : delta.chunk_data) {
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
  }
  return hash == 0U ? 1U : hash;
}

std::vector<std::uint32_t>
rawObstacleChunkIndices(const GridBounds& bounds,
                        const std::span<const std::size_t> changed_cell_indices,
                        const std::uint32_t chunk_size_cells) {
  if (!validChunkSize(chunk_size_cells) || bounds.width_cells <= 0 ||
      bounds.height_cells <= 0) {
    return {};
  }
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t cell_count = width * static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunk_size = static_cast<std::size_t>(chunk_size_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  std::vector<std::uint32_t> result;
  result.reserve(changed_cell_indices.size());
  for (const std::size_t cell_index : changed_cell_indices) {
    if (cell_index >= cell_count) {
      continue;
    }
    const std::size_t x = cell_index % width;
    const std::size_t y = cell_index / width;
    const std::size_t chunk_index = (y / chunk_size) * chunks_x + x / chunk_size;
    if (chunk_index <= std::numeric_limits<std::uint32_t>::max()) {
      result.push_back(static_cast<std::uint32_t>(chunk_index));
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

std::optional<msg::RawObstacleDelta> makeRawObstacleDelta(
    const OccupancyGrid2D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id,
    const std::uint64_t base_snapshot_revision,
    const std::uint64_t obstacle_snapshot_revision,
    const std::uint64_t risk_policy_fingerprint, const double risk_critical_distance_m,
    const double risk_preferred_distance_m, const std::uint32_t chunk_size_cells,
    const std::span<const std::uint32_t> chunk_indices) {
  if (!validChunkSize(chunk_size_cells) || producer_instance_id == 0U ||
      base_snapshot_revision == 0U ||
      obstacle_snapshot_revision <= base_snapshot_revision) {
    return std::nullopt;
  }
  const GridBounds& bounds = grid.bounds();
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunk_size = static_cast<std::size_t>(chunk_size_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  const std::size_t chunks_y = divideRoundUp(height, chunk_size);
  const std::size_t chunk_count = chunks_x * chunks_y;
  const std::size_t cells_per_chunk = chunk_size * chunk_size;
  if (chunk_indices.size() >
      std::numeric_limits<std::size_t>::max() / cells_per_chunk) {
    return std::nullopt;
  }

  msg::RawObstacleDelta message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.base_snapshot_revision = base_snapshot_revision;
  message.obstacle_snapshot_revision = obstacle_snapshot_revision;
  message.risk_policy_fingerprint = risk_policy_fingerprint;
  message.risk_critical_distance_m = risk_critical_distance_m;
  message.risk_preferred_distance_m = risk_preferred_distance_m;
  message.map_info = mapInfo(bounds);
  message.chunk_size_cells = chunk_size_cells;
  message.chunk_indices.assign(chunk_indices.begin(), chunk_indices.end());
  message.chunk_data.reserve(chunk_indices.size() * cells_per_chunk);
  std::unordered_set<std::uint32_t> unique_chunks;
  for (const std::uint32_t chunk_index : chunk_indices) {
    if (static_cast<std::size_t>(chunk_index) >= chunk_count ||
        !unique_chunks.insert(chunk_index).second) {
      return std::nullopt;
    }
    const std::size_t chunk_x = static_cast<std::size_t>(chunk_index) % chunks_x;
    const std::size_t chunk_y = static_cast<std::size_t>(chunk_index) / chunks_x;
    for (std::size_t local_y = 0U; local_y < chunk_size; ++local_y) {
      for (std::size_t local_x = 0U; local_x < chunk_size; ++local_x) {
        const std::size_t x = chunk_x * chunk_size + local_x;
        const std::size_t y = chunk_y * chunk_size + local_y;
        const CellState state =
            x < width && y < height
                ? grid.state(GridIndex{static_cast<int>(x), static_cast<int>(y)})
                : CellState::kUnknown;
        message.chunk_data.push_back(static_cast<std::int8_t>(state));
      }
    }
  }
  return message;
}

RawObstacleGridUpdate RawObstacleDeltaAccumulator::apply(
    const msg::RawObstacleSnapshot& snapshot,
    const ProducerEpochAdmissionState& producer_epoch,
    const std::int64_t receive_stamp_ns, const std::int64_t now_ns,
    const ProducerEpochAdmissionConfig& config, const bool external_contract_valid) {
  const std::int64_t source_stamp_ns = timeNanoseconds(snapshot.grid.header.stamp);
  const ProducerEpochObservation observation{
      .producer_instance_id = snapshot.producer_instance_id,
      .sequence = snapshot.obstacle_snapshot_revision,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = rawObstacleSnapshotWireFingerprint(snapshot),
  };
  const bool claimable =
      snapshot.producer_instance_id != 0U && snapshot.obstacle_snapshot_revision != 0U;
  if (!claimable) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }
  for (std::size_t index = 0U; index < prospective_identity_count_; ++index) {
    ProspectiveIdentity& pending = prospective_identities_[index];
    if (pending.observation.producer_instance_id != observation.producer_instance_id ||
        pending.observation.sequence != observation.sequence) {
      continue;
    }
    const bool exact =
        pending.full_snapshot &&
        observation.source_stamp_ns == pending.observation.source_stamp_ns &&
        observation.content_fingerprint == pending.observation.content_fingerprint;
    if (!exact || pending.identity_conflicted) {
      pending.identity_conflicted = true;
      return {.state = state_,
              .status = RawObstacleGridUpdateStatus::kIdentityConflict};
    }
    return {.state = state_, .status = pending.terminal_status};
  }
  const bool contract_valid =
      external_contract_valid && axisAlignedMapInfo(snapshot.grid.info) &&
      validRiskPolicy(snapshot.risk_policy_fingerprint,
                      snapshot.risk_critical_distance_m,
                      snapshot.risk_preferred_distance_m) &&
      rawOccupancyGridViewFromRos(snapshot.grid, static_cast<std::int8_t>(100))
          .has_value();
  const auto reconstruct = [&snapshot]() -> RawObstacleGridState {
    RawOccupancyGridFromRosResult converted =
        rawOccupancyGridFromRos(snapshot.grid, RawOccupancyGridFromRosConfig{100, 0});
    if (!converted.grid.has_value()) {
      return {};
    }
    return RawObstacleGridState{
        .producer_instance_id = snapshot.producer_instance_id,
        .base_snapshot_revision = snapshot.obstacle_snapshot_revision,
        .obstacle_snapshot_revision = snapshot.obstacle_snapshot_revision,
        .risk_policy_fingerprint = snapshot.risk_policy_fingerprint,
        .risk_critical_distance_m = snapshot.risk_critical_distance_m,
        .risk_preferred_distance_m = snapshot.risk_preferred_distance_m,
        .occupancy =
            std::make_shared<const OccupancyGrid2D>(std::move(*converted.grid)),
    };
  };
  const ProducerEpochAuthority authority = producerEpochAuthority(producer_epoch);
  if (authority.valid() &&
      authority.producer_instance_id == observation.producer_instance_id) {
    if (prospective_identity_capacity_exhausted_ &&
        evidence_admission_.authority_generation != authority.generation) {
      return {.state = state_,
              .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
    }
    const ProducerEvidenceAdmissionResult admission =
        admitProducerEvidence(config, evidence_admission_, authority, observation, true,
                              now_ns, contract_valid);
    evidence_admission_ = admission.next_state;
    if (admission.install_evidence) {
      RawObstacleGridState candidate = reconstruct();
      if (!candidate.occupancy) {
        return {.state = state_,
                .status = RawObstacleGridUpdateStatus::kInvalidMessage,
                .evidence_status = ProducerEvidenceAdmissionStatus::kRejectedInvalid,
                .evidence_observation = observation,
                .authority_generation = authority.generation};
      }
      state_ = std::move(candidate);
      pruneProspectiveIdentities(observation.producer_instance_id,
                                 observation.sequence);
    }
    return {
        .state = state_,
        .status = updateStatus(admission.status),
        .evidence_status = admission.status,
        .evidence_observation = observation,
        .authority_generation = authority.generation,
        .current_identity_conflict = admission.current_identity_conflict,
        .authority_handoff = admission.authority_handoff,
    };
  }
  if (producerEpochRetired(producer_epoch, observation.producer_instance_id)) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kStale};
  }
  if (prospective_identity_capacity_exhausted_) {
    return {.state = state_,
            .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
  }
  if (prospective_identity_count_ == prospective_identities_.size()) {
    prospective_identity_capacity_exhausted_ = true;
    return {.state = state_,
            .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
  }
  const bool sequence_regression =
      observation.sequence <
      prospectiveSequenceHighWater(observation.producer_instance_id);
  const RawObstacleGridUpdateStatus terminal_status =
      sequence_regression ? RawObstacleGridUpdateStatus::kStale
      : !contract_valid   ? RawObstacleGridUpdateStatus::kInvalidMessage
      : !producerEpochObservationFresh(config, observation, now_ns)
          ? RawObstacleGridUpdateStatus::kStale
          : RawObstacleGridUpdateStatus::kProducerEpochPending;
  prospective_identities_[prospective_identity_count_] =
      ProspectiveIdentity{.observation = observation,
                          .state = {},
                          .terminal_status = terminal_status,
                          .full_snapshot = true,
                          .contract_valid = contract_valid && !sequence_regression,
                          .identity_conflicted = false};
  if (terminal_status == RawObstacleGridUpdateStatus::kProducerEpochPending) {
    prospective_identities_[prospective_identity_count_].state = reconstruct();
    if (!prospective_identities_[prospective_identity_count_].state.occupancy) {
      prospective_identities_[prospective_identity_count_].terminal_status =
          RawObstacleGridUpdateStatus::kInvalidMessage;
      prospective_identities_[prospective_identity_count_].contract_valid = false;
    }
  }
  ++prospective_identity_count_;
  return {
      .state = state_,
      .status =
          prospective_identities_[prospective_identity_count_ - 1U].terminal_status};
}

RawObstacleGridUpdate RawObstacleDeltaAccumulator::apply(
    const msg::RawObstacleDelta& delta,
    const ProducerEpochAdmissionState& producer_epoch,
    const std::int64_t receive_stamp_ns, const std::int64_t now_ns,
    const ProducerEpochAdmissionConfig& config, const bool external_contract_valid) {
  const std::int64_t source_stamp_ns = timeNanoseconds(delta.header.stamp);
  const ProducerEpochObservation observation{
      .producer_instance_id = delta.producer_instance_id,
      .sequence = delta.obstacle_snapshot_revision,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = rawObstacleDeltaWireFingerprint(delta),
  };
  if (delta.producer_instance_id == 0U || delta.obstacle_snapshot_revision == 0U) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kInvalidMessage};
  }
  for (std::size_t index = 0U; index < prospective_identity_count_; ++index) {
    ProspectiveIdentity& pending = prospective_identities_[index];
    if (pending.observation.producer_instance_id != observation.producer_instance_id ||
        pending.observation.sequence != observation.sequence) {
      continue;
    }
    const bool exact =
        !pending.full_snapshot &&
        observation.source_stamp_ns == pending.observation.source_stamp_ns &&
        observation.content_fingerprint == pending.observation.content_fingerprint;
    if (!exact || pending.identity_conflicted) {
      pending.identity_conflicted = true;
      return {.state = state_,
              .status = RawObstacleGridUpdateStatus::kIdentityConflict};
    }
    return {.state = state_, .status = pending.terminal_status};
  }
  const ProducerEpochAuthority authority = producerEpochAuthority(producer_epoch);
  if (!authority.valid() ||
      delta.producer_instance_id != authority.producer_instance_id) {
    if (producerEpochRetired(producer_epoch, observation.producer_instance_id)) {
      return {.state = state_, .status = RawObstacleGridUpdateStatus::kStale};
    }
    if (prospective_identity_capacity_exhausted_) {
      return {.state = state_,
              .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
    }
    if (prospective_identity_count_ == prospective_identities_.size()) {
      prospective_identity_capacity_exhausted_ = true;
      return {.state = state_,
              .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
    }
    const bool sequence_regression =
        observation.sequence <
        prospectiveSequenceHighWater(observation.producer_instance_id);
    const RawObstacleGridUpdateStatus terminal_status =
        sequence_regression ? RawObstacleGridUpdateStatus::kStale
        : !external_contract_valid || source_stamp_ns <= 0 || receive_stamp_ns <= 0
            ? RawObstacleGridUpdateStatus::kInvalidMessage
        : !producerEpochObservationFresh(config, observation, now_ns)
            ? RawObstacleGridUpdateStatus::kStale
        : authority.valid() ? RawObstacleGridUpdateStatus::kBaseUnavailable
                            : RawObstacleGridUpdateStatus::kProducerEpochUnavailable;
    prospective_identities_[prospective_identity_count_] = ProspectiveIdentity{
        .observation = observation,
        .state = {},
        .terminal_status = terminal_status,
        .full_snapshot = false,
        .contract_valid = false,
        .identity_conflicted = false,
    };
    ++prospective_identity_count_;
    return {.state = state_, .status = terminal_status};
  }
  if (!state_.occupancy ||
      evidence_admission_.authority_generation != authority.generation ||
      delta.producer_instance_id != state_.producer_instance_id ||
      delta.base_snapshot_revision != state_.base_snapshot_revision) {
    const ProducerEvidenceAdmissionResult admission = admitProducerEvidence(
        config, evidence_admission_, authority, observation, false, now_ns, false);
    evidence_admission_ = admission.next_state;
    const RawObstacleGridUpdateStatus status =
        admission.status == ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict
            ? RawObstacleGridUpdateStatus::kIdentityConflict
        : admission.status == ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity
            ? RawObstacleGridUpdateStatus::kProducerEpochCapacity
            : RawObstacleGridUpdateStatus::kBaseUnavailable;
    return {
        .state = state_,
        .status = status,
        .evidence_status = admission.status,
        .evidence_observation = observation,
        .authority_generation = authority.generation,
        .current_identity_conflict = admission.current_identity_conflict,
        .authority_handoff = admission.authority_handoff,
    };
  }
  const auto admit_candidate = [this, &config, &authority, &observation, now_ns](
                                   const bool contract_valid) -> RawObstacleGridUpdate {
    const ProducerEvidenceAdmissionResult admission =
        admitProducerEvidence(config, evidence_admission_, authority, observation,
                              false, now_ns, contract_valid);
    evidence_admission_ = admission.next_state;
    return {
        .state = state_,
        .status = updateStatus(admission.status),
        .evidence_status = admission.status,
        .evidence_observation = observation,
        .authority_generation = authority.generation,
        .current_identity_conflict = admission.current_identity_conflict,
        .authority_handoff = admission.authority_handoff,
    };
  };
  if (!external_contract_valid || source_stamp_ns <= 0 || receive_stamp_ns <= 0) {
    return admit_candidate(false);
  }
  if (delta.obstacle_snapshot_revision < state_.obstacle_snapshot_revision) {
    return admit_candidate(false);
  }
  const GridBounds& bounds = state_.occupancy->bounds();
  if (!validChunkSize(delta.chunk_size_cells) ||
      !sameGeometry(bounds, delta.map_info) ||
      delta.risk_policy_fingerprint != state_.risk_policy_fingerprint ||
      delta.risk_critical_distance_m != state_.risk_critical_distance_m ||
      delta.risk_preferred_distance_m != state_.risk_preferred_distance_m) {
    return admit_candidate(false);
  }
  const std::size_t chunk_size = static_cast<std::size_t>(delta.chunk_size_cells);
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t chunks_x = divideRoundUp(width, chunk_size);
  const std::size_t chunks_y = divideRoundUp(height, chunk_size);
  const std::size_t chunk_count = chunks_x * chunks_y;
  const std::size_t cells_per_chunk = chunk_size * chunk_size;
  if (delta.chunk_indices.empty() ||
      delta.chunk_indices.size() >
          std::numeric_limits<std::size_t>::max() / cells_per_chunk ||
      delta.chunk_data.size() != delta.chunk_indices.size() * cells_per_chunk) {
    return admit_candidate(false);
  }
  std::unordered_set<std::uint32_t> unique_chunks;
  for (const std::uint32_t chunk_index : delta.chunk_indices) {
    if (static_cast<std::size_t>(chunk_index) >= chunk_count ||
        !unique_chunks.insert(chunk_index).second) {
      return admit_candidate(false);
    }
  }
  RawObstacleGridUpdate admitted = admit_candidate(true);
  if (!admitted.accepted()) {
    return admitted;
  }

  OccupancyGrid2D updated = *state_.occupancy;
  for (std::size_t message_chunk = 0U; message_chunk < delta.chunk_indices.size();
       ++message_chunk) {
    const std::uint32_t chunk_index = delta.chunk_indices[message_chunk];
    const std::size_t chunk_x = static_cast<std::size_t>(chunk_index) % chunks_x;
    const std::size_t chunk_y = static_cast<std::size_t>(chunk_index) / chunks_x;
    const std::size_t data_offset = message_chunk * cells_per_chunk;
    for (std::size_t local_y = 0U; local_y < chunk_size; ++local_y) {
      for (std::size_t local_x = 0U; local_x < chunk_size; ++local_x) {
        const std::size_t x = chunk_x * chunk_size + local_x;
        const std::size_t y = chunk_y * chunk_size + local_y;
        if (x >= width || y >= height) {
          continue;
        }
        const std::size_t data_index = data_offset + local_y * chunk_size + local_x;
        setCellState(updated, GridIndex{static_cast<int>(x), static_cast<int>(y)},
                     decodedCellState(delta.chunk_data[data_index]));
      }
    }
  }
  RawObstacleGridState candidate = state_;
  candidate.obstacle_snapshot_revision = delta.obstacle_snapshot_revision;
  candidate.occupancy = std::make_shared<const OccupancyGrid2D>(std::move(updated));
  state_ = std::move(candidate);
  pruneProspectiveIdentities(observation.producer_instance_id, observation.sequence);
  admitted.state = state_;
  return admitted;
}

void RawObstacleDeltaAccumulator::pruneProspectiveIdentities(
    const std::uint64_t producer_instance_id, const std::uint64_t sequence) noexcept {
  std::size_t write_index{0U};
  for (std::size_t read_index = 0U; read_index < prospective_identity_count_;
       ++read_index) {
    const ProspectiveIdentity& pending = prospective_identities_[read_index];
    const bool remove =
        pending.observation.producer_instance_id == producer_instance_id &&
        pending.observation.sequence <= sequence;
    if (!remove) {
      if (write_index != read_index) {
        prospective_identities_[write_index] = pending;
      }
      ++write_index;
    }
  }
  for (std::size_t index = write_index; index < prospective_identity_count_; ++index) {
    prospective_identities_[index] = ProspectiveIdentity{};
  }
  prospective_identity_count_ = write_index;
}

std::uint64_t RawObstacleDeltaAccumulator::prospectiveSequenceHighWater(
    const std::uint64_t producer_instance_id) const noexcept {
  std::uint64_t high_water{0U};
  for (std::size_t index = 0U; index < prospective_identity_count_; ++index) {
    if (prospective_identities_[index].observation.producer_instance_id ==
        producer_instance_id) {
      high_water =
          std::max(high_water, prospective_identities_[index].observation.sequence);
    }
  }
  return high_water;
}

RawObstacleGridUpdate RawObstacleDeltaAccumulator::synchronizeProducerEpoch(
    const ProducerEpochAdmissionState& producer_epoch, const std::int64_t now_ns,
    const ProducerEpochAdmissionConfig& config) {
  std::size_t write_index{0U};
  for (std::size_t read_index = 0U; read_index < prospective_identity_count_;
       ++read_index) {
    ProspectiveIdentity& pending = prospective_identities_[read_index];
    const bool keep =
        !producerEpochRetired(producer_epoch, pending.observation.producer_instance_id);
    if (keep) {
      if (write_index != read_index) {
        prospective_identities_[write_index] = std::move(pending);
      }
      ++write_index;
    }
  }
  for (std::size_t index = write_index; index < prospective_identity_count_; ++index) {
    prospective_identities_[index] = ProspectiveIdentity{};
  }
  prospective_identity_count_ = write_index;

  const ProducerEpochAuthority authority = producerEpochAuthority(producer_epoch);
  if (!authority.valid()) {
    return {.state = state_,
            .status = RawObstacleGridUpdateStatus::kProducerEpochUnavailable};
  }
  if (prospective_identity_capacity_exhausted_ &&
      evidence_admission_.authority_generation != authority.generation) {
    return {.state = state_,
            .status = RawObstacleGridUpdateStatus::kProducerEpochCapacity};
  }
  std::optional<std::size_t> selected;
  for (std::size_t index = 0U; index < prospective_identity_count_; ++index) {
    const ProspectiveIdentity& pending = prospective_identities_[index];
    if (pending.full_snapshot &&
        pending.observation.producer_instance_id == authority.producer_instance_id &&
        (!selected.has_value() ||
         pending.observation.sequence >
             prospective_identities_[*selected].observation.sequence)) {
      selected = index;
    }
  }
  if (!selected.has_value()) {
    if (evidence_admission_.authority_generation == authority.generation &&
        evidence_admission_.producer_instance_id == authority.producer_instance_id) {
      return {.state = state_,
              .status = evidence_admission_.current_identity_conflicted
                            ? RawObstacleGridUpdateStatus::kIdentityConflict
                            : RawObstacleGridUpdateStatus::kIdempotentReplay};
    }
    return {.state = state_,
            .status = RawObstacleGridUpdateStatus::kProducerEpochUnavailable};
  }
  const ProspectiveIdentity selected_pending = prospective_identities_[*selected];
  if (selected_pending.identity_conflicted) {
    return {.state = state_, .status = RawObstacleGridUpdateStatus::kIdentityConflict};
  }
  const ProducerEvidenceAdmissionResult admission = admitProducerEvidence(
      config, evidence_admission_, authority, selected_pending.observation, true,
      now_ns, selected_pending.contract_valid);
  evidence_admission_ = admission.next_state;
  if (admission.install_evidence) {
    state_ = selected_pending.state;
  }
  write_index = 0U;
  for (std::size_t read_index = 0U; read_index < prospective_identity_count_;
       ++read_index) {
    const ProspectiveIdentity& pending = prospective_identities_[read_index];
    const bool remove =
        read_index == *selected ||
        (admission.install_evidence &&
         pending.observation.producer_instance_id ==
             selected_pending.observation.producer_instance_id &&
         pending.observation.sequence <= selected_pending.observation.sequence);
    if (!remove) {
      if (write_index != read_index) {
        prospective_identities_[write_index] = pending;
      }
      ++write_index;
    }
  }
  for (std::size_t index = write_index; index < prospective_identity_count_; ++index) {
    prospective_identities_[index] = ProspectiveIdentity{};
  }
  prospective_identity_count_ = write_index;
  return {
      .state = state_,
      .status = updateStatus(admission.status),
      .evidence_status = admission.status,
      .evidence_observation = selected_pending.observation,
      .authority_generation = authority.generation,
      .current_identity_conflict = admission.current_identity_conflict,
      .authority_handoff = admission.authority_handoff,
  };
}

const RawObstacleGridState& RawObstacleDeltaAccumulator::state() const noexcept {
  return state_;
}

const ProducerEvidenceAdmissionState&
RawObstacleDeltaAccumulator::evidenceAdmissionState() const noexcept {
  return evidence_admission_;
}

const char*
rawObstacleGridUpdateStatusName(const RawObstacleGridUpdateStatus status) noexcept {
  switch (status) {
    case RawObstacleGridUpdateStatus::kAccepted:
      return "accepted";
    case RawObstacleGridUpdateStatus::kIdempotentReplay:
      return "idempotent_replay";
    case RawObstacleGridUpdateStatus::kProducerEpochPending:
      return "producer_epoch_pending";
    case RawObstacleGridUpdateStatus::kProducerEpochUnavailable:
      return "producer_epoch_unavailable";
    case RawObstacleGridUpdateStatus::kProducerEpochCapacity:
      return "producer_epoch_capacity";
    case RawObstacleGridUpdateStatus::kIdentityConflict:
      return "identity_conflict";
    case RawObstacleGridUpdateStatus::kStale:
      return "stale";
    case RawObstacleGridUpdateStatus::kBaseUnavailable:
      return "base_unavailable";
    case RawObstacleGridUpdateStatus::kInvalidMessage:
      return "invalid_message";
  }
  return "unknown";
}

} // namespace drone_city_nav

#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include "drone_city_nav/msg/observed_obstacle_chunk3_d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_set>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::uint64_t kRawObstacle3DSnapshotWireDomain{0x33534e4150574952ULL};
constexpr std::uint64_t kRawObstacle3DDeltaWireDomain{0x3344454c54415749ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

void hashString(std::uint64_t& hash, const std::string& value) noexcept {
  hashValue(hash, value.size());
  for (const char character : value) {
    hashValue(hash, static_cast<unsigned char>(character));
  }
}

void hashHeader(std::uint64_t& hash, const std_msgs::msg::Header& header) noexcept {
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(header.stamp.sec)));
  hashValue(hash, header.stamp.nanosec);
  hashString(hash, header.frame_id);
}

template<typename Message>
void hashWireGeometry(std::uint64_t& hash, const Message& message) noexcept {
  hashValue(hash, canonicalDoubleBits(message.origin_x_m));
  hashValue(hash, canonicalDoubleBits(message.origin_y_m));
  hashValue(hash, canonicalDoubleBits(message.origin_z_m));
  hashValue(hash, canonicalDoubleBits(message.resolution_m));
  hashValue(hash, message.width_cells);
  hashValue(hash, message.height_cells);
  hashValue(hash, message.depth_cells);
  hashValue(hash, message.chunk_size_cells);
}

template<typename Message>
void hashWireChunks(std::uint64_t& hash, const Message& message) noexcept {
  hashValue(hash, message.chunks.size());
  for (const msg::ObservedObstacleChunk3D& chunk : message.chunks) {
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(chunk.x)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(chunk.y)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(chunk.z)));
    for (const std::uint64_t word : chunk.observed_words) {
      hashValue(hash, word);
    }
    for (const std::uint64_t word : chunk.occupied_words) {
      hashValue(hash, word);
    }
  }
}

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] RawObstacleGridUpdateStatus3D
updateStatus(const ProducerEvidenceAdmissionStatus status) noexcept {
  switch (status) {
    case ProducerEvidenceAdmissionStatus::kAcceptedInitial:
    case ProducerEvidenceAdmissionStatus::kAcceptedNewer:
    case ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff:
      return RawObstacleGridUpdateStatus3D::kAccepted;
    case ProducerEvidenceAdmissionStatus::kIdempotentReplay:
      return RawObstacleGridUpdateStatus3D::kIdempotentReplay;
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict:
      return RawObstacleGridUpdateStatus3D::kIdentityConflict;
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity:
      return RawObstacleGridUpdateStatus3D::kProducerEpochCapacity;
    case ProducerEvidenceAdmissionStatus::kRejectedRegression:
    case ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate:
      return RawObstacleGridUpdateStatus3D::kStale;
    case ProducerEvidenceAdmissionStatus::kRejectedAuthorityMismatch:
    case ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired:
    case ProducerEvidenceAdmissionStatus::kRejectedClockDiscontinuity:
      return RawObstacleGridUpdateStatus3D::kProducerEpochUnavailable;
    case ProducerEvidenceAdmissionStatus::kRejectedInvalid:
      return RawObstacleGridUpdateStatus3D::kInvalidMessage;
  }
  return RawObstacleGridUpdateStatus3D::kInvalidMessage;
}

[[nodiscard]] bool validBounds(const GridBounds3D& bounds) noexcept {
  return std::isfinite(bounds.origin_x) && std::isfinite(bounds.origin_y) &&
         std::isfinite(bounds.origin_z) && std::isfinite(bounds.resolution_m) &&
         bounds.resolution_m > 0.0 && bounds.width_cells > 0 &&
         bounds.height_cells > 0 && bounds.depth_cells > 0;
}

template<typename Message>
[[nodiscard]] std::optional<GridBounds3D> boundsFromMessage(const Message& message) {
  if (message.chunk_size_cells !=
          static_cast<std::uint32_t>(ObservedOccupancyGrid3D::kChunkSize) ||
      message.width_cells == 0U || message.height_cells == 0U ||
      message.depth_cells == 0U ||
      message.width_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      message.height_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      message.depth_cells >
          static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }
  const GridBounds3D bounds{
      .origin_x = message.origin_x_m,
      .origin_y = message.origin_y_m,
      .origin_z = message.origin_z_m,
      .resolution_m = message.resolution_m,
      .width_cells = static_cast<int>(message.width_cells),
      .height_cells = static_cast<int>(message.height_cells),
      .depth_cells = static_cast<int>(message.depth_cells),
  };
  return validBounds(bounds) ? std::optional<GridBounds3D>{bounds} : std::nullopt;
}

template<typename Message>
void setGeometry(Message& message, const GridBounds3D& bounds) {
  message.origin_x_m = bounds.origin_x;
  message.origin_y_m = bounds.origin_y;
  message.origin_z_m = bounds.origin_z;
  message.resolution_m = bounds.resolution_m;
  message.width_cells = static_cast<std::uint32_t>(bounds.width_cells);
  message.height_cells = static_cast<std::uint32_t>(bounds.height_cells);
  message.depth_cells = static_cast<std::uint32_t>(bounds.depth_cells);
  message.chunk_size_cells =
      static_cast<std::uint32_t>(ObservedOccupancyGrid3D::kChunkSize);
}

[[nodiscard]] msg::ObservedObstacleChunk3D
makeChunk(const OccupancyChunkIndex3D index,
          const ObservedOccupancyGrid3D::Chunk* const chunk) {
  msg::ObservedObstacleChunk3D message;
  message.x = index.x;
  message.y = index.y;
  message.z = index.z;
  if (chunk != nullptr) {
    message.observed_words = chunk->observed;
    message.occupied_words = chunk->occupied;
  }
  return message;
}

[[nodiscard]] bool chunkIndexInBounds(const OccupancyChunkIndex3D index,
                                      const GridBounds3D& bounds) noexcept {
  if (index.x < 0 || index.y < 0 || index.z < 0) {
    return false;
  }
  const std::int64_t chunk_size = ObservedOccupancyGrid3D::kChunkSize;
  return static_cast<std::int64_t>(index.x) * chunk_size < bounds.width_cells &&
         static_cast<std::int64_t>(index.y) * chunk_size < bounds.height_cells &&
         static_cast<std::int64_t>(index.z) * chunk_size < bounds.depth_cells;
}

[[nodiscard]] bool
validChunks(const std::span<const msg::ObservedObstacleChunk3D> chunks,
            const GridBounds3D& bounds) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> unique;
  for (const msg::ObservedObstacleChunk3D& message : chunks) {
    const OccupancyChunkIndex3D index{message.x, message.y, message.z};
    if (!chunkIndexInBounds(index, bounds) || !unique.insert(index).second) {
      return false;
    }
    for (std::size_t word = 0U; word < message.occupied_words.size(); ++word) {
      if ((message.occupied_words[word] & ~message.observed_words[word]) != 0U) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] bool
applyChunks(ObservedOccupancyGrid3D& grid,
            const std::span<const msg::ObservedObstacleChunk3D> chunks) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> unique;
  for (const msg::ObservedObstacleChunk3D& message : chunks) {
    const OccupancyChunkIndex3D index{message.x, message.y, message.z};
    if (!chunkIndexInBounds(index, grid.bounds()) || !unique.insert(index).second) {
      return false;
    }
    ObservedOccupancyGrid3D::Chunk chunk;
    chunk.observed = message.observed_words;
    chunk.occupied = message.occupied_words;
    static_cast<void>(grid.replaceChunk(index, chunk));
  }
  return true;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double tolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] std::vector<OccupancyChunkIndex3D>
changedChunkIndices(const std::span<const msg::ObservedObstacleChunk3D> chunks) {
  std::vector<OccupancyChunkIndex3D> indices;
  indices.reserve(chunks.size());
  for (const msg::ObservedObstacleChunk3D& chunk : chunks) {
    indices.push_back(OccupancyChunkIndex3D{chunk.x, chunk.y, chunk.z});
  }
  std::ranges::sort(indices, [](const OccupancyChunkIndex3D first,
                                const OccupancyChunkIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return indices;
}

[[nodiscard]] bool
chunksEqual(const ObservedOccupancyGrid3D::Chunk* const first,
            const ObservedOccupancyGrid3D::Chunk* const second) noexcept {
  constexpr ObservedOccupancyGrid3D::Chunk kEmptyChunk{};
  const ObservedOccupancyGrid3D::Chunk& first_value =
      first == nullptr ? kEmptyChunk : *first;
  const ObservedOccupancyGrid3D::Chunk& second_value =
      second == nullptr ? kEmptyChunk : *second;
  return first_value.observed == second_value.observed &&
         first_value.occupied == second_value.occupied;
}

[[nodiscard]] std::vector<OccupancyChunkIndex3D>
changedChunkIndices(const ObservedOccupancyGrid3D& previous,
                    const ObservedOccupancyGrid3D& current) {
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> unique;
  unique.reserve(previous.chunks().size() + current.chunks().size());
  for (const auto& [index, unused] : previous.chunks()) {
    static_cast<void>(unused);
    unique.insert(index);
  }
  for (const auto& [index, unused] : current.chunks()) {
    static_cast<void>(unused);
    unique.insert(index);
  }

  std::vector<OccupancyChunkIndex3D> changed;
  changed.reserve(unique.size());
  for (const OccupancyChunkIndex3D index : unique) {
    const auto previous_chunk = previous.chunks().find(index);
    const auto current_chunk = current.chunks().find(index);
    if (!chunksEqual(
            previous_chunk == previous.chunks().end() ? nullptr
                                                      : &previous_chunk->second.get(),
            current_chunk == current.chunks().end() ? nullptr
                                                    : &current_chunk->second.get())) {
      changed.push_back(index);
    }
  }
  std::ranges::sort(changed, [](const OccupancyChunkIndex3D first,
                                const OccupancyChunkIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return changed;
}

} // namespace

std::uint64_t rawObstacleSnapshot3DWireFingerprint(
    const msg::RawObstacleSnapshot3D& snapshot) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kRawObstacle3DSnapshotWireDomain);
  hashHeader(hash, snapshot.header);
  hashValue(hash, snapshot.producer_instance_id);
  hashValue(hash, snapshot.obstacle_snapshot_revision);
  hashWireGeometry(hash, snapshot);
  hashWireChunks(hash, snapshot);
  return hash == 0U ? 1U : hash;
}

std::uint64_t
rawObstacleDelta3DWireFingerprint(const msg::RawObstacleDelta3D& delta) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kRawObstacle3DDeltaWireDomain);
  hashHeader(hash, delta.header);
  hashValue(hash, delta.producer_instance_id);
  hashValue(hash, delta.base_snapshot_revision);
  hashValue(hash, delta.obstacle_snapshot_revision);
  hashWireGeometry(hash, delta);
  hashWireChunks(hash, delta);
  return hash == 0U ? 1U : hash;
}

msg::RawObstacleSnapshot3D makeRawObstacleSnapshot3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id, const std::uint64_t revision) {
  msg::RawObstacleSnapshot3D message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.obstacle_snapshot_revision = revision;
  setGeometry(message, grid.bounds());
  message.chunks.reserve(grid.chunks().size());
  for (const auto& [index, chunk] : grid.chunks()) {
    message.chunks.push_back(makeChunk(index, &chunk.get()));
  }
  std::ranges::sort(message.chunks, [](const auto& first, const auto& second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  return message;
}

msg::RawObstacleDelta3D makeRawObstacleDelta3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    const std::uint64_t producer_instance_id,
    const std::uint64_t base_snapshot_revision, const std::uint64_t revision,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks) {
  msg::RawObstacleDelta3D message;
  message.header = header;
  message.producer_instance_id = producer_instance_id;
  message.base_snapshot_revision = base_snapshot_revision;
  message.obstacle_snapshot_revision = revision;
  setGeometry(message, grid.bounds());
  message.chunks.reserve(dirty_chunks.size());
  for (const OccupancyChunkIndex3D index : dirty_chunks) {
    const auto found = grid.chunks().find(index);
    message.chunks.push_back(makeChunk(
        index, found == grid.chunks().end() ? nullptr : &found->second.get()));
  }
  return message;
}

RawObstacleGridUpdate3D RawObstacleDeltaAccumulator3D::apply(
    const msg::RawObstacleSnapshot3D& snapshot,
    const ProducerEpochAdmissionState& producer_epoch,
    const std::int64_t receive_stamp_ns, const std::int64_t now_ns,
    const ProducerEpochAdmissionConfig& config, const bool external_contract_valid) {
  const std::int64_t source_stamp_ns = timeNanoseconds(snapshot.header.stamp);
  const ProducerEpochObservation observation{
      .producer_instance_id = snapshot.producer_instance_id,
      .sequence = snapshot.obstacle_snapshot_revision,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = rawObstacleSnapshot3DWireFingerprint(snapshot),
  };
  const bool claimable =
      snapshot.producer_instance_id != 0U && snapshot.obstacle_snapshot_revision != 0U;
  if (!claimable) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
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
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kIdentityConflict};
    }
    return {.state = state_, .dirty_chunks = {}, .status = pending.terminal_status};
  }
  const std::optional<GridBounds3D> bounds = boundsFromMessage(snapshot);
  const bool contract_valid = external_contract_valid && bounds.has_value() &&
                              validChunks(snapshot.chunks, *bounds);
  const auto reconstruct = [&snapshot,
                            &bounds]() -> std::optional<ObservedOccupancyGrid3D> {
    if (!bounds.has_value()) {
      return std::nullopt;
    }
    ObservedOccupancyGrid3D grid{*bounds};
    if (!applyChunks(grid, snapshot.chunks)) {
      return std::nullopt;
    }
    return grid;
  };
  const ProducerEpochAuthority authority = producerEpochAuthority(producer_epoch);
  if (authority.valid() &&
      authority.producer_instance_id == observation.producer_instance_id) {
    if (prospective_identity_capacity_exhausted_ &&
        evidence_admission_.authority_generation != authority.generation) {
      return {.state = state_,
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
    }
    const ProducerEvidenceAdmissionResult admission =
        admitProducerEvidence(config, evidence_admission_, authority, observation, true,
                              now_ns, contract_valid);
    evidence_admission_ = admission.next_state;
    std::vector<OccupancyChunkIndex3D> dirty_chunks;
    bool full_reset{false};
    if (admission.install_evidence) {
      std::optional<ObservedOccupancyGrid3D> grid = reconstruct();
      if (!grid.has_value()) {
        return {.state = state_,
                .dirty_chunks = {},
                .status = RawObstacleGridUpdateStatus3D::kInvalidMessage,
                .evidence_status = ProducerEvidenceAdmissionStatus::kRejectedInvalid,
                .evidence_observation = observation,
                .authority_generation = authority.generation};
      }
      full_reset = !state_.occupancy ||
                   snapshot.producer_instance_id != state_.producer_instance_id ||
                   !sameBounds(*bounds, state_.occupancy->bounds());
      if (!full_reset) {
        dirty_chunks = changedChunkIndices(*state_.occupancy, *grid);
      }
      state_ = RawObstacleGridState3D{
          .producer_instance_id = snapshot.producer_instance_id,
          .base_snapshot_revision = snapshot.obstacle_snapshot_revision,
          .obstacle_snapshot_revision = snapshot.obstacle_snapshot_revision,
          .occupancy =
              std::make_shared<const ObservedOccupancyGrid3D>(std::move(*grid)),
      };
      pruneProspectiveIdentities(observation.producer_instance_id,
                                 observation.sequence);
    }
    return {
        .state = state_,
        .dirty_chunks = std::move(dirty_chunks),
        .status = updateStatus(admission.status),
        .full_reset =
            admission.install_evidence && (full_reset || admission.authority_handoff),
        .evidence_status = admission.status,
        .evidence_observation = observation,
        .authority_generation = authority.generation,
        .current_identity_conflict = admission.current_identity_conflict,
        .authority_handoff = admission.authority_handoff,
    };
  }
  if (producerEpochRetired(producer_epoch, observation.producer_instance_id)) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kStale};
  }
  if (prospective_identity_capacity_exhausted_) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
  }
  if (prospective_identity_count_ == prospective_identities_.size()) {
    prospective_identity_capacity_exhausted_ = true;
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
  }
  const bool sequence_regression =
      observation.sequence <
      prospectiveSequenceHighWater(observation.producer_instance_id);
  const RawObstacleGridUpdateStatus3D terminal_status =
      sequence_regression ? RawObstacleGridUpdateStatus3D::kStale
      : !contract_valid   ? RawObstacleGridUpdateStatus3D::kInvalidMessage
      : !producerEpochObservationFresh(config, observation, now_ns)
          ? RawObstacleGridUpdateStatus3D::kStale
          : RawObstacleGridUpdateStatus3D::kProducerEpochPending;
  prospective_identities_[prospective_identity_count_] =
      ProspectiveIdentity{.observation = observation,
                          .state = {},
                          .terminal_status = terminal_status,
                          .full_snapshot = true,
                          .contract_valid = contract_valid && !sequence_regression,
                          .identity_conflicted = false};
  if (terminal_status == RawObstacleGridUpdateStatus3D::kProducerEpochPending) {
    std::optional<ObservedOccupancyGrid3D> grid = reconstruct();
    if (grid.has_value()) {
      prospective_identities_[prospective_identity_count_].state =
          RawObstacleGridState3D{
              .producer_instance_id = snapshot.producer_instance_id,
              .base_snapshot_revision = snapshot.obstacle_snapshot_revision,
              .obstacle_snapshot_revision = snapshot.obstacle_snapshot_revision,
              .occupancy =
                  std::make_shared<const ObservedOccupancyGrid3D>(std::move(*grid)),
          };
    } else {
      prospective_identities_[prospective_identity_count_].terminal_status =
          RawObstacleGridUpdateStatus3D::kInvalidMessage;
      prospective_identities_[prospective_identity_count_].contract_valid = false;
    }
  }
  ++prospective_identity_count_;
  return {
      .state = state_,
      .dirty_chunks = {},
      .status =
          prospective_identities_[prospective_identity_count_ - 1U].terminal_status};
}

RawObstacleGridUpdate3D RawObstacleDeltaAccumulator3D::apply(
    const msg::RawObstacleDelta3D& delta,
    const ProducerEpochAdmissionState& producer_epoch,
    const std::int64_t receive_stamp_ns, const std::int64_t now_ns,
    const ProducerEpochAdmissionConfig& config, const bool external_contract_valid) {
  const std::int64_t source_stamp_ns = timeNanoseconds(delta.header.stamp);
  const ProducerEpochObservation observation{
      .producer_instance_id = delta.producer_instance_id,
      .sequence = delta.obstacle_snapshot_revision,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = rawObstacleDelta3DWireFingerprint(delta),
  };
  if (delta.producer_instance_id == 0U || delta.obstacle_snapshot_revision == 0U) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kInvalidMessage};
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
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kIdentityConflict};
    }
    return {.state = state_, .dirty_chunks = {}, .status = pending.terminal_status};
  }
  const ProducerEpochAuthority authority = producerEpochAuthority(producer_epoch);
  if (!authority.valid() ||
      delta.producer_instance_id != authority.producer_instance_id) {
    if (producerEpochRetired(producer_epoch, observation.producer_instance_id)) {
      return {.state = state_,
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kStale};
    }
    if (prospective_identity_capacity_exhausted_) {
      return {.state = state_,
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
    }
    if (prospective_identity_count_ == prospective_identities_.size()) {
      prospective_identity_capacity_exhausted_ = true;
      return {.state = state_,
              .dirty_chunks = {},
              .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
    }
    const bool sequence_regression =
        observation.sequence <
        prospectiveSequenceHighWater(observation.producer_instance_id);
    const RawObstacleGridUpdateStatus3D terminal_status =
        sequence_regression ? RawObstacleGridUpdateStatus3D::kStale
        : !external_contract_valid || source_stamp_ns <= 0 || receive_stamp_ns <= 0
            ? RawObstacleGridUpdateStatus3D::kInvalidMessage
        : !producerEpochObservationFresh(config, observation, now_ns)
            ? RawObstacleGridUpdateStatus3D::kStale
        : authority.valid() ? RawObstacleGridUpdateStatus3D::kBaseUnavailable
                            : RawObstacleGridUpdateStatus3D::kProducerEpochUnavailable;
    prospective_identities_[prospective_identity_count_] = ProspectiveIdentity{
        .observation = observation,
        .state = {},
        .terminal_status = terminal_status,
        .full_snapshot = false,
        .contract_valid = false,
        .identity_conflicted = false,
    };
    ++prospective_identity_count_;
    return {.state = state_, .dirty_chunks = {}, .status = terminal_status};
  }
  if (!state_.occupancy ||
      evidence_admission_.authority_generation != authority.generation ||
      delta.producer_instance_id != state_.producer_instance_id ||
      delta.base_snapshot_revision != state_.base_snapshot_revision) {
    const ProducerEvidenceAdmissionResult admission = admitProducerEvidence(
        config, evidence_admission_, authority, observation, false, now_ns, false);
    evidence_admission_ = admission.next_state;
    const RawObstacleGridUpdateStatus3D status =
        admission.status == ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict
            ? RawObstacleGridUpdateStatus3D::kIdentityConflict
        : admission.status == ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity
            ? RawObstacleGridUpdateStatus3D::kProducerEpochCapacity
            : RawObstacleGridUpdateStatus3D::kBaseUnavailable;
    return {
        .state = state_,
        .dirty_chunks = {},
        .status = status,
        .evidence_status = admission.status,
        .evidence_observation = observation,
        .authority_generation = authority.generation,
        .current_identity_conflict = admission.current_identity_conflict,
        .authority_handoff = admission.authority_handoff,
    };
  }
  const std::optional<GridBounds3D> bounds = boundsFromMessage(delta);
  const auto admit_candidate =
      [this, &config, &authority, &observation,
       now_ns](const bool contract_valid) -> RawObstacleGridUpdate3D {
    const ProducerEvidenceAdmissionResult admission =
        admitProducerEvidence(config, evidence_admission_, authority, observation,
                              false, now_ns, contract_valid);
    evidence_admission_ = admission.next_state;
    return {
        .state = state_,
        .dirty_chunks = {},
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
  if (!bounds.has_value() || delta.chunks.empty() ||
      !sameBounds(*bounds, state_.occupancy->bounds()) ||
      !validChunks(delta.chunks, *bounds)) {
    return admit_candidate(false);
  }
  RawObstacleGridUpdate3D admitted = admit_candidate(true);
  if (!admitted.accepted()) {
    return admitted;
  }
  ObservedOccupancyGrid3D updated = *state_.occupancy;
  if (!applyChunks(updated, delta.chunks)) {
    return admitted;
  }
  RawObstacleGridState3D candidate = state_;
  candidate.obstacle_snapshot_revision = delta.obstacle_snapshot_revision;
  candidate.occupancy =
      std::make_shared<const ObservedOccupancyGrid3D>(std::move(updated));
  state_ = std::move(candidate);
  pruneProspectiveIdentities(observation.producer_instance_id, observation.sequence);
  admitted.state = state_;
  admitted.dirty_chunks = changedChunkIndices(delta.chunks);
  return admitted;
}

void RawObstacleDeltaAccumulator3D::pruneProspectiveIdentities(
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

std::uint64_t RawObstacleDeltaAccumulator3D::prospectiveSequenceHighWater(
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

RawObstacleGridUpdate3D RawObstacleDeltaAccumulator3D::synchronizeProducerEpoch(
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
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kProducerEpochUnavailable};
  }
  if (prospective_identity_capacity_exhausted_ &&
      evidence_admission_.authority_generation != authority.generation) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kProducerEpochCapacity};
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
              .dirty_chunks = {},
              .status = evidence_admission_.current_identity_conflicted
                            ? RawObstacleGridUpdateStatus3D::kIdentityConflict
                            : RawObstacleGridUpdateStatus3D::kIdempotentReplay};
    }
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kProducerEpochUnavailable};
  }
  const ProspectiveIdentity selected_pending = prospective_identities_[*selected];
  if (selected_pending.identity_conflicted) {
    return {.state = state_,
            .dirty_chunks = {},
            .status = RawObstacleGridUpdateStatus3D::kIdentityConflict};
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
      .dirty_chunks = {},
      .status = updateStatus(admission.status),
      .full_reset = admission.install_evidence,
      .evidence_status = admission.status,
      .evidence_observation = selected_pending.observation,
      .authority_generation = authority.generation,
      .current_identity_conflict = admission.current_identity_conflict,
      .authority_handoff = admission.authority_handoff,
  };
}

const RawObstacleGridState3D& RawObstacleDeltaAccumulator3D::state() const noexcept {
  return state_;
}

const ProducerEvidenceAdmissionState&
RawObstacleDeltaAccumulator3D::evidenceAdmissionState() const noexcept {
  return evidence_admission_;
}

const char*
rawObstacleGridUpdateStatus3DName(const RawObstacleGridUpdateStatus3D status) noexcept {
  switch (status) {
    case RawObstacleGridUpdateStatus3D::kAccepted:
      return "accepted";
    case RawObstacleGridUpdateStatus3D::kIdempotentReplay:
      return "idempotent_replay";
    case RawObstacleGridUpdateStatus3D::kProducerEpochPending:
      return "producer_epoch_pending";
    case RawObstacleGridUpdateStatus3D::kProducerEpochUnavailable:
      return "producer_epoch_unavailable";
    case RawObstacleGridUpdateStatus3D::kProducerEpochCapacity:
      return "producer_epoch_capacity";
    case RawObstacleGridUpdateStatus3D::kIdentityConflict:
      return "identity_conflict";
    case RawObstacleGridUpdateStatus3D::kStale:
      return "stale";
    case RawObstacleGridUpdateStatus3D::kBaseUnavailable:
      return "base_unavailable";
    case RawObstacleGridUpdateStatus3D::kInvalidMessage:
      return "invalid_message";
  }
  return "unknown";
}

} // namespace drone_city_nav

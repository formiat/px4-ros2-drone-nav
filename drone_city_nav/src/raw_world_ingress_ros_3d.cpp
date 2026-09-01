#include "raw_world_ingress_ros_3d.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::uint64_t kMemoryStatusFingerprintDomain{0x4d454d5354415455ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= kFnvPrime;
  }
}

void hashString(std::uint64_t& hash, const std::string& value) noexcept {
  hashValue(hash, value.size());
  for (const char character : value) {
    hashValue(hash, static_cast<unsigned char>(character));
  }
}

[[nodiscard]] std::uint64_t
memoryStatusFingerprint(const msg::ObstacleMemoryStatus& message) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kMemoryStatusFingerprintDomain);
  hashString(hash, message.header.frame_id);
  hashValue(hash, message.occupied_cell_count);
  hashValue(hash, message.raw_snapshot_published ? 1U : 0U);
  hashValue(hash, message.raw_delta_published ? 1U : 0U);
  hashValue(hash, message.full_snapshot_published ? 1U : 0U);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] bool
statusAnnouncesRawUpdate(const msg::ObstacleMemoryStatus& message) noexcept {
  return message.raw_snapshot_published || message.raw_delta_published;
}

[[nodiscard]] bool
evidenceMatches(const ProducerEvidenceAdmissionState& evidence,
                const ProducerEpochAuthority& authority,
                const ProducerEpochObservation& observation) noexcept {
  return authority.valid() && !evidence.current_identity_conflicted &&
         evidence.authority_generation == authority.generation &&
         evidence.producer_instance_id == authority.producer_instance_id &&
         evidence.producer_instance_id == observation.producer_instance_id &&
         evidence.sequence == observation.sequence &&
         evidence.source_stamp_ns == observation.source_stamp_ns &&
         evidence.content_fingerprint == observation.content_fingerprint;
}

[[nodiscard]] std::shared_ptr<const ProductionMppiRawWorld3D>
makeRawWorld(const RawObstacleGridUpdate3D& update, const double reconstruction_ms,
             const std::int64_t ready_stamp_ns) {
  const RawMapVersion version{
      .producer_instance_id = update.state.producer_instance_id,
      .base_snapshot_revision = update.state.base_snapshot_revision,
      .revision = update.state.obstacle_snapshot_revision,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner =
      VersionedObservedRawWorld3D::captureOwned(version, update.state.occupancy,
                                                std::nullopt, std::nullopt);
  if (execution_owner == nullptr) {
    return nullptr;
  }
  return ProductionMppiRawWorld3D::capture(
      execution_owner,
      ProductionMppiRawWorldMetadata3D{
          .source_stamp_ns = update.evidence_observation.source_stamp_ns,
          .receive_stamp_ns = update.evidence_observation.receive_stamp_ns,
          .ready_stamp_ns = ready_stamp_ns,
          .reconstruction_ms = reconstruction_ms,
          .dirty_chunks = update.dirty_chunks,
          .full_reset = update.full_reset,
      });
}

} // namespace

std::string_view
rawWorldCommitStatus3DName(const RawWorldCommitStatus3D status) noexcept {
  switch (status) {
    case RawWorldCommitStatus3D::kCommitted:
      return "committed";
    case RawWorldCommitStatus3D::kStopped:
      return "stopped";
    case RawWorldCommitStatus3D::kInvalidUpdate:
      return "invalid_update";
    case RawWorldCommitStatus3D::kInvalidExecutionOwner:
      return "invalid_execution_owner";
    case RawWorldCommitStatus3D::kSupersededEvidence:
      return "superseded_evidence";
  }
  return "unknown";
}

RawWorldIngressRos3D::RawWorldIngressRos3D(WorldPipeline3D& world_pipeline,
                                           RawWorldIngressRosConfig3D config)
    : world_pipeline_{world_pipeline},
      config_{std::move(config)} {
  if (config_.frame_id.empty() ||
      config_.producer_epoch.maximum_observation_age_ns <= 0 ||
      config_.producer_epoch.maximum_confirmation_interval_ns <= 0) {
    throw std::invalid_argument{"raw world ingress configuration is invalid"};
  }
}

RawWorldIngestionResult3D
RawWorldIngressRos3D::ingestRawSnapshot(const msg::RawObstacleSnapshot3D& message,
                                        const std::int64_t receive_stamp_ns) {
  const std::scoped_lock lock{mutex_};
  return finishRawIngestionLocked(raw_delta_accumulator_.apply(
      message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
      receive_stamp_ns, config_.producer_epoch,
      message.header.frame_id == config_.frame_id));
}

RawWorldIngestionResult3D
RawWorldIngressRos3D::ingestRawDelta(const msg::RawObstacleDelta3D& message,
                                     const std::int64_t receive_stamp_ns) {
  const std::scoped_lock lock{mutex_};
  return finishRawIngestionLocked(raw_delta_accumulator_.apply(
      message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
      receive_stamp_ns, config_.producer_epoch,
      message.header.frame_id == config_.frame_id));
}

RawWorldIngestionResult3D
RawWorldIngressRos3D::finishRawIngestionLocked(RawObstacleGridUpdate3D update) {
  const bool conflict_started =
      update.current_identity_conflict && !raw_world_identity_conflicted_;
  if (update.current_identity_conflict) {
    invalidateRawWorldLocked();
  }
  return RawWorldIngestionResult3D{
      .update = std::move(update),
      .execution_revocation_required = conflict_started,
  };
}

MemoryStatusIngestionResult3D
RawWorldIngressRos3D::ingestMemoryStatus(const msg::ObstacleMemoryStatus& message,
                                         const std::int64_t now_ns) {
  const ProducerEpochObservation observation{
      .producer_instance_id = message.producer_instance_id,
      .sequence = message.sequence,
      .source_stamp_ns = timeNanoseconds(message.header.stamp),
      .receive_stamp_ns = now_ns,
      .content_fingerprint = memoryStatusFingerprint(message),
  };
  const std::scoped_lock lock{mutex_};
  const ProducerEpochAdmissionResult admission =
      latest_observation_tracker_.observe(config_.producer_epoch, observation, now_ns,
                                          message.header.frame_id == config_.frame_id);
  const bool authority_boundary =
      admission.status == ProducerEpochAdmissionStatus::kAcceptedInitial ||
      admission.producer_handoff;
  bool execution_revocation_required = false;
  if (authority_boundary) {
    raw_world_identity_conflicted_ = false;
    pending_raw_world_update_ = {};
  }
  if (admission.current_identity_conflict) {
    execution_revocation_required = !raw_world_identity_conflicted_;
    invalidateRawWorldLocked();
  }
  if (admission.install_observation && statusAnnouncesRawUpdate(message)) {
    const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
    const ProducerEvidenceAdmissionState& evidence =
        raw_delta_accumulator_.evidenceAdmissionState();
    const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
        world_pipeline_.latestRawWorld();
    const bool raw_pointer_current =
        raw_world != nullptr &&
        raw_world->version().producer_instance_id == authority.producer_instance_id &&
        raw_world->version().revision == evidence.sequence;
    const bool installed_through_status =
        authority.valid() && !evidence.current_identity_conflicted &&
        evidence.authority_generation == authority.generation &&
        evidence.producer_instance_id == authority.producer_instance_id &&
        evidence.source_stamp_ns >= observation.source_stamp_ns && raw_pointer_current;
    if (!installed_through_status) {
      pending_raw_world_update_ = ProductionMppiPendingRawWorldUpdate{
          .authority_generation = authority.generation,
          .producer_instance_id = authority.producer_instance_id,
          .announced_sequence = observation.sequence,
          .minimum_source_stamp_ns = observation.source_stamp_ns,
      };
    } else if (raw_world != nullptr &&
               pending_raw_world_update_.satisfiedBy(evidence, raw_world->version())) {
      pending_raw_world_update_ = {};
    }
  }
  std::optional<RawObstacleGridUpdate3D> synchronized =
      raw_delta_accumulator_.synchronizeProducerEpoch(
          latest_observation_tracker_.admissionState(), now_ns, config_.producer_epoch);
  if (synchronized.has_value() && synchronized->current_identity_conflict) {
    if (!raw_world_identity_conflicted_) {
      execution_revocation_required = true;
    }
    invalidateRawWorldLocked();
  }
  return MemoryStatusIngestionResult3D{
      .synchronized_update = std::move(synchronized),
      .execution_revocation_required = execution_revocation_required,
  };
}

RawWorldCommitResult3D
RawWorldIngressRos3D::commitRawUpdate(const RawObstacleGridUpdate3D& update,
                                      const double reconstruction_ms,
                                      const std::int64_t ready_stamp_ns) {
  if (!update.accepted()) {
    return {
        .status = RawWorldCommitStatus3D::kInvalidUpdate,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
      makeRawWorld(update, reconstruction_ms, ready_stamp_ns);
  if (raw_world == nullptr) {
    return {
        .status = RawWorldCommitStatus3D::kInvalidExecutionOwner,
        .world = nullptr,
        .replaced_pending = false,
    };
  }

  const std::scoped_lock lock{mutex_};
  const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
  const LatestObservation& status = latest_observation_tracker_.latest();
  const ProducerEvidenceAdmissionState& evidence =
      raw_delta_accumulator_.evidenceAdmissionState();
  if (!status.available() ||
      status.producer_instance_id != authority.producer_instance_id ||
      status.producer_epoch_generation != authority.generation ||
      update.authority_generation != authority.generation ||
      !evidenceMatches(evidence, authority, update.evidence_observation) ||
      !producerEpochObservationFresh(config_.producer_epoch,
                                     update.evidence_observation, ready_stamp_ns)) {
    return {
        .status = RawWorldCommitStatus3D::kSupersededEvidence,
        .world = nullptr,
        .replaced_pending = false,
    };
  }

  const RawWorldPublicationResult3D publication =
      world_pipeline_.publishRawWorld(raw_world);
  if (publication.status == RawWorldPublicationStatus3D::kStopped) {
    return {
        .status = RawWorldCommitStatus3D::kStopped,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  if (!publication.published()) {
    return {
        .status = RawWorldCommitStatus3D::kInvalidExecutionOwner,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  raw_world_identity_conflicted_ = false;
  if (pending_raw_world_update_.satisfiedBy(evidence, publication.world->version())) {
    pending_raw_world_update_ = {};
  }
  return RawWorldCommitResult3D{
      .status = RawWorldCommitStatus3D::kCommitted,
      .world = publication.world,
      .replaced_pending = publication.replaced_pending,
  };
}

RawWorldIngressSnapshot3D RawWorldIngressRos3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return RawWorldIngressSnapshot3D{
      .latest_observation = latest_observation_tracker_.latest(),
      .latest_raw_world = world_pipeline_.latestRawWorld(),
      .raw_world_identity_conflicted = raw_world_identity_conflicted_,
  };
}

void RawWorldIngressRos3D::invalidateRawWorldLocked() noexcept {
  raw_world_identity_conflicted_ = true;
  world_pipeline_.invalidateRawWorld();
}

} // namespace drone_city_nav

#include "production_mppi_raw_world.hpp"

#include <cmath>
#include <memory>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameRawMapVersion(const RawMapVersion& first,
                                     const RawMapVersion& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.base_snapshot_revision == second.base_snapshot_revision &&
         first.revision == second.revision;
}

} // namespace

ProductionMppiRawWorld3D::ProductionMppiRawWorld3D(
    CaptureToken /*capture_token*/,
    std::shared_ptr<const VersionedObservedRawWorld3D> authoritative_owner,
    ProductionMppiRawWorldMetadata3D metadata)
    : authoritative_owner_{std::move(authoritative_owner)},
      metadata_{std::move(metadata)} {
}

std::shared_ptr<const ProductionMppiRawWorld3D> ProductionMppiRawWorld3D::capture(
    std::shared_ptr<const VersionedObservedRawWorld3D> authoritative_owner,
    ProductionMppiRawWorldMetadata3D metadata) {
  if (authoritative_owner == nullptr || !authoritative_owner->valid() ||
      authoritative_owner->proprioceptiveFreeSpaceSeed().has_value() ||
      authoritative_owner->launchSupportContact().has_value() || !metadata.valid()) {
    return nullptr;
  }
  auto result = std::make_shared<const ProductionMppiRawWorld3D>(
      CaptureToken{}, std::move(authoritative_owner), std::move(metadata));
  return result->valid() ? result : nullptr;
}

const RawMapVersion& ProductionMppiRawWorld3D::version() const noexcept {
  return authoritative_owner_->version();
}

const ObservedOccupancyGrid3D& ProductionMppiRawWorld3D::occupancy() const noexcept {
  return authoritative_owner_->occupancy();
}

const std::shared_ptr<const ObservedOccupancyGrid3D>&
ProductionMppiRawWorld3D::occupancyOwner() const noexcept {
  return authoritative_owner_->occupancyOwner();
}

const std::shared_ptr<const VersionedObservedRawWorld3D>&
ProductionMppiRawWorld3D::authoritativeOwner() const noexcept {
  return authoritative_owner_;
}

std::shared_ptr<const VersionedObservedRawWorld3D>
ProductionMppiRawWorld3D::deriveRouteEvidence(
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact) const {
  if (!valid()) {
    return nullptr;
  }
  std::shared_ptr<const VersionedObservedRawWorld3D> evidence =
      authoritative_owner_->deriveRouteEvidence(proprioceptive_free_space_seed,
                                                std::move(launch_support_contact));
  return evidence != nullptr && ownsRouteEvidence(*evidence) ? evidence : nullptr;
}

bool ProductionMppiRawWorld3D::ownsRouteEvidence(
    const VersionedObservedRawWorld3D& evidence) const noexcept {
  return valid() && evidence.valid() &&
         sameRawMapVersion(version(), evidence.version()) &&
         authoritative_owner_->sharesObservationOwner(evidence) &&
         authoritative_owner_->occupiedSnapshot() == evidence.occupiedSnapshot();
}

std::int64_t ProductionMppiRawWorld3D::sourceStampNs() const noexcept {
  return metadata_.source_stamp_ns;
}

std::int64_t ProductionMppiRawWorld3D::receiveStampNs() const noexcept {
  return metadata_.receive_stamp_ns;
}

std::int64_t ProductionMppiRawWorld3D::readyStampNs() const noexcept {
  return metadata_.ready_stamp_ns;
}

double ProductionMppiRawWorld3D::reconstructionMs() const noexcept {
  return metadata_.reconstruction_ms;
}

const std::vector<OccupancyChunkIndex3D>&
ProductionMppiRawWorld3D::dirtyChunks() const noexcept {
  return metadata_.dirty_chunks;
}

bool ProductionMppiRawWorld3D::fullReset() const noexcept {
  return metadata_.full_reset;
}

bool ProductionMppiRawWorld3D::valid() const noexcept {
  return authoritative_owner_ != nullptr && authoritative_owner_->valid() &&
         !authoritative_owner_->proprioceptiveFreeSpaceSeed().has_value() &&
         !authoritative_owner_->launchSupportContact().has_value() && metadata_.valid();
}

double ProductionMppiRawWorld3D::proprioceptiveContactToleranceM() const noexcept {
  if (!valid()) {
    return 0.0;
  }
  const double resolution_m = occupancy().bounds().resolution_m;
  return std::isfinite(resolution_m) && resolution_m > 0.0 ? 0.5 * resolution_m : 0.0;
}

} // namespace drone_city_nav

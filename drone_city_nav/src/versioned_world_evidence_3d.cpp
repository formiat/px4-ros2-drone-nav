#include "drone_city_nav/versioned_world_evidence_3d.hpp"

#include "drone_city_nav/observed_world_content_hash_3d.hpp"

#include <algorithm>
#include <utility>

namespace drone_city_nav {
namespace {

using namespace observed_world_content_3d;

} // namespace

bool NavigationWorldCertificate3D::valid() const noexcept {
  return esdf_fingerprint != 0U &&
         raw_validated_through_revision >= esdf_source_raw_revision;
}

VersionedObservedRawWorld3D::VersionedObservedRawWorld3D(
    CaptureToken /*capture_token*/, RawMapVersion version,
    std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
    std::shared_ptr<const OccupancyGrid3D> occupied_snapshot,
    const std::uint64_t observation_content_fingerprint,
    const std::uint64_t occupied_content_fingerprint,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact)
    : version_{version},
      occupancy_{std::move(occupancy)},
      occupied_snapshot_{std::move(occupied_snapshot)},
      observation_content_fingerprint_{observation_content_fingerprint},
      occupied_content_fingerprint_{occupied_content_fingerprint},
      proprioceptive_free_space_seed_{proprioceptive_free_space_seed},
      launch_support_contact_{std::move(launch_support_contact)} {
  content_fingerprint_ =
      occupancy_ == nullptr
          ? 0U
          : observedWorldContentFingerprintFromObservation(
                observation_content_fingerprint_,
                proprioceptive_free_space_seed_.has_value()
                    ? &*proprioceptive_free_space_seed_
                    : nullptr,
                launch_support_contact_.has_value() ? &*launch_support_contact_
                                                    : nullptr);
}

std::shared_ptr<const VersionedObservedRawWorld3D> VersionedObservedRawWorld3D::capture(
    const RawMapVersion version, const ObservedOccupancyGrid3D& occupancy,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact) {
  return captureOwned(
      version, std::make_shared<const ObservedOccupancyGrid3D>(occupancy),
      proprioceptive_free_space_seed, std::move(launch_support_contact));
}

std::shared_ptr<const VersionedObservedRawWorld3D>
VersionedObservedRawWorld3D::captureOwned(
    const RawMapVersion version,
    std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact) {
  const ProprioceptiveFreeSpaceSeed3D* const seed =
      proprioceptive_free_space_seed.has_value()
          ? std::addressof(*proprioceptive_free_space_seed)
          : nullptr;
  const LaunchSupportContact3D* const support =
      launch_support_contact.has_value() ? std::addressof(*launch_support_contact)
                                         : nullptr;
  std::uint64_t seed_hash{kFnvOffset};
  if (!version.valid() || version.producer_instance_id == 0U ||
      !hashProprioceptiveFreeSpaceSeed(seed_hash, seed) || occupancy == nullptr ||
      (support != nullptr &&
       (seed == nullptr || !sameFreeSpaceSeed(*seed, support->seed) ||
        !launchSupportMatchesOwnedOccupancy(*support, *occupancy)))) {
    return nullptr;
  }
  const std::uint64_t observation_content_fingerprint =
      observedOccupancyContentFingerprint(*occupancy);
  const auto occupied_snapshot =
      std::make_shared<const OccupancyGrid3D>(occupancy->occupiedSnapshot());
  // The observed grid computes the occupied fingerprint canonically and caches
  // it; every later consumer of this shared grid reuses it instead of
  // materializing another dense snapshot.
  const std::uint64_t occupied_content_fingerprint =
      occupancy->occupiedContentFingerprint();
  auto result = std::make_shared<const VersionedObservedRawWorld3D>(
      CaptureToken{}, version, std::move(occupancy), occupied_snapshot,
      observation_content_fingerprint, occupied_content_fingerprint,
      proprioceptive_free_space_seed, std::move(launch_support_contact));
  if (!result->valid()) {
    return nullptr;
  }
  return result;
}

const RawMapVersion& VersionedObservedRawWorld3D::version() const noexcept {
  return version_;
}

const ObservedOccupancyGrid3D& VersionedObservedRawWorld3D::occupancy() const noexcept {
  return *occupancy_;
}

const std::shared_ptr<const ObservedOccupancyGrid3D>&
VersionedObservedRawWorld3D::occupancyOwner() const noexcept {
  return occupancy_;
}

std::uint64_t VersionedObservedRawWorld3D::contentFingerprint() const noexcept {
  return content_fingerprint_;
}

std::uint64_t VersionedObservedRawWorld3D::occupiedContentFingerprint() const noexcept {
  return occupied_content_fingerprint_;
}

std::shared_ptr<const OccupancyGrid3D>
VersionedObservedRawWorld3D::occupiedSnapshot() const noexcept {
  return occupied_snapshot_;
}

bool VersionedObservedRawWorld3D::sharesObservationOwner(
    const VersionedObservedRawWorld3D& other) const noexcept {
  return occupancy_ != nullptr && occupancy_ == other.occupancy_;
}

std::shared_ptr<const VersionedObservedRawWorld3D>
VersionedObservedRawWorld3D::deriveRouteEvidence(
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact) const {
  const ProprioceptiveFreeSpaceSeed3D* const seed =
      proprioceptive_free_space_seed.has_value()
          ? std::addressof(*proprioceptive_free_space_seed)
          : nullptr;
  const LaunchSupportContact3D* const support =
      launch_support_contact.has_value() ? std::addressof(*launch_support_contact)
                                         : nullptr;
  std::uint64_t seed_hash{kFnvOffset};
  if (!valid() || !hashProprioceptiveFreeSpaceSeed(seed_hash, seed) ||
      (support != nullptr &&
       (seed == nullptr || !sameFreeSpaceSeed(*seed, support->seed) ||
        !launchSupportMatchesOwnedOccupancy(*support, *occupancy_)))) {
    return nullptr;
  }
  auto result = std::make_shared<const VersionedObservedRawWorld3D>(
      CaptureToken{}, version_, occupancy_, occupied_snapshot_,
      observation_content_fingerprint_, occupied_content_fingerprint_,
      proprioceptive_free_space_seed, std::move(launch_support_contact));
  return result->valid() && result->sharesObservationOwner(*this) &&
                 result->occupied_snapshot_ == occupied_snapshot_
             ? result
             : nullptr;
}

const std::optional<ProprioceptiveFreeSpaceSeed3D>&
VersionedObservedRawWorld3D::proprioceptiveFreeSpaceSeed() const noexcept {
  return proprioceptive_free_space_seed_;
}

const std::optional<LaunchSupportContact3D>&
VersionedObservedRawWorld3D::launchSupportContact() const noexcept {
  return launch_support_contact_;
}

bool VersionedObservedRawWorld3D::valid() const noexcept {
  return version_.valid() && version_.producer_instance_id != 0U &&
         occupancy_ != nullptr && occupied_snapshot_ != nullptr &&
         content_fingerprint_ != 0U && occupied_content_fingerprint_ != 0U &&
         (!launch_support_contact_.has_value() ||
          (proprioceptive_free_space_seed_.has_value() &&
           launchSupportContactValid3D(*launch_support_contact_) &&
           sameFreeSpaceSeed(*proprioceptive_free_space_seed_,
                             launch_support_contact_->seed)));
}

VersionedStaticWorld3D::VersionedStaticWorld3D(
    CaptureToken /*capture_token*/, NavigationWorldCertificate3D certificate,
    std::shared_ptr<const OccupancyGrid3D> occupancy,
    const std::uint64_t content_fingerprint)
    : certificate_{certificate},
      occupancy_{std::move(occupancy)},
      content_fingerprint_{content_fingerprint} {
}

std::shared_ptr<const VersionedStaticWorld3D>
VersionedStaticWorld3D::capture(const NavigationWorldCertificate3D certificate,
                                const OccupancyGrid3D& occupancy) {
  auto owned_occupancy = std::make_shared<const OccupancyGrid3D>(occupancy);
  return captureOwned(certificate, std::move(owned_occupancy));
}

std::shared_ptr<const VersionedStaticWorld3D>
VersionedStaticWorld3D::captureOwned(const NavigationWorldCertificate3D certificate,
                                     std::shared_ptr<const OccupancyGrid3D> occupancy) {
  if (occupancy == nullptr) {
    return nullptr;
  }
  const std::uint64_t expected_fingerprint =
      certificate.esdf_source_occupied_fingerprint != 0U
          ? certificate.esdf_source_occupied_fingerprint
          : certificate.esdf_fingerprint;
  const std::uint64_t occupancy_content_fingerprint = occupancy->contentFingerprint();
  if (!certificate.valid() || expected_fingerprint == 0U ||
      occupancy->fingerprint() != expected_fingerprint ||
      occupancy_content_fingerprint == 0U) {
    return nullptr;
  }
  return std::make_shared<const VersionedStaticWorld3D>(
      CaptureToken{}, certificate, std::move(occupancy), occupancy_content_fingerprint);
}

const NavigationWorldCertificate3D&
VersionedStaticWorld3D::certificate() const noexcept {
  return certificate_;
}

const OccupancyGrid3D& VersionedStaticWorld3D::occupancy() const noexcept {
  return *occupancy_;
}

std::uint64_t VersionedStaticWorld3D::contentFingerprint() const noexcept {
  return content_fingerprint_;
}

bool VersionedStaticWorld3D::valid() const noexcept {
  const std::uint64_t expected_fingerprint =
      certificate_.esdf_source_occupied_fingerprint != 0U
          ? certificate_.esdf_source_occupied_fingerprint
          : certificate_.esdf_fingerprint;
  return certificate_.valid() && occupancy_ != nullptr && expected_fingerprint != 0U &&
         occupancy_->fingerprint() == expected_fingerprint &&
         content_fingerprint_ != 0U;
}
} // namespace drone_city_nav

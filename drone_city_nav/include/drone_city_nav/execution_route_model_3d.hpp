#pragma once

#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

class ObservedOccupancyGrid3D;
class OccupancyGrid3D;

struct RouteInstanceId3D {
  std::uint64_t value{0U};

  [[nodiscard]] bool valid() const noexcept {
    return value != 0U;
  }

  friend bool operator==(const RouteInstanceId3D&, const RouteInstanceId3D&) = default;
};

// Immutable progress evidence for one certified route revision. A route waiting
// for activation has no execution input; every activated route owns the exact
// input that established its current station and position.
struct CertifiedRouteProgress3D {
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  double station_m{0.0};
  Point3 last_observed_position{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;

  [[nodiscard]] bool valid() const noexcept;
};

// An observed world is captured once and shared by identity. The derived binary
// occupancy snapshot is part of the same immutable owner, so route evidence
// cannot mix observation and collision generations.
class VersionedObservedRawWorld3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedObservedRawWorld3D>
  capture(RawMapVersion version, const ObservedOccupancyGrid3D& occupancy,
          std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
          std::optional<LaunchSupportContact3D> launch_support_contact);

  [[nodiscard]] static std::shared_ptr<const VersionedObservedRawWorld3D> captureOwned(
      RawMapVersion version, std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact);

  [[nodiscard]] const RawMapVersion& version() const noexcept;
  [[nodiscard]] const ObservedOccupancyGrid3D& occupancy() const noexcept;
  [[nodiscard]] const std::shared_ptr<const ObservedOccupancyGrid3D>&
  occupancyOwner() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] std::uint64_t occupiedContentFingerprint() const noexcept;
  [[nodiscard]] std::shared_ptr<const OccupancyGrid3D>
  occupiedSnapshot() const noexcept;
  [[nodiscard]] bool
  sharesObservationOwner(const VersionedObservedRawWorld3D& other) const noexcept;
  [[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D> deriveRouteEvidence(
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact) const;
  [[nodiscard]] const std::optional<ProprioceptiveFreeSpaceSeed3D>&
  proprioceptiveFreeSpaceSeed() const noexcept;
  [[nodiscard]] const std::optional<LaunchSupportContact3D>&
  launchSupportContact() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedObservedRawWorld3D(
      CaptureToken, RawMapVersion version,
      std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      std::shared_ptr<const OccupancyGrid3D> occupied_snapshot,
      std::uint64_t observation_content_fingerprint,
      std::uint64_t occupied_content_fingerprint,
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact);

private:
  RawMapVersion version_{};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy_;
  std::shared_ptr<const OccupancyGrid3D> occupied_snapshot_;
  std::uint64_t observation_content_fingerprint_{0U};
  std::uint64_t content_fingerprint_{0U};
  std::uint64_t occupied_content_fingerprint_{0U};
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed_;
  std::optional<LaunchSupportContact3D> launch_support_contact_;
};

class VersionedStaticWorld3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedStaticWorld3D>
  capture(NavigationWorldCertificate3D certificate, const OccupancyGrid3D& occupancy);

  [[nodiscard]] static std::shared_ptr<const VersionedStaticWorld3D>
  captureOwned(NavigationWorldCertificate3D certificate,
               std::shared_ptr<const OccupancyGrid3D> occupancy);

  [[nodiscard]] const NavigationWorldCertificate3D& certificate() const noexcept;
  [[nodiscard]] const OccupancyGrid3D& occupancy() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedStaticWorld3D(CaptureToken, NavigationWorldCertificate3D certificate,
                         std::shared_ptr<const OccupancyGrid3D> occupancy,
                         std::uint64_t content_fingerprint);

private:
  NavigationWorldCertificate3D certificate_{};
  std::shared_ptr<const OccupancyGrid3D> occupancy_;
  std::uint64_t content_fingerprint_{0U};
};

} // namespace drone_city_nav

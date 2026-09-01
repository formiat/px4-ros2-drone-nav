#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

// Identity of the world a navigation decision was made against. It names the
// exact raw observation and derived ESDF the decision may claim, so it is world
// evidence rather than route or execution state.
struct NavigationWorldCertificate3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t esdf_fingerprint{0U};
  std::uint64_t esdf_source_raw_revision{0U};
  std::uint64_t esdf_source_occupied_fingerprint{0U};
  std::uint64_t raw_validated_through_revision{0U};
  std::uint64_t local_world_generation{0U};
  std::uint64_t topology_revision{0U};

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

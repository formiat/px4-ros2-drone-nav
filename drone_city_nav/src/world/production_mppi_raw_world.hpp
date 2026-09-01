#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/versioned_world_evidence_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiRawWorldMetadata3D {
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double reconstruction_ms{0.0};
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  bool full_reset{false};

  [[nodiscard]] bool valid() const noexcept {
    return source_stamp_ns > 0 && receive_stamp_ns > 0 && ready_stamp_ns > 0 &&
           std::isfinite(reconstruction_ms) && reconstruction_ms >= 0.0;
  }
};

// One closed publication of observed raw occupancy. Version and occupancy are
// derived from the sole authoritative owner, so mismatched identities cannot
// be represented after construction.
class ProductionMppiRawWorld3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const ProductionMppiRawWorld3D>
  capture(std::shared_ptr<const VersionedObservedRawWorld3D> authoritative_owner,
          ProductionMppiRawWorldMetadata3D metadata);

  ProductionMppiRawWorld3D(const ProductionMppiRawWorld3D&) = delete;
  ProductionMppiRawWorld3D& operator=(const ProductionMppiRawWorld3D&) = delete;
  ProductionMppiRawWorld3D(ProductionMppiRawWorld3D&&) = delete;
  ProductionMppiRawWorld3D& operator=(ProductionMppiRawWorld3D&&) = delete;
  ~ProductionMppiRawWorld3D() = default;

  [[nodiscard]] const RawMapVersion& version() const noexcept;
  [[nodiscard]] const ObservedOccupancyGrid3D& occupancy() const noexcept;
  [[nodiscard]] const std::shared_ptr<const ObservedOccupancyGrid3D>&
  occupancyOwner() const noexcept;
  [[nodiscard]] const std::shared_ptr<const VersionedObservedRawWorld3D>&
  authoritativeOwner() const noexcept;
  [[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D> deriveRouteEvidence(
      std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
      std::optional<LaunchSupportContact3D> launch_support_contact) const;
  [[nodiscard]] bool
  ownsRouteEvidence(const VersionedObservedRawWorld3D& evidence) const noexcept;

  [[nodiscard]] std::int64_t sourceStampNs() const noexcept;
  [[nodiscard]] std::int64_t receiveStampNs() const noexcept;
  [[nodiscard]] std::int64_t readyStampNs() const noexcept;
  [[nodiscard]] double reconstructionMs() const noexcept;
  [[nodiscard]] const std::vector<OccupancyChunkIndex3D>& dirtyChunks() const noexcept;
  [[nodiscard]] bool fullReset() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  ProductionMppiRawWorld3D(
      CaptureToken,
      std::shared_ptr<const VersionedObservedRawWorld3D> authoritative_owner,
      ProductionMppiRawWorldMetadata3D metadata);

private:
  std::shared_ptr<const VersionedObservedRawWorld3D> authoritative_owner_;
  ProductionMppiRawWorldMetadata3D metadata_{};
};

struct ProductionMppiPendingRawWorldUpdate {
  std::uint64_t authority_generation{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t announced_sequence{0U};
  std::int64_t minimum_source_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    return authority_generation != 0U && producer_instance_id != 0U &&
           announced_sequence != 0U && minimum_source_stamp_ns > 0;
  }

  [[nodiscard]] bool
  satisfiedBy(const ProducerEvidenceAdmissionState& evidence,
              const RawMapVersion& committed_version) const noexcept {
    return valid() && !evidence.current_identity_conflicted &&
           evidence.authority_generation == authority_generation &&
           evidence.producer_instance_id == producer_instance_id &&
           evidence.sequence >= announced_sequence &&
           evidence.source_stamp_ns >= minimum_source_stamp_ns &&
           committed_version.producer_instance_id == producer_instance_id &&
           committed_version.revision == evidence.sequence;
  }
};

[[nodiscard]] inline double
committedRawWorldAgeMs(const ProductionMppiRawWorld3D* world,
                       const std::int64_t now_ns) noexcept {
  if (world == nullptr || !world->valid() || now_ns <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const auto absolute_age = [now_ns](const std::int64_t stamp_ns) noexcept {
    return now_ns >= stamp_ns ? now_ns - stamp_ns : stamp_ns - now_ns;
  };
  return static_cast<double>(std::max(absolute_age(world->sourceStampNs()),
                                      absolute_age(world->receiveStampNs()))) *
         1.0e-6;
}

} // namespace drone_city_nav

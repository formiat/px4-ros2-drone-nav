#pragma once

#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"
#include "drone_city_nav/producer_epoch_admission.hpp"

#include <std_msgs/msg/header.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RawObstacleGridState3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t base_snapshot_revision{0U};
  std::uint64_t obstacle_snapshot_revision{0U};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy;
};

enum class RawObstacleGridUpdateStatus3D : std::uint8_t {
  kAccepted,
  kIdempotentReplay,
  kProducerEpochPending,
  kProducerEpochUnavailable,
  kProducerEpochCapacity,
  kIdentityConflict,
  kStale,
  kBaseUnavailable,
  kInvalidMessage,
};

struct RawObstacleGridUpdate3D {
  RawObstacleGridState3D state{};
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  RawObstacleGridUpdateStatus3D status{RawObstacleGridUpdateStatus3D::kInvalidMessage};
  bool full_reset{false};
  ProducerEvidenceAdmissionStatus evidence_status{
      ProducerEvidenceAdmissionStatus::kRejectedInvalid};
  ProducerEpochObservation evidence_observation{};
  std::uint64_t authority_generation{0U};
  bool current_identity_conflict{false};
  bool authority_handoff{false};

  [[nodiscard]] bool accepted() const noexcept {
    return status == RawObstacleGridUpdateStatus3D::kAccepted;
  }
};

[[nodiscard]] std::uint64_t rawObstacleSnapshot3DWireFingerprint(
    const msg::RawObstacleSnapshot3D& snapshot) noexcept;

[[nodiscard]] std::uint64_t
rawObstacleDelta3DWireFingerprint(const msg::RawObstacleDelta3D& delta) noexcept;

[[nodiscard]] msg::RawObstacleSnapshot3D
makeRawObstacleSnapshot3D(const ObservedOccupancyGrid3D& grid,
                          const std_msgs::msg::Header& header,
                          std::uint64_t producer_instance_id, std::uint64_t revision);

[[nodiscard]] msg::RawObstacleDelta3D makeRawObstacleDelta3D(
    const ObservedOccupancyGrid3D& grid, const std_msgs::msg::Header& header,
    std::uint64_t producer_instance_id, std::uint64_t base_snapshot_revision,
    std::uint64_t revision, std::span<const OccupancyChunkIndex3D> dirty_chunks);

class RawObstacleDeltaAccumulator3D final {
public:
  [[nodiscard]] RawObstacleGridUpdate3D
  apply(const msg::RawObstacleSnapshot3D& snapshot,
        const ProducerEpochAdmissionState& producer_epoch,
        std::int64_t receive_stamp_ns, std::int64_t now_ns,
        const ProducerEpochAdmissionConfig& config,
        bool external_contract_valid = true);
  [[nodiscard]] RawObstacleGridUpdate3D
  apply(const msg::RawObstacleDelta3D& delta,
        const ProducerEpochAdmissionState& producer_epoch,
        std::int64_t receive_stamp_ns, std::int64_t now_ns,
        const ProducerEpochAdmissionConfig& config,
        bool external_contract_valid = true);
  [[nodiscard]] RawObstacleGridUpdate3D
  synchronizeProducerEpoch(const ProducerEpochAdmissionState& producer_epoch,
                           std::int64_t now_ns,
                           const ProducerEpochAdmissionConfig& config);
  [[nodiscard]] const RawObstacleGridState3D& state() const noexcept;
  [[nodiscard]] const ProducerEvidenceAdmissionState&
  evidenceAdmissionState() const noexcept;

private:
  static constexpr std::size_t kProspectiveIdentityCapacity{4U};

  struct ProspectiveIdentity {
    ProducerEpochObservation observation{};
    RawObstacleGridState3D state{};
    RawObstacleGridUpdateStatus3D terminal_status{
        RawObstacleGridUpdateStatus3D::kProducerEpochPending};
    bool full_snapshot{false};
    bool contract_valid{false};
    bool identity_conflicted{false};
  };

  void pruneProspectiveIdentities(std::uint64_t producer_instance_id,
                                  std::uint64_t sequence) noexcept;
  [[nodiscard]] std::uint64_t
  prospectiveSequenceHighWater(std::uint64_t producer_instance_id) const noexcept;

  RawObstacleGridState3D state_{};
  ProducerEvidenceAdmissionState evidence_admission_{};
  std::array<ProspectiveIdentity, kProspectiveIdentityCapacity>
      prospective_identities_{};
  std::size_t prospective_identity_count_{0U};
  bool prospective_identity_capacity_exhausted_{false};
};

[[nodiscard]] const char*
rawObstacleGridUpdateStatus3DName(RawObstacleGridUpdateStatus3D status) noexcept;

} // namespace drone_city_nav

#pragma once

#include "drone_city_nav/msg/raw_obstacle_delta.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/producer_epoch_admission.hpp"

#include <std_msgs/msg/header.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct RawObstacleGridState {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t base_snapshot_revision{0U};
  std::uint64_t obstacle_snapshot_revision{0U};
  std::uint64_t risk_policy_fingerprint{0U};
  double risk_critical_distance_m{0.0};
  double risk_preferred_distance_m{0.0};
  std::shared_ptr<const OccupancyGrid2D> occupancy;
};

enum class RawObstacleGridUpdateStatus : std::uint8_t {
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

struct RawObstacleGridUpdate {
  RawObstacleGridState state{};
  RawObstacleGridUpdateStatus status{RawObstacleGridUpdateStatus::kInvalidMessage};
  ProducerEvidenceAdmissionStatus evidence_status{
      ProducerEvidenceAdmissionStatus::kRejectedInvalid};
  ProducerEpochObservation evidence_observation{};
  std::uint64_t authority_generation{0U};
  bool current_identity_conflict{false};
  bool authority_handoff{false};

  [[nodiscard]] bool accepted() const noexcept {
    return status == RawObstacleGridUpdateStatus::kAccepted;
  }
};

[[nodiscard]] std::uint64_t
rawObstacleSnapshotWireFingerprint(const msg::RawObstacleSnapshot& snapshot) noexcept;

[[nodiscard]] std::uint64_t
rawObstacleDeltaWireFingerprint(const msg::RawObstacleDelta& delta) noexcept;

[[nodiscard]] std::vector<std::uint32_t>
rawObstacleChunkIndices(const GridBounds& bounds,
                        std::span<const std::size_t> changed_cell_indices,
                        std::uint32_t chunk_size_cells);

[[nodiscard]] std::optional<msg::RawObstacleDelta> makeRawObstacleDelta(
    const OccupancyGrid2D& grid, const std_msgs::msg::Header& header,
    std::uint64_t producer_instance_id, std::uint64_t base_snapshot_revision,
    std::uint64_t obstacle_snapshot_revision, std::uint64_t risk_policy_fingerprint,
    double risk_critical_distance_m, double risk_preferred_distance_m,
    std::uint32_t chunk_size_cells, std::span<const std::uint32_t> chunk_indices);

class RawObstacleDeltaAccumulator final {
public:
  [[nodiscard]] RawObstacleGridUpdate
  apply(const msg::RawObstacleSnapshot& snapshot,
        const ProducerEpochAdmissionState& producer_epoch,
        std::int64_t receive_stamp_ns, std::int64_t now_ns,
        const ProducerEpochAdmissionConfig& config,
        bool external_contract_valid = true);
  [[nodiscard]] RawObstacleGridUpdate
  apply(const msg::RawObstacleDelta& delta,
        const ProducerEpochAdmissionState& producer_epoch,
        std::int64_t receive_stamp_ns, std::int64_t now_ns,
        const ProducerEpochAdmissionConfig& config,
        bool external_contract_valid = true);
  [[nodiscard]] RawObstacleGridUpdate
  synchronizeProducerEpoch(const ProducerEpochAdmissionState& producer_epoch,
                           std::int64_t now_ns,
                           const ProducerEpochAdmissionConfig& config);
  [[nodiscard]] const RawObstacleGridState& state() const noexcept;
  [[nodiscard]] const ProducerEvidenceAdmissionState&
  evidenceAdmissionState() const noexcept;

private:
  static constexpr std::size_t kProspectiveIdentityCapacity{4U};

  struct ProspectiveIdentity {
    ProducerEpochObservation observation{};
    RawObstacleGridState state{};
    RawObstacleGridUpdateStatus terminal_status{
        RawObstacleGridUpdateStatus::kProducerEpochPending};
    bool full_snapshot{false};
    bool contract_valid{false};
    bool identity_conflicted{false};
  };

  void pruneProspectiveIdentities(std::uint64_t producer_instance_id,
                                  std::uint64_t sequence) noexcept;
  [[nodiscard]] std::uint64_t
  prospectiveSequenceHighWater(std::uint64_t producer_instance_id) const noexcept;

  RawObstacleGridState state_{};
  ProducerEvidenceAdmissionState evidence_admission_{};
  std::array<ProspectiveIdentity, kProspectiveIdentityCapacity>
      prospective_identities_{};
  std::size_t prospective_identity_count_{0U};
  bool prospective_identity_capacity_exhausted_{false};
};

[[nodiscard]] const char*
rawObstacleGridUpdateStatusName(RawObstacleGridUpdateStatus status) noexcept;

} // namespace drone_city_nav

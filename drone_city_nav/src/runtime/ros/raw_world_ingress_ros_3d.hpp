#pragma once

#include "drone_city_nav/msg/obstacle_memory_status.hpp"
#include "drone_city_nav/raw_obstacle_3d_ros.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "production_mppi_raw_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

struct RawWorldIngressRosConfig3D {
  ProducerEpochAdmissionConfig producer_epoch{};
  std::string frame_id;
};

struct RawWorldIngestionResult3D {
  RawObstacleGridUpdate3D update{};
  bool execution_revocation_required{false};
};

struct MemoryStatusIngestionResult3D {
  std::optional<RawObstacleGridUpdate3D> synchronized_update;
  bool execution_revocation_required{false};
};

enum class RawWorldCommitStatus3D : std::uint8_t {
  kCommitted,
  kStopped,
  kInvalidUpdate,
  kInvalidExecutionOwner,
  kSupersededEvidence,
};

[[nodiscard]] std::string_view
rawWorldCommitStatus3DName(RawWorldCommitStatus3D status) noexcept;

struct RawWorldCommitResult3D {
  RawWorldCommitStatus3D status{RawWorldCommitStatus3D::kInvalidUpdate};
  std::shared_ptr<const ProductionMppiRawWorld3D> world;
  bool replaced_pending{false};

  [[nodiscard]] bool committed() const noexcept {
    return status == RawWorldCommitStatus3D::kCommitted && world != nullptr;
  }
};

struct RawWorldIngressSnapshot3D {
  LatestObservation latest_observation{};
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  bool raw_world_identity_conflicted{false};
};

class RawWorldIngressRos3D final {
public:
  RawWorldIngressRos3D(WorldPipeline3D& world_pipeline,
                       RawWorldIngressRosConfig3D config);

  RawWorldIngressRos3D(const RawWorldIngressRos3D&) = delete;
  RawWorldIngressRos3D& operator=(const RawWorldIngressRos3D&) = delete;
  RawWorldIngressRos3D(RawWorldIngressRos3D&&) = delete;
  RawWorldIngressRos3D& operator=(RawWorldIngressRos3D&&) = delete;

  [[nodiscard]] RawWorldIngestionResult3D
  ingestRawSnapshot(const msg::RawObstacleSnapshot3D& message,
                    std::int64_t receive_stamp_ns);
  [[nodiscard]] RawWorldIngestionResult3D
  ingestRawDelta(const msg::RawObstacleDelta3D& message, std::int64_t receive_stamp_ns);
  [[nodiscard]] MemoryStatusIngestionResult3D
  ingestMemoryStatus(const msg::ObstacleMemoryStatus& message, std::int64_t now_ns);
  [[nodiscard]] RawWorldCommitResult3D
  commitRawUpdate(const RawObstacleGridUpdate3D& update, double reconstruction_ms,
                  std::int64_t ready_stamp_ns);

  [[nodiscard]] RawWorldIngressSnapshot3D snapshot() const;

private:
  [[nodiscard]] RawWorldIngestionResult3D
  finishRawIngestionLocked(RawObstacleGridUpdate3D update);
  void invalidateRawWorldLocked() noexcept;

  WorldPipeline3D& world_pipeline_;
  RawWorldIngressRosConfig3D config_{};

  mutable std::mutex mutex_;
  LatestObservationTracker latest_observation_tracker_{};
  RawObstacleDeltaAccumulator3D raw_delta_accumulator_{};
  ProductionMppiPendingRawWorldUpdate pending_raw_world_update_{};
  bool raw_world_identity_conflicted_{false};
};

} // namespace drone_city_nav

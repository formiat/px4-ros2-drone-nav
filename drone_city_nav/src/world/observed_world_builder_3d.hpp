#pragma once

#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "production_mppi_raw_world.hpp"
#include "production_world_build_telemetry_3d.hpp"

namespace drone_city_nav {

class BoundedWorkerPool;

struct ObservedWorldBuilderConfig3D {
  LocalObservedEsdfWindow3D local_window{};
  SweptFootprintConfig footprint{};
  double preferred_distance_m{6.0};
  double update_rate_hz{1.0};
  BoundedWorkerPool* worker_pool{nullptr};
};

struct ObservedWorldBuildHistory3D {
  std::chrono::steady_clock::time_point last_build_time{};
  std::uint64_t completed_builds{0U};
};

struct ObservedWorldBuildRequest3D {
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  Point3 position{};
  std::uint64_t pose_revision{0U};
  std::int64_t ready_stamp_ns{0};
  std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  bool launch_support_resolution_pending{false};
  std::chrono::steady_clock::time_point build_started_at{};
};

enum class ObservedWorldUpdateStatus3D : std::uint8_t {
  kPrepared,
  kPublished,
  kAlreadyCurrent,
  kRateLimited,
  kUnavailableObservedGrid,
  kRawExecutionOwnerMismatch,
  kRouteEvidenceDerivationFailed,
  kInvalidCoverage,
  kSupersededTransientEvidenceParent,
  kSupersededEsdfParent,
  kUploadRejected,
  kMixedLocalWorldGeneration,
};

[[nodiscard]] std::string_view
observedWorldUpdateStatus3DName(ObservedWorldUpdateStatus3D status) noexcept;

struct ObservedWorldEvidenceChange3D {
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  bool launch_support_resolution_pending{false};
  bool persistent_changed{false};
  bool transient_changed{false};

  [[nodiscard]] bool changed() const noexcept {
    return persistent_changed || transient_changed;
  }
};

// Cheap, immutable decision captured before any expensive ESDF work. The world
// pipeline emits its evidence event before materializing this transaction so a
// route search based on obsolete launch-support evidence can be cancelled
// promptly.
struct ObservedWorldBuildAssessment3D {
  ObservedWorldUpdateStatus3D status{
      ObservedWorldUpdateStatus3D::kUnavailableObservedGrid};
  ObservedWorldBuildRequest3D request{};
  std::shared_ptr<const WorldSnapshot3D> active_world;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner;
  GridBounds3D local_bounds{};
  std::uint64_t local_occupied_fingerprint{0U};
  double maximum_distance_m{0.0};
  std::optional<std::chrono::steady_clock::time_point> retry_not_before;
  ObservedWorldEvidenceChange3D evidence_change{};
  bool same_raw_lineage{false};
  bool recentered{false};

  [[nodiscard]] bool buildRequired() const noexcept {
    return status == ObservedWorldUpdateStatus3D::kPrepared;
  }
};

struct PreparedObservedWorldBuild3D {
  ObservedWorldUpdateStatus3D status{ObservedWorldUpdateStatus3D::kInvalidCoverage};
  ObservedWorldBuildRequest3D request{};
  WorldSnapshot3D world{};
  std::shared_ptr<const WorldSnapshot3D> expected_parent;
  RawMapVersion expected_parent_raw_version{};
  std::uint64_t expected_parent_esdf_fingerprint{0U};
  ProductionWorldBuildTelemetry3D telemetry{};
  ObservedEsdf3DBuildStats stats{};
  ObservedWorldEvidenceChange3D evidence_change{};
  double maximum_distance_m{0.0};
  bool parent_required{false};
  bool upload_required{false};
  bool recentered{false};

  [[nodiscard]] bool valid() const noexcept {
    return status == ObservedWorldUpdateStatus3D::kPrepared &&
           world.distances_m != nullptr;
  }
};

class ObservedWorldBuilder3D final {
public:
  explicit ObservedWorldBuilder3D(const ObservedWorldBuilderConfig3D& config);

  ObservedWorldBuilder3D(const ObservedWorldBuilder3D&) = delete;
  ObservedWorldBuilder3D& operator=(const ObservedWorldBuilder3D&) = delete;
  ObservedWorldBuilder3D(ObservedWorldBuilder3D&&) = delete;
  ObservedWorldBuilder3D& operator=(ObservedWorldBuilder3D&&) = delete;

  [[nodiscard]] static ObservedWorldUpdateStatus3D
  assessRawWorld(const ProductionMppiRawWorld3D& raw_world) noexcept;

  [[nodiscard]] bool
  needsRefresh(const ProductionMppiRawWorld3D& raw_world,
               const std::shared_ptr<const WorldSnapshot3D>& resident_world,
               const Point3& position) const noexcept;

  [[nodiscard]] ObservedWorldBuildAssessment3D
  assess(ObservedWorldBuildRequest3D request,
         std::shared_ptr<const WorldSnapshot3D> active_world,
         const ObservedWorldBuildHistory3D& history) const;

  [[nodiscard]] PreparedObservedWorldBuild3D
  materialize(const ObservedWorldBuildAssessment3D& assessment) const;

private:
  ObservedWorldBuilderConfig3D config_{};
};

} // namespace drone_city_nav

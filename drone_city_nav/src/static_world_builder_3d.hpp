#pragma once

#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

class BoundedWorkerPool;

enum class StaticWorldRefreshPurpose3D : std::uint8_t {
  kRouteExtension,
  kTrackingObjective,
};

struct StaticWorldRefreshRequest3D {
  std::uint64_t sequence{0U};
  std::uint64_t base_route_generation{0U};
  StaticWorldRefreshPurpose3D purpose{StaticWorldRefreshPurpose3D::kRouteExtension};

  [[nodiscard]] bool valid() const noexcept {
    return sequence != 0U && base_route_generation != 0U;
  }
};

struct StaticWorldObjective3D {
  Point3 goal{};
  std::uint64_t mission_epoch{0U};
  std::uint64_t sample_sequence{0U};
  std::uint64_t assignment_generation{0U};
  std::uint64_t target_detection_id{0U};
  std::uint64_t target_track_id{0U};
  bool continuous_tracking{false};
  bool available{false};
};

struct StaticWorldBuildRequest3D {
  StaticWorldRefreshRequest3D refresh{};
  StaticWorldObjective3D objective{};
  Point3 position{};
  std::uint64_t resident_route_generation{0U};
  std::int64_t source_stamp_ns{0};
  bool world_state_authoritative{false};
};

struct StaticWorldCommitContext3D {
  Point3 position{};
  std::uint64_t pose_revision{0U};
  std::uint64_t resident_route_generation{0U};
  std::int64_t ready_stamp_ns{0};
  bool navigation_valid{false};

  [[nodiscard]] bool valid() const noexcept {
    return navigation_valid && pose_revision != 0U && ready_stamp_ns > 0 &&
           std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(position.z);
  }
};

struct StaticWorldResources3D {
  std::shared_ptr<const OccupancyGrid3D> occupancy;
  std::shared_ptr<const FreeSpaceTopology3D> topology;
  std::optional<StaticEsdfCache> esdf_cache;
};

struct StaticWorldBuilderConfig3D {
  StaticWorldResources3D resources{};
  double route_lookahead_m{180.0};
  double roi_halo_m{40.0};
  double maximum_distance_m{26.0};
  BoundedWorkerPool* worker_pool{nullptr};
};

enum class StaticWorldUpdateStatus3D : std::uint8_t {
  kPrepared,
  kPublished,
  kAlreadyCurrent,
  kUnavailableStaticOccupancy,
  kUnavailableNavigation,
  kUnavailableObjective,
  kRefreshSuperseded,
  kInvalidCommitContext,
  kConstructionFailed,
  kUploadRejected,
  kUploadFailed,
  kMixedLocalWorldGeneration,
};

[[nodiscard]] std::string_view
staticWorldUpdateStatus3DName(StaticWorldUpdateStatus3D status) noexcept;

enum class StaticWorldEsdfSource3D : std::uint8_t {
  kPrecomputedCache,
  kRuntimeEdt,
};

[[nodiscard]] std::string_view
staticWorldEsdfSource3DName(StaticWorldEsdfSource3D source) noexcept;

struct StaticWorldBuildDiagnostics3D {
  DistanceField3DBuildStats field{};
  StaticEsdfCacheExtractionStats cache{};
  StaticWorldEsdfSource3D source{StaticWorldEsdfSource3D::kRuntimeEdt};
  std::string cache_fallback_error;
  double maximum_distance_m{0.0};
  bool cpu_resource_reused{false};
  bool gpu_resource_reused{false};
};

struct StaticWorldEsdfArtifact3D {
  EsdfGrid3D grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  ProductionWorldBuildTelemetry3D telemetry{};
  StaticWorldBuildDiagnostics3D diagnostics{};
};

struct PreparedStaticWorldBuild3D {
  StaticWorldUpdateStatus3D status{
      StaticWorldUpdateStatus3D::kUnavailableStaticOccupancy};
  StaticWorldBuildRequest3D request{};
  WorldSnapshot3D world{};
  std::shared_ptr<const StaticWorldEsdfArtifact3D> artifact;
  ProductionWorldBuildTelemetry3D telemetry{};
  StaticWorldBuildDiagnostics3D diagnostics{};
  bool proactive_refresh{false};
  bool upload_required{false};

  [[nodiscard]] bool valid() const noexcept {
    return status == StaticWorldUpdateStatus3D::kPrepared && artifact != nullptr &&
           world.distances_m != nullptr;
  }
};

class StaticWorldBuilder3D final {
public:
  explicit StaticWorldBuilder3D(StaticWorldBuilderConfig3D&& config);

  StaticWorldBuilder3D(const StaticWorldBuilder3D&) = delete;
  StaticWorldBuilder3D& operator=(const StaticWorldBuilder3D&) = delete;
  StaticWorldBuilder3D(StaticWorldBuilder3D&&) = delete;
  StaticWorldBuilder3D& operator=(StaticWorldBuilder3D&&) = delete;

  [[nodiscard]] const std::shared_ptr<const OccupancyGrid3D>&
  occupancy() const noexcept;

  [[nodiscard]] PreparedStaticWorldBuild3D
  prepare(const StaticWorldBuildRequest3D& request,
          const std::shared_ptr<const WorldSnapshot3D>& active_world,
          const std::shared_ptr<const StaticWorldEsdfArtifact3D>& uploaded_artifact);

private:
  StaticWorldBuilderConfig3D config_{};
  std::shared_ptr<const std::vector<PassageTraversalEdge>> topology_traversals_;
  std::shared_ptr<const StaticWorldEsdfArtifact3D> current_artifact_;
};

} // namespace drone_city_nav

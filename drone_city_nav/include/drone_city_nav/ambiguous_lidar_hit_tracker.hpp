#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

namespace drone_city_nav {

struct UncertainLidarHitTrackerConfig {
  std::size_t required_independent_scans{3U};
  std::int64_t max_scan_gap_ns{500'000'000};
  std::int64_t retention_ns{2'000'000'000};
  double endpoint_voxel_size_m{0.5};
  double min_viewpoint_translation_m{0.5};
  double min_viewpoint_direction_change_rad{0.0523598776};
};

enum class UncertainLidarHitKind : std::uint8_t {
  kNone,
  kKnownStaticBoundary,
  kGroundCandidate,
  kProjectionUncertainUnknown,
};

enum class UncertainLidarHitEvidence : std::uint8_t {
  kExpectedSurfaceAttached,
  kDetachedObstacle,
};

enum class UncertainLidarHitResolution : std::uint8_t {
  kPending,
  kConfirmedExpectedSurface,
  kConfirmedObstacle,
};

struct UncertainLidarHitObservation {
  UncertainLidarHitKind kind{UncertainLidarHitKind::kNone};
  UncertainLidarHitEvidence evidence{UncertainLidarHitEvidence::kDetachedObstacle};
  std::string_view association_id;
  std::string_view part_id;
  Point3 endpoint_map_m{};
  Point3 ray_origin_map_m{};
  Point3 ray_direction_map{};
  double endpoint_surface_distance_m{0.0};
  double distance_before_surface_m{0.0};
  double range_residual_m{0.0};
  std::int64_t scan_stamp_ns{0};
};

struct UncertainLidarHitConfirmation {
  std::size_t independent_scans{0U};
  std::size_t expected_surface_observations{0U};
  std::size_t detached_obstacle_observations{0U};
  std::size_t expired_candidates{0U};
  double viewpoint_translation_m{0.0};
  double viewpoint_direction_change_rad{0.0};
  UncertainLidarHitResolution resolution{UncertainLidarHitResolution::kPending};
  bool new_scan_vote{false};
};

class UncertainLidarHitTracker {
public:
  explicit UncertainLidarHitTracker(const UncertainLidarHitTrackerConfig& config = {});

  void configure(const UncertainLidarHitTrackerConfig& config);
  [[nodiscard]] UncertainLidarHitConfirmation
  observe(const UncertainLidarHitObservation& observation);
  [[nodiscard]] std::size_t expire(std::int64_t scan_stamp_ns);
  void clear() noexcept;
  [[nodiscard]] std::size_t candidateCount() const noexcept;

private:
  struct Key {
    UncertainLidarHitKind kind{UncertainLidarHitKind::kNone};
    std::string association_id;
    std::string part_id;
    int voxel_x{0};
    int voxel_y{0};
    int voxel_z{0};

    bool operator==(const Key&) const = default;
  };

  struct KeyHash {
    [[nodiscard]] std::size_t operator()(const Key& key) const noexcept;
  };

  struct Evidence {
    std::int64_t first_scan_stamp_ns{0};
    std::int64_t last_scan_stamp_ns{0};
    std::size_t independent_scans{0U};
    std::size_t expected_surface_observations{0U};
    std::size_t detached_obstacle_observations{0U};
    std::size_t consecutive_expected_surface_observations{0U};
    std::size_t consecutive_detached_obstacle_observations{0U};
    Point3 last_endpoint_map_m{};
    Point3 last_ray_origin_map_m{};
    Point3 last_ray_direction_map{};
    double min_endpoint_surface_distance_m{0.0};
    double max_endpoint_surface_distance_m{0.0};
    double last_distance_before_surface_m{0.0};
    double last_range_residual_m{0.0};
  };

  [[nodiscard]] Key baseKeyFor(const UncertainLidarHitObservation& observation) const;
  [[nodiscard]] Key matchingKeyFor(const UncertainLidarHitObservation& observation,
                                   const Key& base_key) const;

  UncertainLidarHitTrackerConfig config_{};
  std::unordered_map<Key, Evidence, KeyHash> evidence_;
};

// Keep existing configuration APIs source-compatible while the tracker now
// handles all pre-grid uncertain lidar hypotheses, not only known-static hits.
using AmbiguousLidarHitTrackerConfig = UncertainLidarHitTrackerConfig;
using AmbiguousLidarHitTracker = UncertainLidarHitTracker;

[[nodiscard]] const char*
uncertainLidarHitKindName(UncertainLidarHitKind kind) noexcept;

[[nodiscard]] const char*
uncertainLidarHitEvidenceName(UncertainLidarHitEvidence evidence) noexcept;

[[nodiscard]] const char*
uncertainLidarHitResolutionName(UncertainLidarHitResolution resolution) noexcept;

} // namespace drone_city_nav

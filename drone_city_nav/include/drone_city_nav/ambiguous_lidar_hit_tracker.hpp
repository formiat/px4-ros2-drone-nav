#pragma once

#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

namespace drone_city_nav {

struct AmbiguousLidarHitTrackerConfig {
  std::size_t required_independent_scans{3U};
  std::int64_t max_scan_gap_ns{500'000'000};
  std::int64_t retention_ns{2'000'000'000};
  double endpoint_voxel_size_m{0.5};
  double min_viewpoint_translation_m{0.5};
  double min_viewpoint_direction_change_rad{0.0523598776};
};

enum class AmbiguousLidarHitResolution {
  kPending,
  kConfirmedStaticAttached,
  kConfirmedDetachedObstacle,
};

struct AmbiguousStaticHitObservation {
  std::string_view structure_id;
  std::string_view part_id;
  Point3 endpoint_map_m{};
  Point3 ray_origin_map_m{};
  Point3 ray_direction_map{};
  KnownStaticEndpointRelation endpoint_relation{KnownStaticEndpointRelation::kOutside};
  double endpoint_solid_distance_m{0.0};
  double distance_before_solid_m{0.0};
  double range_residual_m{0.0};
  std::int64_t scan_stamp_ns{0};
};

struct AmbiguousLidarHitConfirmation {
  std::size_t independent_scans{0U};
  std::size_t static_attached_observations{0U};
  std::size_t detached_obstacle_observations{0U};
  std::size_t expired_candidates{0U};
  double viewpoint_translation_m{0.0};
  double viewpoint_direction_change_rad{0.0};
  AmbiguousLidarHitResolution resolution{AmbiguousLidarHitResolution::kPending};
  bool new_scan_vote{false};
  bool opening_boundary_observed{false};
};

class AmbiguousLidarHitTracker {
public:
  explicit AmbiguousLidarHitTracker(const AmbiguousLidarHitTrackerConfig& config = {});

  void configure(const AmbiguousLidarHitTrackerConfig& config);
  [[nodiscard]] AmbiguousLidarHitConfirmation
  observe(const AmbiguousStaticHitObservation& observation);
  void clear() noexcept;
  [[nodiscard]] std::size_t candidateCount() const noexcept;

private:
  struct Key {
    std::string structure_id;
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
    std::size_t static_attached_observations{0U};
    std::size_t detached_obstacle_observations{0U};
    std::size_t consecutive_static_attached_observations{0U};
    std::size_t consecutive_detached_obstacle_observations{0U};
    bool opening_boundary_observed{false};
    Point3 last_endpoint_map_m{};
    Point3 last_ray_origin_map_m{};
    Point3 last_ray_direction_map{};
    double min_endpoint_solid_distance_m{0.0};
    double max_endpoint_solid_distance_m{0.0};
    double last_distance_before_solid_m{0.0};
    double last_range_residual_m{0.0};
  };

  [[nodiscard]] Key keyFor(const AmbiguousStaticHitObservation& observation) const;
  [[nodiscard]] std::size_t prune(std::int64_t scan_stamp_ns);

  AmbiguousLidarHitTrackerConfig config_{};
  std::unordered_map<Key, Evidence, KeyHash> evidence_;
};

[[nodiscard]] const char*
ambiguousLidarHitResolutionName(AmbiguousLidarHitResolution resolution) noexcept;

} // namespace drone_city_nav

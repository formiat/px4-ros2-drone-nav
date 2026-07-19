#pragma once

#include "drone_city_nav/known_passage_solid_volumes.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class KnownStaticLidarHitClassification {
  kExpectedStatic,
  kUnexpected,
  kAmbiguous,
};

enum class KnownStaticEndpointRelation {
  kOutside,
  kNearSurface,
  kInsideSolid,
  kInsideOpeningBoundary,
  kInsideOpening,
};

struct KnownStaticLidarHitClassifierConfig {
  // Keep a tighter limit for a hit before a known surface so an unknown object
  // in front of the building remains obstacle evidence.
  double closer_range_tolerance_m{0.5};
  // Gazebo collision and projection timing can place a known-surface return
  // slightly behind its analytic intersection.
  double farther_range_tolerance_m{1.5};
  double endpoint_volume_tolerance_m{0.75};
  double opening_boundary_tolerance_m{0.15};
};

struct KnownStaticLidarHitResult {
  KnownStaticLidarHitClassification classification{
      KnownStaticLidarHitClassification::kAmbiguous};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
  KnownPassageSolidPartKind part_kind{KnownPassageSolidPartKind::kLeft};
  std::string_view structure_id;
  std::string_view opening_id;
  std::string_view part_id;
  bool volume_matched{false};
  bool confident_face_interior{false};
  bool endpoint_volume_fallback{false};
  KnownStaticEndpointRelation endpoint_relation{KnownStaticEndpointRelation::kOutside};
  double endpoint_solid_distance_m{std::numeric_limits<double>::infinity()};
  double endpoint_opening_margin_m{-std::numeric_limits<double>::infinity()};
  double opening_min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double opening_max_z_m{std::numeric_limits<double>::quiet_NaN()};
  double opening_boundary_tolerance_m{std::numeric_limits<double>::quiet_NaN()};
  double distance_before_solid_m{std::numeric_limits<double>::quiet_NaN()};
  double incidence_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  bool closer_side_fallback{false};
};

struct KnownStaticExpectedSurface {
  double range_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_range_m{std::numeric_limits<double>::quiet_NaN()};
  Point3 intersection_map_m{};
  KnownPassageSolidPartKind part_kind{KnownPassageSolidPartKind::kLeft};
  std::string_view structure_id;
  std::string_view opening_id;
  std::string_view part_id;
  Point2 volume_center{};
  Point2 volume_normal_xy{};
  Point2 volume_lateral_xy{};
  double volume_depth_m{std::numeric_limits<double>::quiet_NaN()};
  double volume_width_m{std::numeric_limits<double>::quiet_NaN()};
  double volume_min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double volume_max_z_m{std::numeric_limits<double>::quiet_NaN()};
  double incidence_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  bool confident_face_interior{false};
};

struct KnownStaticLidarHitDiagnostic {
  bool available{false};
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
};

struct KnownStaticLidarHitProvenance {
  KnownStaticLidarHitClassification classification{
      KnownStaticLidarHitClassification::kAmbiguous};
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  int cell_x{-1};
  int cell_y{-1};
  Point3 endpoint_map_m{};
  double measured_range_m{std::numeric_limits<double>::quiet_NaN()};
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
};

struct KnownStaticLidarPartCounters {
  std::size_t left{0U};
  std::size_t right{0U};
  std::size_t lower{0U};
  std::size_t upper{0U};
};

struct KnownStaticLidarHitStats {
  std::size_t expected_static_hits_ignored{0U};
  std::size_t endpoint_volume_fallback_hits_ignored{0U};
  std::size_t unexpected_hits_kept{0U};
  std::size_t ambiguous_hits_kept{0U};
  KnownStaticLidarPartCounters expected_static_by_part{};
  KnownStaticLidarHitDiagnostic first_ignored;
  KnownStaticLidarHitDiagnostic first_ambiguous;
};

struct KnownStaticBeamEvaluation {
  std::optional<KnownStaticExpectedSurface> in_range_surface;
  std::optional<KnownStaticExpectedSurface> endpoint_fallback_surface;
  KnownStaticEndpointRelation endpoint_relation{KnownStaticEndpointRelation::kOutside};
  KnownStaticLidarHitResult hit_result{};
};

class KnownStaticLidarHitClassifier {
public:
  KnownStaticLidarHitClassifier(std::vector<KnownPassageSolidVolume> volumes,
                                const KnownStaticLidarHitClassifierConfig& config = {});

  [[nodiscard]] KnownStaticLidarHitResult
  classify(const Point3& ray_origin_map_m, const Point3& ray_direction_map,
           double measured_range_m, double effective_max_range_m) const noexcept;

  [[nodiscard]] KnownStaticBeamEvaluation
  evaluateBeam(const Point3& ray_origin_map_m, const Point3& ray_direction_map,
               double measured_range_m, double effective_max_range_m) const noexcept;

  [[nodiscard]] std::optional<KnownStaticExpectedSurface>
  nearestExpectedSurface(const Point3& ray_origin_map_m,
                         const Point3& ray_direction_map,
                         double max_range_m) const noexcept;

  [[nodiscard]] std::size_t volumeCount() const noexcept;
  [[nodiscard]] double closerRangeToleranceM() const noexcept;
  [[nodiscard]] double fartherRangeToleranceM() const noexcept;
  [[nodiscard]] double endpointVolumeToleranceM() const noexcept;
  [[nodiscard]] double openingBoundaryToleranceM() const noexcept;

private:
  std::vector<KnownPassageSolidVolume> volumes_;
  KnownStaticLidarHitClassifierConfig config_{};
};

void recordKnownStaticLidarHit(const KnownStaticLidarHitResult& result,
                               KnownStaticLidarHitStats& stats, bool retained = true);

[[nodiscard]] std::optional<KnownStaticLidarHitProvenance>
makeKnownStaticLidarHitProvenance(const KnownStaticLidarHitResult& result,
                                  const Point3& endpoint_map_m, int cell_x, int cell_y);

[[nodiscard]] const char* knownStaticLidarHitClassificationName(
    KnownStaticLidarHitClassification classification) noexcept;

[[nodiscard]] const char*
knownStaticEndpointRelationName(KnownStaticEndpointRelation relation) noexcept;

} // namespace drone_city_nav

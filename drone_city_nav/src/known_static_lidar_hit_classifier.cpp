#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"

#include <utility>

namespace drone_city_nav {

KnownStaticLidarHitClassifier::KnownStaticLidarHitClassifier(
    const KnownStaticLidarHitClassifierConfig& config)
    : config_{config} {
}

KnownStaticLidarHitResult KnownStaticLidarHitClassifier::classify(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double measured_range_m, const double effective_max_range_m) const noexcept {
  static_cast<void>(ray_origin_map_m);
  static_cast<void>(ray_direction_map);
  static_cast<void>(measured_range_m);
  static_cast<void>(effective_max_range_m);
  return {};
}

KnownStaticBeamEvaluation KnownStaticLidarHitClassifier::evaluateBeam(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double measured_range_m, const double effective_max_range_m) const noexcept {
  static_cast<void>(ray_origin_map_m);
  static_cast<void>(ray_direction_map);
  static_cast<void>(measured_range_m);
  static_cast<void>(effective_max_range_m);
  return {};
}

std::optional<KnownStaticExpectedSurface>
KnownStaticLidarHitClassifier::nearestExpectedSurface(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double max_range_m) const noexcept {
  static_cast<void>(ray_origin_map_m);
  static_cast<void>(ray_direction_map);
  static_cast<void>(max_range_m);
  return std::nullopt;
}

std::size_t KnownStaticLidarHitClassifier::volumeCount() const noexcept {
  return 0U;
}

double KnownStaticLidarHitClassifier::closerRangeToleranceM() const noexcept {
  return config_.closer_range_tolerance_m;
}

double KnownStaticLidarHitClassifier::fartherRangeToleranceM() const noexcept {
  return config_.farther_range_tolerance_m;
}

double KnownStaticLidarHitClassifier::endpointVolumeToleranceM() const noexcept {
  return config_.endpoint_volume_tolerance_m;
}

double KnownStaticLidarHitClassifier::openingBoundaryToleranceM() const noexcept {
  return config_.opening_boundary_tolerance_m;
}

void recordKnownStaticLidarHit(const KnownStaticLidarHitResult& result,
                               KnownStaticLidarHitStats& stats, const bool retained) {
  switch (result.classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
      if (retained) {
        ++stats.expected_static_hits_ignored;
      }
      break;
    case KnownStaticLidarHitClassification::kUnexpected:
      ++stats.unexpected_hits_kept;
      break;
    case KnownStaticLidarHitClassification::kAmbiguous:
      ++stats.ambiguous_hits_kept;
      break;
  }
}

std::optional<KnownStaticLidarHitProvenance>
makeKnownStaticLidarHitProvenance(const KnownStaticLidarHitResult& result,
                                  const Point3& endpoint_map_m, const int cell_x,
                                  const int cell_y) {
  if (!result.volume_matched) {
    return std::nullopt;
  }
  return KnownStaticLidarHitProvenance{
      .classification = result.classification,
      .structure_id = std::string{result.structure_id},
      .opening_id = std::string{result.opening_id},
      .part_id = std::string{result.part_id},
      .cell_x = cell_x,
      .cell_y = cell_y,
      .endpoint_map_m = endpoint_map_m,
      .measured_range_m = result.expected_range_m + result.range_delta_m,
      .expected_range_m = result.expected_range_m,
      .range_delta_m = result.range_delta_m,
  };
}

const char* knownStaticLidarHitClassificationName(
    const KnownStaticLidarHitClassification classification) noexcept {
  switch (classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
      return "expected_static";
    case KnownStaticLidarHitClassification::kUnexpected:
      return "unexpected";
    case KnownStaticLidarHitClassification::kAmbiguous:
      return "ambiguous";
  }
  return "unknown";
}

const char*
knownStaticEndpointRelationName(const KnownStaticEndpointRelation relation) noexcept {
  switch (relation) {
    case KnownStaticEndpointRelation::kOutside:
      return "outside";
    case KnownStaticEndpointRelation::kNearSurface:
      return "near_surface";
    case KnownStaticEndpointRelation::kInsideSolid:
      return "inside_solid";
    case KnownStaticEndpointRelation::kInsideOpeningBoundary:
      return "inside_opening_boundary";
    case KnownStaticEndpointRelation::kInsideOpening:
      return "inside_opening";
  }
  return "unknown";
}

} // namespace drone_city_nav

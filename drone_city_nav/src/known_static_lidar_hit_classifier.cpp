#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kLinearEpsilonM = 1.0e-6;
constexpr double kDirectionEpsilon = 1.0e-9;
constexpr double kConfidentIncidenceEpsilon = 1.0e-6;
constexpr double kRangeTieEpsilonM = 1.0e-6;

struct LocalRay {
  std::array<double, 3U> origin{};
  std::array<double, 3U> direction{};
  std::array<double, 3U> half_extent{};
};

struct SolidIntersection {
  const KnownPassageSolidVolume* volume{nullptr};
  double range_m{0.0};
  double exit_range_m{0.0};
  double incidence_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  bool confident_face_interior{false};
};

struct EndpointGeometry {
  KnownStaticEndpointRelation relation{KnownStaticEndpointRelation::kOutside};
  const KnownPassageSolidVolume* opening_volume{nullptr};
  const KnownPassageSolidVolume* nearest_solid_volume{nullptr};
  double solid_distance_m{std::numeric_limits<double>::infinity()};
  double opening_margin_m{-std::numeric_limits<double>::infinity()};
};

[[nodiscard]] bool finitePoint3(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double squaredNorm(const Point3& vector) noexcept {
  return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
}

[[nodiscard]] std::array<double, 3U>
localPoint(const Point3& point, const Point2 center, const Point2 normal,
           const Point2 lateral, const double center_z_m) noexcept {
  const Point3 relative{point.x - center.x, point.y - center.y, point.z - center_z_m};
  return {relative.x * normal.x + relative.y * normal.y,
          relative.x * lateral.x + relative.y * lateral.y, relative.z};
}

[[nodiscard]] double signedDistanceToBox(const Point3& point, const Point2 center,
                                         const Point2 normal, const Point2 lateral,
                                         const double depth_m, const double width_m,
                                         const double min_z_m,
                                         const double max_z_m) noexcept {
  if (!(depth_m > 0.0) || !(width_m > 0.0) || !(max_z_m > min_z_m)) {
    return std::numeric_limits<double>::infinity();
  }
  const std::array<double, 3U> local =
      localPoint(point, center, normal, lateral, (min_z_m + max_z_m) / 2.0);
  const std::array<double, 3U> half_extent{depth_m / 2.0, width_m / 2.0,
                                           (max_z_m - min_z_m) / 2.0};
  std::array<double, 3U> outside{};
  double max_axis_distance = -std::numeric_limits<double>::infinity();
  for (std::size_t axis = 0U; axis < local.size(); ++axis) {
    const double axis_distance = std::abs(local.at(axis)) - half_extent.at(axis);
    outside.at(axis) = std::max(0.0, axis_distance);
    max_axis_distance = std::max(max_axis_distance, axis_distance);
  }
  return std::hypot(outside.at(0), outside.at(1), outside.at(2)) +
         std::min(0.0, max_axis_distance);
}

[[nodiscard]] double openingMargin(const Point3& point,
                                   const KnownPassageSolidVolume& volume) noexcept {
  if (!(volume.opening_depth_m > 0.0) || !(volume.opening_width_m > 0.0) ||
      !(volume.opening_max_z_m > volume.opening_min_z_m)) {
    return -std::numeric_limits<double>::infinity();
  }
  const std::array<double, 3U> local =
      localPoint(point, volume.opening_center, volume.normal_xy, volume.lateral_xy,
                 (volume.opening_min_z_m + volume.opening_max_z_m) / 2.0);
  return std::min({volume.opening_depth_m / 2.0 - std::abs(local.at(0)),
                   volume.opening_width_m / 2.0 - std::abs(local.at(1)),
                   (volume.opening_max_z_m - volume.opening_min_z_m) / 2.0 -
                       std::abs(local.at(2))});
}

[[nodiscard]] EndpointGeometry endpointGeometry(
    const Point3& endpoint, const std::vector<KnownPassageSolidVolume>& volumes,
    const double near_tolerance_m, const double opening_boundary_tolerance_m) noexcept {
  EndpointGeometry geometry;
  for (const KnownPassageSolidVolume& volume : volumes) {
    const double opening_margin_m = openingMargin(endpoint, volume);
    if (opening_margin_m > kLinearEpsilonM &&
        opening_margin_m > geometry.opening_margin_m) {
      geometry.opening_volume = &volume;
      geometry.opening_margin_m = opening_margin_m;
    }
  }
  for (const KnownPassageSolidVolume& volume : volumes) {
    const double distance_m = signedDistanceToBox(
        endpoint, volume.center, volume.normal_xy, volume.lateral_xy, volume.depth_m,
        volume.width_m, volume.min_z_m, volume.max_z_m);
    if (distance_m < geometry.solid_distance_m) {
      geometry.solid_distance_m = distance_m;
      geometry.nearest_solid_volume = &volume;
    }
  }
  const bool opening_and_solid_match =
      geometry.opening_volume != nullptr && geometry.nearest_solid_volume != nullptr &&
      geometry.opening_volume->structure_id ==
          geometry.nearest_solid_volume->structure_id &&
      geometry.opening_volume->opening_id == geometry.nearest_solid_volume->opening_id;
  if (geometry.solid_distance_m <= 0.0) {
    geometry.relation = KnownStaticEndpointRelation::kInsideSolid;
  } else if (opening_and_solid_match &&
             geometry.solid_distance_m <= opening_boundary_tolerance_m) {
    geometry.relation = KnownStaticEndpointRelation::kInsideOpeningBoundary;
  } else if (geometry.opening_volume != nullptr) {
    geometry.relation = KnownStaticEndpointRelation::kInsideOpening;
  } else if (geometry.solid_distance_m <= near_tolerance_m) {
    geometry.relation = KnownStaticEndpointRelation::kNearSurface;
  }
  return geometry;
}

void incrementPartCounter(const KnownPassageSolidPartKind kind,
                          KnownStaticLidarPartCounters& counters) noexcept {
  switch (kind) {
    case KnownPassageSolidPartKind::kLeft:
      ++counters.left;
      return;
    case KnownPassageSolidPartKind::kRight:
      ++counters.right;
      return;
    case KnownPassageSolidPartKind::kLower:
      ++counters.lower;
      return;
    case KnownPassageSolidPartKind::kUpper:
      ++counters.upper;
      return;
  }
}

[[nodiscard]] LocalRay toLocalRay(const Point3& origin, const Point3& direction,
                                  const KnownPassageSolidVolume& volume) noexcept {
  const double center_z_m = (volume.min_z_m + volume.max_z_m) / 2.0;
  const Point3 relative{origin.x - volume.center.x, origin.y - volume.center.y,
                        origin.z - center_z_m};
  return LocalRay{
      .origin = {relative.x * volume.normal_xy.x + relative.y * volume.normal_xy.y,
                 relative.x * volume.lateral_xy.x + relative.y * volume.lateral_xy.y,
                 relative.z},
      .direction = {direction.x * volume.normal_xy.x + direction.y * volume.normal_xy.y,
                    direction.x * volume.lateral_xy.x +
                        direction.y * volume.lateral_xy.y,
                    direction.z},
      .half_extent = {volume.depth_m / 2.0, volume.width_m / 2.0,
                      (volume.max_z_m - volume.min_z_m) / 2.0},
  };
}

[[nodiscard]] std::optional<SolidIntersection>
intersectSolid(const Point3& origin, const Point3& direction,
               const KnownPassageSolidVolume& volume) noexcept {
  const LocalRay ray = toLocalRay(origin, direction, volume);
  double t_enter = -std::numeric_limits<double>::infinity();
  double t_exit = std::numeric_limits<double>::infinity();
  std::array<bool, 3U> entering_axes{};
  bool boundary_parallel = false;

  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const double coordinate = ray.origin.at(axis);
    const double half_extent = ray.half_extent.at(axis);
    const double axis_direction = ray.direction.at(axis);
    if (!(half_extent > 0.0) || !std::isfinite(coordinate) ||
        !std::isfinite(axis_direction)) {
      return std::nullopt;
    }
    if (std::abs(axis_direction) <= kDirectionEpsilon) {
      if (coordinate < -half_extent - kLinearEpsilonM ||
          coordinate > half_extent + kLinearEpsilonM) {
        return std::nullopt;
      }
      boundary_parallel = boundary_parallel || std::abs(std::abs(coordinate) -
                                                        half_extent) <= kLinearEpsilonM;
      continue;
    }

    double near_t = (-half_extent - coordinate) / axis_direction;
    double far_t = (half_extent - coordinate) / axis_direction;
    if (near_t > far_t) {
      std::swap(near_t, far_t);
    }
    if (near_t > t_enter + kRangeTieEpsilonM) {
      t_enter = near_t;
      entering_axes.fill(false);
      entering_axes.at(axis) = true;
    } else if (std::abs(near_t - t_enter) <= kRangeTieEpsilonM) {
      entering_axes.at(axis) = true;
    }
    t_exit = std::min(t_exit, far_t);
    if (t_enter > t_exit + kRangeTieEpsilonM) {
      return std::nullopt;
    }
  }

  if (t_exit < 0.0 || !std::isfinite(t_exit)) {
    return std::nullopt;
  }
  const double range_m = std::max(0.0, t_enter);
  const bool origin_inside = t_enter < 0.0;
  const std::size_t entering_face_count = static_cast<std::size_t>(
      std::count(entering_axes.begin(), entering_axes.end(), true));
  bool entry_near_other_boundary = false;
  if (!origin_inside && std::isfinite(t_enter)) {
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      if (entering_axes.at(axis)) {
        continue;
      }
      const double entry_coordinate =
          ray.origin.at(axis) + t_enter * ray.direction.at(axis);
      entry_near_other_boundary = entry_near_other_boundary ||
                                  std::abs(std::abs(entry_coordinate) -
                                           ray.half_extent.at(axis)) <= kLinearEpsilonM;
    }
  }
  const bool positive_thickness =
      std::isfinite(t_enter) && t_exit - t_enter > kLinearEpsilonM;
  bool near_parallel_entry = false;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    near_parallel_entry = near_parallel_entry ||
                          (entering_axes.at(axis) && std::abs(ray.direction.at(axis)) <=
                                                         kConfidentIncidenceEpsilon);
  }
  const bool confident = !origin_inside && positive_thickness &&
                         entering_face_count == 1U && !boundary_parallel &&
                         !entry_near_other_boundary && !near_parallel_entry;
  double incidence_angle_rad = std::numeric_limits<double>::quiet_NaN();
  if (entering_face_count == 1U) {
    const auto entering_axis = static_cast<std::size_t>(
        std::distance(entering_axes.begin(),
                      std::find(entering_axes.begin(), entering_axes.end(), true)));
    incidence_angle_rad =
        std::acos(std::clamp(std::abs(ray.direction.at(entering_axis)), 0.0, 1.0));
  }
  return SolidIntersection{&volume, range_m, t_exit, incidence_angle_rad, confident};
}

[[nodiscard]] bool endpointInsideSurface(const Point3& endpoint,
                                         const KnownStaticExpectedSurface& surface,
                                         const double tolerance_m) noexcept {
  const Point2 delta{endpoint.x - surface.volume_center.x,
                     endpoint.y - surface.volume_center.y};
  const double normal_distance = std::abs(delta.x * surface.volume_normal_xy.x +
                                          delta.y * surface.volume_normal_xy.y);
  const double lateral_distance = std::abs(delta.x * surface.volume_lateral_xy.x +
                                           delta.y * surface.volume_lateral_xy.y);
  return normal_distance <= surface.volume_depth_m / 2.0 + tolerance_m &&
         lateral_distance <= surface.volume_width_m / 2.0 + tolerance_m &&
         endpoint.z >= surface.volume_min_z_m - tolerance_m &&
         endpoint.z <= surface.volume_max_z_m + tolerance_m;
}

void assignDiagnostic(const KnownStaticLidarHitResult& result,
                      KnownStaticLidarHitDiagnostic& diagnostic) {
  if (diagnostic.available || !result.volume_matched) {
    return;
  }
  diagnostic.available = true;
  diagnostic.structure_id = result.structure_id;
  diagnostic.opening_id = result.opening_id;
  diagnostic.part_id = result.part_id;
  diagnostic.range_delta_m = result.range_delta_m;
}

} // namespace

KnownStaticLidarHitClassifier::KnownStaticLidarHitClassifier(
    std::vector<KnownPassageSolidVolume> volumes,
    const KnownStaticLidarHitClassifierConfig& config)
    : volumes_{std::move(volumes)},
      config_{config} {
  if (!std::isfinite(config_.closer_range_tolerance_m) ||
      config_.closer_range_tolerance_m < 0.0) {
    config_.closer_range_tolerance_m = 0.5;
  }
  if (!std::isfinite(config_.farther_range_tolerance_m) ||
      config_.farther_range_tolerance_m < 0.0) {
    config_.farther_range_tolerance_m = 1.5;
  }
  if (!std::isfinite(config_.endpoint_volume_tolerance_m) ||
      config_.endpoint_volume_tolerance_m < 0.0) {
    config_.endpoint_volume_tolerance_m = 0.75;
  }
  if (!std::isfinite(config_.opening_boundary_tolerance_m) ||
      config_.opening_boundary_tolerance_m < 0.0) {
    config_.opening_boundary_tolerance_m = 0.30;
  }
}

KnownStaticLidarHitResult KnownStaticLidarHitClassifier::classify(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double measured_range_m, const double effective_max_range_m) const noexcept {
  return evaluateBeam(ray_origin_map_m, ray_direction_map, measured_range_m,
                      effective_max_range_m)
      .hit_result;
}

KnownStaticBeamEvaluation KnownStaticLidarHitClassifier::evaluateBeam(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double measured_range_m, const double effective_max_range_m) const noexcept {
  KnownStaticBeamEvaluation evaluation{};
  KnownStaticLidarHitResult result{};
  const double direction_norm_sq = squaredNorm(ray_direction_map);
  if (!finitePoint3(ray_origin_map_m) || !finitePoint3(ray_direction_map) ||
      !std::isfinite(measured_range_m) || measured_range_m < 0.0 ||
      !std::isfinite(effective_max_range_m) || effective_max_range_m <= 0.0 ||
      !std::isfinite(direction_norm_sq) || std::abs(direction_norm_sq - 1.0) > 1.0e-6 ||
      volumes_.empty()) {
    evaluation.hit_result = result;
    return evaluation;
  }
  const Point3 measured_endpoint{
      ray_origin_map_m.x + measured_range_m * ray_direction_map.x,
      ray_origin_map_m.y + measured_range_m * ray_direction_map.y,
      ray_origin_map_m.z + measured_range_m * ray_direction_map.z};
  const EndpointGeometry endpoint_geometry =
      endpointGeometry(measured_endpoint, volumes_, config_.endpoint_volume_tolerance_m,
                       config_.opening_boundary_tolerance_m);
  result.endpoint_relation = endpoint_geometry.relation;
  evaluation.endpoint_relation = endpoint_geometry.relation;
  result.endpoint_solid_distance_m = endpoint_geometry.solid_distance_m;
  result.endpoint_opening_margin_m = endpoint_geometry.opening_margin_m;
  result.opening_boundary_tolerance_m = config_.opening_boundary_tolerance_m;
  if (endpoint_geometry.opening_volume != nullptr) {
    result.opening_min_z_m = endpoint_geometry.opening_volume->opening_min_z_m;
    result.opening_max_z_m = endpoint_geometry.opening_volume->opening_max_z_m;
  }
  const KnownPassageSolidVolume* related_volume =
      endpoint_geometry.relation == KnownStaticEndpointRelation::kInsideOpening
          ? endpoint_geometry.opening_volume
          : endpoint_geometry.nearest_solid_volume;
  if (related_volume != nullptr) {
    result.part_kind = related_volume->part_kind;
    result.structure_id = related_volume->structure_id;
    result.opening_id = related_volume->opening_id;
    result.part_id = related_volume->part_id;
  }
  if (endpoint_geometry.relation == KnownStaticEndpointRelation::kInsideOpening) {
    result.classification = KnownStaticLidarHitClassification::kUnexpected;
    evaluation.hit_result = result;
    return evaluation;
  }

  evaluation.in_range_surface = nearestExpectedSurface(
      ray_origin_map_m, ray_direction_map, effective_max_range_m);
  std::optional<KnownStaticExpectedSurface> nearest = evaluation.in_range_surface;
  if (!nearest.has_value() &&
      (endpoint_geometry.relation == KnownStaticEndpointRelation::kInsideSolid ||
       endpoint_geometry.relation == KnownStaticEndpointRelation::kNearSurface ||
       endpoint_geometry.relation ==
           KnownStaticEndpointRelation::kInsideOpeningBoundary)) {
    evaluation.endpoint_fallback_surface = nearestExpectedSurface(
        ray_origin_map_m, ray_direction_map,
        effective_max_range_m + config_.endpoint_volume_tolerance_m);
    nearest = evaluation.endpoint_fallback_surface;
  }
  if (!nearest.has_value()) {
    if (endpoint_geometry.relation == KnownStaticEndpointRelation::kInsideSolid) {
      result.classification = KnownStaticLidarHitClassification::kExpectedStatic;
      result.volume_matched = true;
      result.endpoint_volume_fallback = true;
    } else if (endpoint_geometry.relation ==
                   KnownStaticEndpointRelation::kNearSurface ||
               endpoint_geometry.relation ==
                   KnownStaticEndpointRelation::kInsideOpeningBoundary) {
      result.classification = KnownStaticLidarHitClassification::kAmbiguous;
      result.volume_matched = true;
      result.endpoint_volume_fallback = true;
    } else {
      result.classification = KnownStaticLidarHitClassification::kUnexpected;
    }
    evaluation.hit_result = result;
    return evaluation;
  }

  result.expected_range_m = nearest->range_m;
  result.range_delta_m = measured_range_m - nearest->range_m;
  if (endpoint_geometry.relation !=
      KnownStaticEndpointRelation::kInsideOpeningBoundary) {
    result.part_kind = nearest->part_kind;
    result.structure_id = nearest->structure_id;
    result.opening_id = nearest->opening_id;
    result.part_id = nearest->part_id;
  }
  result.volume_matched = true;
  result.confident_face_interior = nearest->confident_face_interior;
  result.distance_before_solid_m = nearest->range_m - measured_range_m;
  result.incidence_angle_rad = nearest->incidence_angle_rad;
  if (endpoint_geometry.relation == KnownStaticEndpointRelation::kInsideSolid ||
      endpoint_geometry.relation == KnownStaticEndpointRelation::kNearSurface ||
      endpoint_geometry.relation ==
          KnownStaticEndpointRelation::kInsideOpeningBoundary) {
    result.endpoint_volume_fallback = true;
    const bool closer_outside_tolerance =
        measured_range_m < nearest->range_m - config_.closer_range_tolerance_m;
    const bool farther_outside_tolerance =
        measured_range_m > nearest->range_m + config_.farther_range_tolerance_m;
    if (endpoint_geometry.relation ==
            KnownStaticEndpointRelation::kInsideOpeningBoundary ||
        closer_outside_tolerance || !nearest->confident_face_interior ||
        (endpoint_geometry.relation == KnownStaticEndpointRelation::kNearSurface &&
         farther_outside_tolerance)) {
      result.classification = KnownStaticLidarHitClassification::kAmbiguous;
      result.closer_side_fallback = closer_outside_tolerance;
    } else {
      result.classification = KnownStaticLidarHitClassification::kExpectedStatic;
    }
    evaluation.hit_result = result;
    return evaluation;
  }
  if (!nearest->confident_face_interior) {
    evaluation.hit_result = result;
    return evaluation;
  }
  if (measured_range_m < nearest->range_m - config_.closer_range_tolerance_m) {
    result.classification = KnownStaticLidarHitClassification::kUnexpected;
  } else if (measured_range_m <= nearest->range_m + config_.farther_range_tolerance_m) {
    result.classification = KnownStaticLidarHitClassification::kExpectedStatic;
  } else {
    const bool before_or_inside_far_face =
        measured_range_m <= nearest->exit_range_m + config_.endpoint_volume_tolerance_m;
    if (before_or_inside_far_face &&
        endpointInsideSurface(measured_endpoint, *nearest,
                              config_.endpoint_volume_tolerance_m)) {
      result.classification = KnownStaticLidarHitClassification::kExpectedStatic;
      result.endpoint_volume_fallback = true;
    }
  }
  evaluation.hit_result = result;
  return evaluation;
}

std::optional<KnownStaticExpectedSurface>
KnownStaticLidarHitClassifier::nearestExpectedSurface(
    const Point3& ray_origin_map_m, const Point3& ray_direction_map,
    const double max_range_m) const noexcept {
  const double direction_norm_sq = squaredNorm(ray_direction_map);
  if (!finitePoint3(ray_origin_map_m) || !finitePoint3(ray_direction_map) ||
      !(max_range_m >= 0.0) || !std::isfinite(direction_norm_sq) ||
      std::abs(direction_norm_sq - 1.0) > 1.0e-6 || volumes_.empty()) {
    return std::nullopt;
  }

  std::optional<SolidIntersection> nearest;
  for (const KnownPassageSolidVolume& volume : volumes_) {
    const std::optional<SolidIntersection> candidate =
        intersectSolid(ray_origin_map_m, ray_direction_map, volume);
    if (!candidate.has_value() || candidate->range_m > max_range_m) {
      continue;
    }
    if (!nearest.has_value() || candidate->range_m < nearest->range_m) {
      nearest = candidate;
    }
  }
  if (!nearest.has_value()) {
    return std::nullopt;
  }
  const KnownPassageSolidVolume& volume = *nearest->volume;
  const Point3 intersection{ray_origin_map_m.x + nearest->range_m * ray_direction_map.x,
                            ray_origin_map_m.y + nearest->range_m * ray_direction_map.y,
                            ray_origin_map_m.z +
                                nearest->range_m * ray_direction_map.z};
  return KnownStaticExpectedSurface{
      .range_m = nearest->range_m,
      .exit_range_m = nearest->exit_range_m,
      .intersection_map_m = intersection,
      .part_kind = volume.part_kind,
      .structure_id = volume.structure_id,
      .opening_id = volume.opening_id,
      .part_id = volume.part_id,
      .volume_center = volume.center,
      .volume_normal_xy = volume.normal_xy,
      .volume_lateral_xy = volume.lateral_xy,
      .volume_depth_m = volume.depth_m,
      .volume_width_m = volume.width_m,
      .volume_min_z_m = volume.min_z_m,
      .volume_max_z_m = volume.max_z_m,
      .incidence_angle_rad = nearest->incidence_angle_rad,
      .confident_face_interior = nearest->confident_face_interior,
  };
}

std::size_t KnownStaticLidarHitClassifier::volumeCount() const noexcept {
  return volumes_.size();
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

std::optional<KnownStaticLidarHitProvenance>
makeKnownStaticLidarHitProvenance(const KnownStaticLidarHitResult& result,
                                  const Point3& endpoint_map_m, const int cell_x,
                                  const int cell_y) {
  if (!result.volume_matched || !finitePoint3(endpoint_map_m)) {
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

void recordKnownStaticLidarHit(const KnownStaticLidarHitResult& result,
                               KnownStaticLidarHitStats& stats, const bool retained) {
  switch (result.classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
      ++stats.expected_static_hits_ignored;
      if (result.endpoint_volume_fallback) {
        ++stats.endpoint_volume_fallback_hits_ignored;
      }
      incrementPartCounter(result.part_kind, stats.expected_static_by_part);
      assignDiagnostic(result, stats.first_ignored);
      return;
    case KnownStaticLidarHitClassification::kUnexpected:
      ++stats.unexpected_hits_kept;
      return;
    case KnownStaticLidarHitClassification::kAmbiguous:
      if (retained) {
        ++stats.ambiguous_hits_kept;
      }
      assignDiagnostic(result, stats.first_ambiguous);
      return;
  }
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

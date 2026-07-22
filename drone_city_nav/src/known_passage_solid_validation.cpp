#include "drone_city_nav/known_passage_solid_validation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kGeometryEpsilon = 1.0e-9;

struct LocalPoint3 {
  double depth{0.0};
  double lateral{0.0};
  double z{0.0};
};

[[nodiscard]] bool sampleIsFinite(const TrajectoryPointSample& sample) {
  return std::isfinite(sample.point.x) && std::isfinite(sample.point.y) &&
         std::isfinite(sample.z_m) && std::isfinite(sample.s_m);
}

[[nodiscard]] LocalPoint3 toLocal(const TrajectoryPointSample& sample,
                                  const KnownPassageSolidVolume& volume) {
  const double dx = sample.point.x - volume.center.x;
  const double dy = sample.point.y - volume.center.y;
  return LocalPoint3{dx * volume.normal_xy.x + dy * volume.normal_xy.y,
                     dx * volume.lateral_xy.x + dy * volume.lateral_xy.y, sample.z_m};
}

[[nodiscard]] bool clipAxis(const double start, const double delta,
                            const double minimum, const double maximum, double& enter_t,
                            double& exit_t) {
  if (std::abs(delta) <= kGeometryEpsilon) {
    return start >= minimum - kGeometryEpsilon && start <= maximum + kGeometryEpsilon;
  }
  double axis_enter = (minimum - start) / delta;
  double axis_exit = (maximum - start) / delta;
  if (axis_enter > axis_exit) {
    std::swap(axis_enter, axis_exit);
  }
  enter_t = std::max(enter_t, axis_enter);
  exit_t = std::min(exit_t, axis_exit);
  return enter_t <= exit_t + kGeometryEpsilon;
}

[[nodiscard]] bool segmentIntersection(const TrajectoryPointSample& start,
                                       const TrajectoryPointSample& end,
                                       const KnownPassageSolidVolume& volume,
                                       double& intersection_t) {
  const LocalPoint3 local_start = toLocal(start, volume);
  const LocalPoint3 local_end = toLocal(end, volume);
  const double half_depth = volume.depth_m * 0.5;
  const double half_width = volume.width_m * 0.5;
  double enter_t = 0.0;
  double exit_t = 1.0;
  if (!clipAxis(local_start.depth, local_end.depth - local_start.depth, -half_depth,
                half_depth, enter_t, exit_t) ||
      !clipAxis(local_start.lateral, local_end.lateral - local_start.lateral,
                -half_width, half_width, enter_t, exit_t) ||
      !clipAxis(local_start.z, local_end.z - local_start.z, volume.min_z_m,
                volume.max_z_m, enter_t, exit_t)) {
    return false;
  }
  intersection_t = std::clamp(enter_t, 0.0, 1.0);
  return true;
}

} // namespace

const char* knownPassageSolidValidationReasonName(
    const KnownPassageSolidValidationReason reason) noexcept {
  switch (reason) {
    case KnownPassageSolidValidationReason::kNoMap:
      return "no_map";
    case KnownPassageSolidValidationReason::kInvalidTrajectory:
      return "invalid_trajectory";
    case KnownPassageSolidValidationReason::kClear:
      return "clear";
    case KnownPassageSolidValidationReason::kIntersection:
      return "intersection";
  }
  return "unknown";
}

KnownPassageSolidValidationSummary validateTrajectoryAgainstKnownPassageSolids(
    const std::span<const TrajectoryPointSample> samples,
    const KnownPassageMap* const known_passage_map) {
  KnownPassageSolidValidationSummary summary{};
  if (known_passage_map == nullptr) {
    return summary;
  }
  if (samples.size() < 2U || !std::ranges::all_of(samples, sampleIsFinite)) {
    summary.valid = false;
    summary.reason = KnownPassageSolidValidationReason::kInvalidTrajectory;
    return summary;
  }

  const std::vector<KnownPassageSolidVolume> volumes =
      knownPassageSolidVolumes(*known_passage_map);
  summary.volumes_checked = volumes.size();
  summary.segments_checked = samples.size() - 1U;
  double first_s = std::numeric_limits<double>::infinity();
  for (std::size_t segment_index = 0U; segment_index + 1U < samples.size();
       ++segment_index) {
    const TrajectoryPointSample& start = samples[segment_index];
    const TrajectoryPointSample& end = samples[segment_index + 1U];
    for (const KnownPassageSolidVolume& volume : volumes) {
      double intersection_t = 0.0;
      if (!segmentIntersection(start, end, volume, intersection_t)) {
        continue;
      }
      ++summary.intersections;
      const double intersection_s = start.s_m + intersection_t * (end.s_m - start.s_m);
      if (intersection_s >= first_s) {
        continue;
      }
      first_s = intersection_s;
      summary.has_first_intersection = true;
      summary.first_intersection = KnownPassageSolidIntersection{
          volume.structure_id,
          volume.opening_id,
          volume.part_id,
          volume.part_kind,
          segment_index,
          intersection_t,
          intersection_s,
          Point3{start.point.x + intersection_t * (end.point.x - start.point.x),
                 start.point.y + intersection_t * (end.point.y - start.point.y),
                 start.z_m + intersection_t * (end.z_m - start.z_m)}};
    }
  }
  summary.valid = summary.intersections == 0U;
  summary.reason = summary.valid ? KnownPassageSolidValidationReason::kClear
                                 : KnownPassageSolidValidationReason::kIntersection;
  return summary;
}

} // namespace drone_city_nav

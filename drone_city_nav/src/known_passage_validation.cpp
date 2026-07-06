#include "drone_city_nav/known_passage_validation.hpp"

#include "drone_city_nav/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-9;

struct Point3S {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double s_m{0.0};
};

struct AxisRange {
  double min{0.0};
  double max{0.0};
};

struct Rect {
  AxisRange x{};
  AxisRange y{};
};

struct ClipInterval {
  double t0{0.0};
  double t1{0.0};
  double s0_m{0.0};
  double s1_m{0.0};
  Point3S p0{};
  Point3S p1{};
};

struct FootprintSpan {
  double entry_s_m{0.0};
  double exit_s_m{0.0};
};

struct OpeningMatch {
  std::string opening_id;
  double overlap_m{0.0};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] Rect footprintRect(const PassageStructure& structure) noexcept {
  const double half_x = structure.size_x_m / 2.0;
  const double half_y = structure.size_y_m / 2.0;
  return Rect{AxisRange{structure.center.x - half_x, structure.center.x + half_x},
              AxisRange{structure.center.y - half_y, structure.center.y + half_y}};
}

[[nodiscard]] Point3S samplePointAtT(const TrajectoryPointSample& start,
                                     const TrajectoryPointSample& end,
                                     const double t) noexcept {
  return Point3S{
      .x = start.point.x + (end.point.x - start.point.x) * t,
      .y = start.point.y + (end.point.y - start.point.y) * t,
      .z = start.z_m + (end.z_m - start.z_m) * t,
      .s_m = start.s_m + (end.s_m - start.s_m) * t,
  };
}

[[nodiscard]] bool updateLineClip(const double coordinate, const double delta,
                                  const AxisRange range, double& t0,
                                  double& t1) noexcept {
  if (std::abs(delta) <= kTinyDistanceM) {
    return coordinate >= range.min && coordinate <= range.max;
  }

  double enter = (range.min - coordinate) / delta;
  double exit = (range.max - coordinate) / delta;
  if (enter > exit) {
    std::swap(enter, exit);
  }

  t0 = std::max(t0, enter);
  t1 = std::min(t1, exit);
  return t0 <= t1;
}

[[nodiscard]] std::optional<ClipInterval>
clipSampleSegmentToFootprint(const TrajectoryPointSample& start,
                             const TrajectoryPointSample& end,
                             const PassageStructure& structure) {
  double t0 = 0.0;
  double t1 = 1.0;
  const Rect rect = footprintRect(structure);
  const double dx = end.point.x - start.point.x;
  const double dy = end.point.y - start.point.y;
  if (!updateLineClip(start.point.x, dx, rect.x, t0, t1) ||
      !updateLineClip(start.point.y, dy, rect.y, t0, t1) || t1 < 0.0 || t0 > 1.0) {
    return std::nullopt;
  }

  t0 = std::clamp(t0, 0.0, 1.0);
  t1 = std::clamp(t1, 0.0, 1.0);
  if (t1 + kTinyDistanceM < t0) {
    return std::nullopt;
  }

  ClipInterval interval{};
  interval.t0 = t0;
  interval.t1 = t1;
  interval.p0 = samplePointAtT(start, end, t0);
  interval.p1 = samplePointAtT(start, end, t1);
  interval.s0_m = interval.p0.s_m;
  interval.s1_m = interval.p1.s_m;
  if (interval.s1_m + kTinyDistanceM < interval.s0_m) {
    return std::nullopt;
  }
  return interval;
}

void mergeFootprintClip(std::vector<FootprintSpan>& spans, const ClipInterval& clip) {
  if (clip.s1_m <= clip.s0_m + kTinyDistanceM) {
    return;
  }
  if (!spans.empty() && clip.s0_m <= spans.back().exit_s_m + kTinyDistanceM) {
    spans.back().exit_s_m = std::max(spans.back().exit_s_m, clip.s1_m);
    return;
  }
  spans.push_back(FootprintSpan{.entry_s_m = clip.s0_m, .exit_s_m = clip.s1_m});
}

[[nodiscard]] std::vector<FootprintSpan>
findFootprintSpans(std::span<const TrajectoryPointSample> samples,
                   const PassageStructure& structure) {
  std::vector<FootprintSpan> spans;
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const std::optional<ClipInterval> clip =
        clipSampleSegmentToFootprint(samples[i], samples[i + 1U], structure);
    if (clip.has_value()) {
      mergeFootprintClip(spans, *clip);
    }
  }
  return spans;
}

[[nodiscard]] std::optional<ClipInterval>
clipSampleSegmentToStationRange(const TrajectoryPointSample& start,
                                const TrajectoryPointSample& end,
                                const FootprintSpan& span) {
  const double station_delta_m = end.s_m - start.s_m;
  if (!(station_delta_m > kTinyDistanceM) || end.s_m < span.entry_s_m ||
      start.s_m > span.exit_s_m) {
    return std::nullopt;
  }

  const double t0 =
      std::clamp((span.entry_s_m - start.s_m) / station_delta_m, 0.0, 1.0);
  const double t1 = std::clamp((span.exit_s_m - start.s_m) / station_delta_m, 0.0, 1.0);
  if (t1 <= t0 + kTinyDistanceM) {
    return std::nullopt;
  }

  ClipInterval interval{};
  interval.t0 = t0;
  interval.t1 = t1;
  interval.p0 = samplePointAtT(start, end, t0);
  interval.p1 = samplePointAtT(start, end, t1);
  interval.s0_m = interval.p0.s_m;
  interval.s1_m = interval.p1.s_m;
  return interval;
}

struct OpeningLocalPoint {
  double u{0.0};
  double v{0.0};
  double z{0.0};
  double s_m{0.0};
};

[[nodiscard]] OpeningLocalPoint
toOpeningLocalPoint(const Point3S& point, const PassageOpening& opening) noexcept {
  const double dx = point.x - opening.center.x;
  const double dy = point.y - opening.center.y;
  const Point2 normal = opening.normal_xy;
  const Point2 lateral{-normal.y, normal.x};
  return OpeningLocalPoint{
      .u = dx * normal.x + dy * normal.y,
      .v = dx * lateral.x + dy * lateral.y,
      .z = point.z,
      .s_m = point.s_m,
  };
}

[[nodiscard]] bool updateBoxClip(const double coordinate, const double delta,
                                 const AxisRange range, double& t0,
                                 double& t1) noexcept {
  return updateLineClip(coordinate, delta, range, t0, t1);
}

[[nodiscard]] double openingPassageClearance(const OpeningLocalPoint& point,
                                             const PassageOpening& opening) noexcept {
  const double half_width = opening.width_m / 2.0;
  return std::min({half_width - std::abs(point.v), point.z - opening.min_z_m,
                   opening.max_z_m - point.z});
}

[[nodiscard]] double signedOpeningVolumeMargin(const OpeningLocalPoint& point,
                                               const PassageOpening& opening) noexcept {
  const double half_depth = opening.depth_m / 2.0;
  return std::min(half_depth - std::abs(point.u),
                  openingPassageClearance(point, opening));
}

[[nodiscard]] std::optional<OpeningMatch>
clipStationSegmentToOpening(const ClipInterval& station_clip,
                            const PassageOpening& opening,
                            const KnownPassageValidationConfig& config) {
  const OpeningLocalPoint start = toOpeningLocalPoint(station_clip.p0, opening);
  const OpeningLocalPoint end = toOpeningLocalPoint(station_clip.p1, opening);
  double t0 = 0.0;
  double t1 = 1.0;
  const double half_depth = opening.depth_m / 2.0;
  const double half_width = opening.width_m / 2.0;
  if (!updateBoxClip(start.u, end.u - start.u, AxisRange{-half_depth, half_depth}, t0,
                     t1) ||
      !updateBoxClip(start.v, end.v - start.v, AxisRange{-half_width, half_width}, t0,
                     t1) ||
      !updateBoxClip(start.z, end.z - start.z,
                     AxisRange{opening.min_z_m, opening.max_z_m}, t0, t1)) {
    return std::nullopt;
  }
  t0 = std::clamp(t0, 0.0, 1.0);
  t1 = std::clamp(t1, 0.0, 1.0);
  if (t1 <= t0 + kTinyDistanceM) {
    return std::nullopt;
  }

  const double overlap_m = (station_clip.s1_m - station_clip.s0_m) * (t1 - t0);

  const auto localAtT = [&start, &end](const double t) noexcept {
    return OpeningLocalPoint{
        .u = start.u + (end.u - start.u) * t,
        .v = start.v + (end.v - start.v) * t,
        .z = start.z + (end.z - start.z) * t,
        .s_m = start.s_m + (end.s_m - start.s_m) * t,
    };
  };
  const double clearance_m = std::min(openingPassageClearance(localAtT(t0), opening),
                                      openingPassageClearance(localAtT(t1), opening));
  if (clearance_m + kTinyDistanceM < config.clearance_margin_m) {
    return std::nullopt;
  }

  return OpeningMatch{
      .opening_id = opening.id,
      .overlap_m = overlap_m,
      .clearance_m = clearance_m,
  };
}

void mergeOpeningMatch(std::optional<OpeningMatch>& best_match,
                       const OpeningMatch& candidate) {
  if (!best_match.has_value() || candidate.overlap_m > best_match->overlap_m ||
      (std::abs(candidate.overlap_m - best_match->overlap_m) <= kTinyDistanceM &&
       candidate.clearance_m > best_match->clearance_m)) {
    best_match = candidate;
  }
}

[[nodiscard]] double
estimateOpeningMissClearance(std::span<const TrajectoryPointSample> samples,
                             const FootprintSpan& span, const PassageOpening& opening) {
  double best_clearance_m = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const std::optional<ClipInterval> station_clip =
        clipSampleSegmentToStationRange(samples[i], samples[i + 1U], span);
    if (!station_clip.has_value()) {
      continue;
    }
    best_clearance_m = std::max(
        best_clearance_m, signedOpeningVolumeMargin(
                              toOpeningLocalPoint(station_clip->p0, opening), opening));
    best_clearance_m = std::max(
        best_clearance_m, signedOpeningVolumeMargin(
                              toOpeningLocalPoint(station_clip->p1, opening), opening));
  }
  if (!std::isfinite(best_clearance_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best_clearance_m;
}

[[nodiscard]] std::optional<OpeningMatch>
findBestOpeningMatch(std::span<const TrajectoryPointSample> samples,
                     const FootprintSpan& span, const PassageStructure& structure,
                     const KnownPassageValidationConfig& config) {
  std::optional<OpeningMatch> best_match;
  for (const PassageOpening& opening : structure.openings) {
    std::optional<OpeningMatch> opening_match;
    double overlap_sum_m = 0.0;
    double min_clearance_m = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
      const std::optional<ClipInterval> station_clip =
          clipSampleSegmentToStationRange(samples[i], samples[i + 1U], span);
      if (!station_clip.has_value()) {
        continue;
      }
      const std::optional<OpeningMatch> segment_match =
          clipStationSegmentToOpening(*station_clip, opening, config);
      if (!segment_match.has_value()) {
        continue;
      }
      overlap_sum_m += segment_match->overlap_m;
      min_clearance_m = std::min(min_clearance_m, segment_match->clearance_m);
      opening_match = OpeningMatch{
          .opening_id = opening.id,
          .overlap_m = overlap_sum_m,
          .clearance_m = min_clearance_m,
      };
    }
    if (opening_match.has_value() &&
        opening_match->overlap_m + kTinyDistanceM >= config.min_opening_overlap_m) {
      mergeOpeningMatch(best_match, *opening_match);
    }
  }
  return best_match;
}

[[nodiscard]] double bestMissClearance(std::span<const TrajectoryPointSample> samples,
                                       const FootprintSpan& span,
                                       const PassageStructure& structure) {
  double best_clearance_m = -std::numeric_limits<double>::infinity();
  for (const PassageOpening& opening : structure.openings) {
    best_clearance_m = std::max(best_clearance_m,
                                estimateOpeningMissClearance(samples, span, opening));
  }
  if (!std::isfinite(best_clearance_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best_clearance_m;
}

void appendDiagnostic(KnownPassageValidationSummary& summary,
                      const KnownPassageValidationConfig& config,
                      const KnownPassageValidationSpan& span) {
  if (summary.diagnostics.size() < config.max_diagnostics) {
    summary.diagnostics.push_back(span);
  }
}

void setWorstReason(KnownPassageValidationSummary& summary,
                    const KnownPassageValidationReason reason) noexcept {
  if (summary.valid) {
    summary.worst_reason = reason;
    return;
  }
  if (summary.worst_reason == KnownPassageValidationReason::kMatchedOpening ||
      summary.worst_reason == KnownPassageValidationReason::kNoStructureIntersection) {
    summary.worst_reason = reason;
  }
}

} // namespace

const char*
knownPassageValidationReasonName(const KnownPassageValidationReason reason) noexcept {
  switch (reason) {
    case KnownPassageValidationReason::kDisabled:
      return "disabled";
    case KnownPassageValidationReason::kNoMap:
      return "no_map";
    case KnownPassageValidationReason::kInvalidTrajectory:
      return "invalid_trajectory";
    case KnownPassageValidationReason::kNoStructureIntersection:
      return "no_structure_intersection";
    case KnownPassageValidationReason::kMatchedOpening:
      return "matched_opening";
    case KnownPassageValidationReason::kStructureWithoutOpening:
      return "structure_without_opening";
    case KnownPassageValidationReason::kOpeningVolumeMiss:
      return "opening_volume_miss";
  }
  return "unknown";
}

KnownPassageValidationSummary
validateKnownPassageTraversal(const std::span<const TrajectoryPointSample> samples,
                              const KnownPassageMap* const map,
                              const KnownPassageValidationConfig& config) {
  KnownPassageValidationSummary summary{};
  summary.enabled = config.enabled;

  if (!config.enabled) {
    summary.worst_reason = KnownPassageValidationReason::kDisabled;
    return summary;
  }
  if (map == nullptr) {
    summary.worst_reason = KnownPassageValidationReason::kNoMap;
    return summary;
  }
  summary.structures_checked = map->structures.size();
  if (!trajectorySamplesAreUsable(samples)) {
    summary.valid = false;
    summary.violations = 1U;
    summary.worst_reason = KnownPassageValidationReason::kInvalidTrajectory;
    return summary;
  }

  bool had_intersection = false;
  bool had_match = false;
  for (const PassageStructure& structure : map->structures) {
    const std::vector<FootprintSpan> spans = findFootprintSpans(samples, structure);
    if (spans.empty()) {
      continue;
    }
    had_intersection = true;
    ++summary.structures_intersected;

    for (const FootprintSpan& span : spans) {
      KnownPassageValidationSpan diagnostic{};
      diagnostic.structure_id = structure.id;
      diagnostic.entry_s_m = span.entry_s_m;
      diagnostic.exit_s_m = span.exit_s_m;

      if (structure.openings.empty()) {
        summary.valid = false;
        ++summary.violations;
        diagnostic.reason = KnownPassageValidationReason::kStructureWithoutOpening;
        diagnostic.clearance_m = std::numeric_limits<double>::quiet_NaN();
        setWorstReason(summary, diagnostic.reason);
        appendDiagnostic(summary, config, diagnostic);
        continue;
      }

      const std::optional<OpeningMatch> match =
          findBestOpeningMatch(samples, span, structure, config);
      if (match.has_value()) {
        had_match = true;
        ++summary.opening_matches;
        diagnostic.opening_id = match->opening_id;
        diagnostic.overlap_m = match->overlap_m;
        diagnostic.clearance_m = match->clearance_m;
        diagnostic.reason = KnownPassageValidationReason::kMatchedOpening;
        diagnostic.valid = true;
        appendDiagnostic(summary, config, diagnostic);
        continue;
      }

      summary.valid = false;
      ++summary.violations;
      diagnostic.reason = KnownPassageValidationReason::kOpeningVolumeMiss;
      diagnostic.clearance_m = bestMissClearance(samples, span, structure);
      setWorstReason(summary, diagnostic.reason);
      appendDiagnostic(summary, config, diagnostic);
    }
  }

  if (summary.valid) {
    summary.worst_reason = had_match
                               ? KnownPassageValidationReason::kMatchedOpening
                               : KnownPassageValidationReason::kNoStructureIntersection;
  } else if (!had_intersection) {
    summary.worst_reason = KnownPassageValidationReason::kNoStructureIntersection;
  }
  return summary;
}

} // namespace drone_city_nav

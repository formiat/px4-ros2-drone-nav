#include "drone_city_nav/trajectory_passage_insertion.hpp"

#include "drone_city_nav/known_passage_geometry.hpp"
#include "drone_city_nav/known_passage_matching.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kEndpointToleranceM = 1.0e-4;

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{1.0, 0.0};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double angleBetween(const Point2 a, const Point2 b) noexcept {
  const Point2 an = normalized(a);
  const Point2 bn = normalized(b);
  return std::acos(std::clamp(dot(an, bn), -1.0, 1.0));
}

[[nodiscard]] PassageInsertionConfig
sanitizeConfig(const PassageInsertionConfig& input) noexcept {
  PassageInsertionConfig config = input;
  config.sample_step_m = std::clamp(
      std::isfinite(config.sample_step_m) ? config.sample_step_m : 1.0, 0.1, 20.0);
  config.min_anchor_margin_m = std::clamp(
      std::isfinite(config.min_anchor_margin_m) ? config.min_anchor_margin_m : 8.0, 0.0,
      1000.0);
  config.max_anchor_margin_m = std::max(
      config.min_anchor_margin_m,
      std::clamp(std::isfinite(config.max_anchor_margin_m) ? config.max_anchor_margin_m
                                                           : 60.0,
                 0.0, 5000.0));
  config.opening_lateral_target_margin_m =
      std::clamp(std::isfinite(config.opening_lateral_target_margin_m)
                     ? config.opening_lateral_target_margin_m
                     : 0.0,
                 0.0, 1000.0);
  config.repair_clearance_margin_m =
      std::clamp(std::isfinite(config.repair_clearance_margin_m)
                     ? config.repair_clearance_margin_m
                     : config.opening_lateral_target_margin_m,
                 0.0, 1000.0);
  config.opening_lateral_target_margin_m = std::max(
      config.opening_lateral_target_margin_m, config.repair_clearance_margin_m);
  config.max_lateral_shift_m = std::clamp(
      std::isfinite(config.max_lateral_shift_m) ? config.max_lateral_shift_m : 80.0,
      0.0, 5000.0);
  config.max_join_tangent_delta_rad =
      std::clamp(std::isfinite(config.max_join_tangent_delta_rad)
                     ? config.max_join_tangent_delta_rad
                     : 0.35,
                 0.0, std::numbers::pi);
  config.max_join_curvature_jump_1pm =
      std::clamp(std::isfinite(config.max_join_curvature_jump_1pm)
                     ? config.max_join_curvature_jump_1pm
                     : 0.08,
                 0.0, 1000.0);
  config.min_inserted_radius_m = std::clamp(
      std::isfinite(config.min_inserted_radius_m) ? config.min_inserted_radius_m : 0.0,
      0.0, 100000.0);
  config.max_candidates = std::clamp<std::size_t>(config.max_candidates, 0U, 100U);
  config.max_diagnostics = std::clamp<std::size_t>(config.max_diagnostics, 0U, 100U);
  return config;
}

void appendDiagnostic(PassageInsertionStats& stats,
                      const PassageInsertionConfig& config,
                      const PassageInsertionDiagnostic& diagnostic) {
  if (stats.diagnostics.size() >= config.max_diagnostics) {
    ++stats.diagnostics_dropped;
    return;
  }
  stats.diagnostics.push_back(diagnostic);
}

void countReject(PassageInsertionStats& stats,
                 const PassageInsertionRejectReason reason) noexcept {
  switch (reason) {
    case PassageInsertionRejectReason::kNonTraversable:
      ++stats.rejected_traversability;
      break;
    case PassageInsertionRejectReason::kValidationNotImproved:
      ++stats.rejected_validation;
      break;
    case PassageInsertionRejectReason::kInvalidGeometry:
    case PassageInsertionRejectReason::kEndpointMismatch:
    case PassageInsertionRejectReason::kInvalidOpeningFrame:
    case PassageInsertionRejectReason::kExcessiveLateralShift:
      ++stats.rejected_geometry;
      break;
    case PassageInsertionRejectReason::kJoinTangent:
    case PassageInsertionRejectReason::kJoinCurvature:
    case PassageInsertionRejectReason::kInsertedRadius:
      ++stats.rejected_join;
      break;
    default:
      break;
  }
}

[[nodiscard]] const PassageStructure*
findStructure(const KnownPassageMap& map, const std::string& structure_id) noexcept {
  for (const PassageStructure& structure : map.structures) {
    if (structure.id == structure_id) {
      return &structure;
    }
  }
  return nullptr;
}

[[nodiscard]] TrajectoryPointSample
sampleAtS(const std::span<const TrajectoryPointSample> samples, const double s_m) {
  if (samples.empty()) {
    return TrajectoryPointSample{};
  }
  const double clamped_s = std::clamp(std::isfinite(s_m) ? s_m : 0.0,
                                      samples.front().s_m, samples.back().s_m);
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (clamped_s > end.s_m && i + 2U < samples.size()) {
      continue;
    }
    const double ds = end.s_m - start.s_m;
    const double t =
        ds > kTinyDistanceM ? std::clamp((clamped_s - start.s_m) / ds, 0.0, 1.0) : 0.0;
    TrajectoryPointSample sample = t <= 0.5 ? start : end;
    sample.s_m = clamped_s;
    sample.point = start.point * (1.0 - t) + end.point * t;
    sample.tangent = normalized(start.tangent * (1.0 - t) + end.tangent * t);
    sample.curvature_1pm = start.curvature_1pm * (1.0 - t) + end.curvature_1pm * t;
    sample.z_m = start.z_m * (1.0 - t) + end.z_m * t;
    return sample;
  }
  return samples.back();
}

[[nodiscard]] PassageInsertionBlockedSegmentDiagnostic
firstNonTraversableSegment(const OccupancyGrid2D& grid,
                           const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return PassageInsertionBlockedSegmentDiagnostic{};
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i - 1U];
    const TrajectoryPointSample& end = samples[i];
    PassageInsertionBlockedSegmentDiagnostic diagnostic{};
    diagnostic.available = true;
    diagnostic.segment_index = i - 1U;
    diagnostic.start_s_m = start.s_m;
    diagnostic.end_s_m = end.s_m;
    diagnostic.start_point = start.point;
    diagnostic.end_point = end.point;
    const std::optional<GridIndex> start_cell = grid.worldToCell(start.point);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end.point);
    diagnostic.start_cell_available = start_cell.has_value();
    diagnostic.end_cell_available = end_cell.has_value();
    if (!start_cell.has_value() || !end_cell.has_value()) {
      return diagnostic;
    }
    diagnostic.start_cell = *start_cell;
    diagnostic.end_cell = *end_cell;
    const std::vector<GridIndex> line_cells = grid.cellsOnLine(*start_cell, *end_cell);
    for (std::size_t cell_index = 0U; cell_index < line_cells.size(); ++cell_index) {
      const GridIndex cell = line_cells[cell_index];
      if (!grid.isProhibited(cell)) {
        continue;
      }
      diagnostic.line_cell_index = cell_index;
      diagnostic.line_cell_count = line_cells.size();
      diagnostic.blocked_cell_available = true;
      diagnostic.blocked_cell = cell;
      diagnostic.blocked_cell_center = grid.cellCenter(cell);
      diagnostic.occupied = grid.isOccupied(cell);
      diagnostic.inflated = grid.isInflated(cell);
      return diagnostic;
    }
  }
  return PassageInsertionBlockedSegmentDiagnostic{};
}

[[nodiscard]] std::size_t
countInvalidMatches(const std::vector<KnownPassageTraversalMatch>& matches) noexcept {
  return static_cast<std::size_t>(std::ranges::count_if(
      matches, [](const KnownPassageTraversalMatch& match) { return !match.valid; }));
}

[[nodiscard]] bool
needsPassageInsertionRepair(const KnownPassageTraversalMatch& match,
                            const PassageInsertionConfig& config) noexcept {
  if (!match.valid) {
    return match.reason == KnownPassageValidationReason::kOpeningVolumeMiss ||
           match.reason == KnownPassageValidationReason::kStructureWithoutOpening;
  }
  return config.repair_clearance_margin_m > 0.0 && std::isfinite(match.clearance_m) &&
         match.clearance_m + kTinyDistanceM < config.repair_clearance_margin_m;
}

[[nodiscard]] std::size_t
countRepairCandidates(const std::vector<KnownPassageTraversalMatch>& matches,
                      const PassageInsertionConfig& config) noexcept {
  return static_cast<std::size_t>(std::ranges::count_if(
      matches, [&config](const KnownPassageTraversalMatch& match) {
        return needsPassageInsertionRepair(match, config);
      }));
}

[[nodiscard]] double
spanLateralMissM(const std::span<const TrajectoryPointSample> samples,
                 const double entry_s_m, const double exit_s_m,
                 const PassageOpening& opening, const KnownPassageOpeningFrame& frame,
                 const PassageInsertionConfig& config) {
  double max_miss = 0.0;
  const std::array<double, 3U> stations{entry_s_m, 0.5 * (entry_s_m + exit_s_m),
                                        exit_s_m};
  for (const double s_m : stations) {
    const TrajectoryPointSample sample = sampleAtS(samples, s_m);
    const KnownPassageOpeningLocalPoint local = knownPassageOpeningLocalPoint(
        KnownPassageOpeningWorldPoint{
            .point = sample.point, .z_m = opening.center.z, .s_m = s_m},
        frame);
    const double allowed_v =
        std::max(0.0, frame.half_width_m - config.opening_lateral_target_margin_m);
    max_miss = std::max(max_miss, std::max(0.0, std::abs(local.v_m) - allowed_v));
  }
  return max_miss;
}

void appendSampleIfSeparated(std::vector<TrajectoryPointSample>& samples,
                             TrajectoryPointSample sample) {
  if (!samples.empty() &&
      distance(samples.back().point, sample.point) <= kTinyDistanceM) {
    samples.back() = sample;
    return;
  }
  samples.push_back(std::move(sample));
}

void appendHermitePoints(std::vector<TrajectoryPointSample>& output, const Point2 p0,
                         const Point2 t0, const Point2 p1, const Point2 t1,
                         const double step_m) {
  const double length_m = distance(p0, p1);
  if (!(length_m > kTinyDistanceM)) {
    return;
  }
  const std::size_t steps =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  const Point2 m0 = normalized(t0) * length_m * 0.65;
  const Point2 m1 = normalized(t1) * length_m * 0.65;
  for (std::size_t i = 0U; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    TrajectoryPointSample sample{};
    sample.point = p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11;
    sample.left_bound_m = std::numeric_limits<double>::quiet_NaN();
    sample.right_bound_m = std::numeric_limits<double>::quiet_NaN();
    sample.lateral_offset_m = std::numeric_limits<double>::quiet_NaN();
    appendSampleIfSeparated(output, sample);
  }
}

[[nodiscard]] double
maxCurvatureJumpNear(const std::span<const TrajectoryPointSample> samples,
                     const double s_m) {
  double max_jump = 0.0;
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (std::abs(samples[i].s_m - s_m) > 2.5 &&
        std::abs(samples[i - 1U].s_m - s_m) > 2.5) {
      continue;
    }
    max_jump = std::max(
        max_jump, std::abs(samples[i].curvature_1pm - samples[i - 1U].curvature_1pm));
  }
  return max_jump;
}

[[nodiscard]] TrajectoryPointSample
closestSampleToPoint(const std::span<const TrajectoryPointSample> samples,
                     const Point2 point) {
  if (samples.empty()) {
    return TrajectoryPointSample{};
  }
  const TrajectoryPointSample* best = &samples.front();
  double best_distance_sq = squaredDistance(best->point, point);
  for (const TrajectoryPointSample& sample : samples) {
    const double distance_sq = squaredDistance(sample.point, point);
    if (distance_sq < best_distance_sq) {
      best = &sample;
      best_distance_sq = distance_sq;
    }
  }
  return *best;
}

[[nodiscard]] double
minRadiusInRange(const std::span<const TrajectoryPointSample> samples,
                 const double start_s_m, const double end_s_m) {
  double max_abs_curvature = 0.0;
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m + kTinyDistanceM < start_s_m ||
        sample.s_m > end_s_m + kTinyDistanceM) {
      continue;
    }
    max_abs_curvature = std::max(max_abs_curvature, std::abs(sample.curvature_1pm));
  }
  if (!(max_abs_curvature > kTinyDistanceM)) {
    return std::numeric_limits<double>::infinity();
  }
  return 1.0 / max_abs_curvature;
}

[[nodiscard]] std::vector<TrajectoryPointSample> buildStitchedCandidate(
    const std::span<const TrajectoryPointSample> samples, const PassageOpening& opening,
    KnownPassageOpeningFrame frame, const double anchor_s_m, const double reconnect_s_m,
    const PassageInsertionConfig& config, const double initial_altitude_m) {
  const TrajectoryPointSample anchor = sampleAtS(samples, anchor_s_m);
  const TrajectoryPointSample reconnect = sampleAtS(samples, reconnect_s_m);
  if (dot(reconnect.point - anchor.point, frame.normal) < 0.0) {
    frame.normal = frame.normal * -1.0;
    frame.lateral = Point2{-frame.normal.y, frame.normal.x};
  }

  const Point2 gate_entry = knownPassageOpeningGateEntryPoint(opening, frame);
  const Point2 gate_exit = knownPassageOpeningGateExitPoint(opening, frame);

  std::vector<TrajectoryPointSample> stitched;
  stitched.reserve(samples.size() + 16U);
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m + kTinyDistanceM < anchor_s_m) {
      stitched.push_back(sample);
    }
  }
  appendSampleIfSeparated(stitched, anchor);
  appendHermitePoints(stitched, anchor.point, anchor.tangent, gate_entry, frame.normal,
                      config.sample_step_m);
  appendHermitePoints(stitched, gate_entry, frame.normal, gate_exit, frame.normal,
                      config.sample_step_m);
  appendHermitePoints(stitched, gate_exit, frame.normal, reconnect.point,
                      reconnect.tangent, config.sample_step_m);
  appendSampleIfSeparated(stitched, reconnect);
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m > reconnect_s_m + kTinyDistanceM) {
      stitched.push_back(sample);
    }
  }

  populateTrajectorySampleGeometry(stitched);
  for (TrajectoryPointSample& sample : stitched) {
    sample.z_m = initial_altitude_m;
    sample.vertical_slope_dz_ds = 0.0;
    sample.vertical_speed_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_accel_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_jerk_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_constraint_active = false;
    sample.vertical_hard_window_active = false;
    sample.vertical_safe_min_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_safe_max_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_gate_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_profile_passage_id.clear();
  }
  return stitched;
}

struct CandidateEvaluation {
  std::vector<TrajectoryPointSample> samples;
  PassageInsertionDiagnostic diagnostic{};
  PassageInsertionRejectReason reason{PassageInsertionRejectReason::kNoCandidate};
  bool accepted{false};
  double score{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] CandidateEvaluation
evaluateCandidate(const std::span<const TrajectoryPointSample> original_samples,
                  const OccupancyGrid2D& grid, const KnownPassageMap& map,
                  const KnownPassageValidationConfig& validation_config,
                  const PassageInsertionConfig& config,
                  const KnownPassageTraversalMatch& match,
                  const PassageOpening& opening, const std::size_t before_violations,
                  const double initial_altitude_m) {
  CandidateEvaluation evaluation{};
  evaluation.diagnostic.structure_id = match.structure_id;
  evaluation.diagnostic.opening_id = opening.id;
  evaluation.diagnostic.entry_s_m = match.entry_s_m;
  evaluation.diagnostic.exit_s_m = match.exit_s_m;

  const std::optional<KnownPassageOpeningFrame> frame =
      knownPassageOpeningFrame(opening);
  if (!frame.has_value()) {
    evaluation.reason = PassageInsertionRejectReason::kInvalidOpeningFrame;
    return evaluation;
  }
  evaluation.diagnostic.lateral_miss_before_m = spanLateralMissM(
      original_samples, match.entry_s_m, match.exit_s_m, opening, *frame, config);
  if (evaluation.diagnostic.lateral_miss_before_m > config.max_lateral_shift_m) {
    evaluation.reason = PassageInsertionRejectReason::kExcessiveLateralShift;
    return evaluation;
  }

  const double pre_margin =
      std::clamp(std::max(config.min_anchor_margin_m, opening.approach_distance_m),
                 config.min_anchor_margin_m, config.max_anchor_margin_m);
  const double post_margin =
      std::clamp(std::max(config.min_anchor_margin_m, opening.exit_distance_m),
                 config.min_anchor_margin_m, config.max_anchor_margin_m);
  const double anchor_s_m =
      std::max(original_samples.front().s_m, match.entry_s_m - pre_margin);
  const double reconnect_s_m =
      std::min(original_samples.back().s_m, match.exit_s_m + post_margin);
  evaluation.diagnostic.anchor_s_m = anchor_s_m;
  evaluation.diagnostic.reconnect_s_m = reconnect_s_m;

  evaluation.samples =
      buildStitchedCandidate(original_samples, opening, *frame, anchor_s_m,
                             reconnect_s_m, config, initial_altitude_m);
  if (!trajectorySamplesAreUsable(evaluation.samples)) {
    evaluation.reason = PassageInsertionRejectReason::kInvalidGeometry;
    return evaluation;
  }
  if (distance(evaluation.samples.front().point, original_samples.front().point) >
          kEndpointToleranceM ||
      distance(evaluation.samples.back().point, original_samples.back().point) >
          kEndpointToleranceM) {
    evaluation.reason = PassageInsertionRejectReason::kEndpointMismatch;
    return evaluation;
  }
  const PassageInsertionBlockedSegmentDiagnostic blocked_segment =
      firstNonTraversableSegment(grid, evaluation.samples);
  if (blocked_segment.available) {
    evaluation.diagnostic.blocked_segment = blocked_segment;
    evaluation.reason = PassageInsertionRejectReason::kNonTraversable;
    return evaluation;
  }

  const TrajectoryPointSample original_anchor = sampleAtS(original_samples, anchor_s_m);
  const TrajectoryPointSample original_reconnect =
      sampleAtS(original_samples, reconnect_s_m);
  const TrajectoryPointSample candidate_anchor =
      closestSampleToPoint(evaluation.samples, original_anchor.point);
  const TrajectoryPointSample candidate_reconnect =
      closestSampleToPoint(evaluation.samples, original_reconnect.point);
  evaluation.diagnostic.join_tangent_delta_before_rad =
      angleBetween(original_anchor.tangent, candidate_anchor.tangent);
  evaluation.diagnostic.join_tangent_delta_after_rad =
      angleBetween(original_reconnect.tangent, candidate_reconnect.tangent);
  if (evaluation.diagnostic.join_tangent_delta_before_rad >
          config.max_join_tangent_delta_rad ||
      evaluation.diagnostic.join_tangent_delta_after_rad >
          config.max_join_tangent_delta_rad) {
    evaluation.reason = PassageInsertionRejectReason::kJoinTangent;
    return evaluation;
  }

  evaluation.diagnostic.join_curvature_jump_before_1pm =
      maxCurvatureJumpNear(evaluation.samples, candidate_anchor.s_m);
  evaluation.diagnostic.join_curvature_jump_after_1pm =
      maxCurvatureJumpNear(evaluation.samples, candidate_reconnect.s_m);
  if (evaluation.diagnostic.join_curvature_jump_before_1pm >
          config.max_join_curvature_jump_1pm ||
      evaluation.diagnostic.join_curvature_jump_after_1pm >
          config.max_join_curvature_jump_1pm) {
    evaluation.reason = PassageInsertionRejectReason::kJoinCurvature;
    return evaluation;
  }

  evaluation.diagnostic.min_inserted_radius_m =
      minRadiusInRange(evaluation.samples, anchor_s_m, reconnect_s_m);
  if (config.min_inserted_radius_m > 0.0 &&
      evaluation.diagnostic.min_inserted_radius_m + kTinyDistanceM <
          config.min_inserted_radius_m) {
    evaluation.reason = PassageInsertionRejectReason::kInsertedRadius;
    return evaluation;
  }
  evaluation.diagnostic.lateral_miss_after_m = spanLateralMissM(
      evaluation.samples, match.entry_s_m, match.exit_s_m, opening, *frame, config);
  const std::vector<KnownPassageTraversalMatch> after_matches =
      findKnownPassageTraversalMatches(evaluation.samples, map, validation_config,
                                       true);
  const std::size_t after_violations = countInvalidMatches(after_matches);
  const bool invalid_matches_improved = after_violations < before_violations;
  const bool low_clearance_improved =
      before_violations == 0U && after_violations == 0U &&
      evaluation.diagnostic.lateral_miss_after_m + kTinyDistanceM <
          evaluation.diagnostic.lateral_miss_before_m;
  if (!invalid_matches_improved && !low_clearance_improved) {
    evaluation.reason = PassageInsertionRejectReason::kValidationNotImproved;
    return evaluation;
  }

  evaluation.reason = PassageInsertionRejectReason::kNone;
  evaluation.accepted = true;
  evaluation.score = evaluation.diagnostic.lateral_miss_after_m +
                     evaluation.diagnostic.join_tangent_delta_before_rad +
                     evaluation.diagnostic.join_tangent_delta_after_rad +
                     evaluation.diagnostic.join_curvature_jump_before_1pm * 10.0 +
                     evaluation.diagnostic.join_curvature_jump_after_1pm * 10.0;
  return evaluation;
}

} // namespace

const char*
passageInsertionRejectReasonName(const PassageInsertionRejectReason reason) noexcept {
  switch (reason) {
    case PassageInsertionRejectReason::kNone:
      return "none";
    case PassageInsertionRejectReason::kDisabled:
      return "disabled";
    case PassageInsertionRejectReason::kNoMap:
      return "no_map";
    case PassageInsertionRejectReason::kInvalidInput:
      return "invalid_input";
    case PassageInsertionRejectReason::kNoRepairNeeded:
      return "no_repair_needed";
    case PassageInsertionRejectReason::kNoCandidate:
      return "no_candidate";
    case PassageInsertionRejectReason::kTooManyCandidates:
      return "too_many_candidates";
    case PassageInsertionRejectReason::kInvalidOpeningFrame:
      return "invalid_opening_frame";
    case PassageInsertionRejectReason::kExcessiveLateralShift:
      return "excessive_lateral_shift";
    case PassageInsertionRejectReason::kInvalidGeometry:
      return "invalid_geometry";
    case PassageInsertionRejectReason::kNonTraversable:
      return "non_traversable";
    case PassageInsertionRejectReason::kEndpointMismatch:
      return "endpoint_mismatch";
    case PassageInsertionRejectReason::kJoinTangent:
      return "join_tangent";
    case PassageInsertionRejectReason::kJoinCurvature:
      return "join_curvature";
    case PassageInsertionRejectReason::kInsertedRadius:
      return "inserted_radius";
    case PassageInsertionRejectReason::kValidationNotImproved:
      return "validation_not_improved";
  }
  return "unknown";
}

PassageInsertionResult insertLocalPassageSegments(
    const std::span<const TrajectoryPointSample> samples, const OccupancyGrid2D& grid,
    const KnownPassageMap* const map,
    const KnownPassageValidationConfig& validation_config,
    const PassageInsertionConfig& input_config, const double initial_altitude_m) {
  const PassageInsertionConfig config = sanitizeConfig(input_config);
  PassageInsertionResult result{};
  result.samples.assign(samples.begin(), samples.end());
  result.stats.enabled = config.enabled;
  result.valid = trajectorySamplesAreUsable(samples);

  if (!config.enabled) {
    result.stats.final_reason = PassageInsertionRejectReason::kDisabled;
    return result;
  }
  if (map == nullptr) {
    result.stats.final_reason = PassageInsertionRejectReason::kNoMap;
    return result;
  }
  if (!trajectorySamplesAreUsable(samples)) {
    result.stats.final_reason = PassageInsertionRejectReason::kInvalidInput;
    result.valid = false;
    return result;
  }
  if (!validation_config.enabled) {
    result.stats.final_reason = PassageInsertionRejectReason::kNoRepairNeeded;
    return result;
  }

  const std::vector<KnownPassageTraversalMatch> before_matches =
      findKnownPassageTraversalMatches(samples, *map, validation_config, true);
  const std::size_t before_violations = countInvalidMatches(before_matches);
  const std::size_t repair_candidates = countRepairCandidates(before_matches, config);
  if (repair_candidates == 0U) {
    result.stats.final_reason = PassageInsertionRejectReason::kNoRepairNeeded;
    return result;
  }
  if (config.max_candidates == 0U) {
    result.stats.final_reason = PassageInsertionRejectReason::kTooManyCandidates;
    return result;
  }

  std::optional<CandidateEvaluation> best;
  for (const KnownPassageTraversalMatch& match : before_matches) {
    if (!needsPassageInsertionRepair(match, config)) {
      continue;
    }
    const PassageStructure* const structure = findStructure(*map, match.structure_id);
    if (structure == nullptr || structure->openings.empty()) {
      continue;
    }
    std::vector<PassageOpening> candidate_openings;
    if (match.valid) {
      candidate_openings.push_back(match.opening);
    } else {
      candidate_openings = structure->openings;
    }
    for (const PassageOpening& opening : candidate_openings) {
      if (result.stats.candidates >= config.max_candidates) {
        result.stats.final_reason = PassageInsertionRejectReason::kTooManyCandidates;
        break;
      }
      ++result.stats.candidates;
      CandidateEvaluation evaluation =
          evaluateCandidate(samples, grid, *map, validation_config, config, match,
                            opening, before_violations, initial_altitude_m);
      evaluation.diagnostic.reason = evaluation.reason;
      evaluation.diagnostic.accepted = evaluation.accepted;
      appendDiagnostic(result.stats, config, evaluation.diagnostic);
      if (!evaluation.accepted) {
        countReject(result.stats, evaluation.reason);
        continue;
      }
      if (!best.has_value() || evaluation.score < best->score) {
        best = std::move(evaluation);
      }
    }
  }

  if (!best.has_value()) {
    if (result.stats.final_reason == PassageInsertionRejectReason::kDisabled ||
        result.stats.final_reason == PassageInsertionRejectReason::kNone) {
      result.stats.final_reason = PassageInsertionRejectReason::kNoCandidate;
    }
    return result;
  }

  result.samples = std::move(best->samples);
  result.applied = true;
  result.valid = true;
  result.stats.applied = true;
  result.stats.inserted_count = 1U;
  result.stats.final_reason = PassageInsertionRejectReason::kNone;
  return result;
}

} // namespace drone_city_nav

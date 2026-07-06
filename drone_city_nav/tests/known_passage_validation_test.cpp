#include "drone_city_nav/known_passage_validation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageOpening
makeOpening(const std::string& id = "main", const double center_y_m = 0.0,
            const double min_z_m = 8.0, const double max_z_m = 12.0,
            const double width_m = 4.0, const double depth_m = 4.0) {
  PassageOpening opening{};
  opening.id = id;
  opening.structure_id = "arch";
  opening.center = Point3{0.0, center_y_m, (min_z_m + max_z_m) / 2.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = width_m;
  opening.height_m = max_z_m - min_z_m;
  opening.depth_m = depth_m;
  opening.min_z_m = min_z_m;
  opening.max_z_m = max_z_m;
  opening.approach_distance_m = 10.0;
  opening.exit_distance_m = 10.0;
  return opening;
}

[[nodiscard]] PassageStructure makeStructure() {
  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(makeOpening());
  return structure;
}

[[nodiscard]] KnownPassageMap makeMap(PassageStructure structure = makeStructure()) {
  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(std::move(structure));
  return map;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
makeLineSamples(const Point2 start, const Point2 end, const double z_m,
                const std::size_t count = 5U) {
  std::vector<TrajectoryPointSample> samples;
  samples.reserve(count);
  for (std::size_t i = 0U; i < count; ++i) {
    const double t =
        count <= 1U ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1U);
    TrajectoryPointSample sample{};
    sample.point =
        Point2{start.x + (end.x - start.x) * t, start.y + (end.y - start.y) * t};
    sample.s_m = distance(start, sample.point);
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = z_m;
    samples.push_back(sample);
  }
  return samples;
}

[[nodiscard]] KnownPassageValidationSummary
validate(std::span<const TrajectoryPointSample> samples, const KnownPassageMap* map,
         KnownPassageValidationConfig config = KnownPassageValidationConfig{}) {
  return validateKnownPassageTraversal(samples, map, config);
}

} // namespace

TEST(KnownPassageValidation, ReasonNamesAreStable) {
  EXPECT_STREQ(
      knownPassageValidationReasonName(KnownPassageValidationReason::kDisabled),
      "disabled");
  EXPECT_STREQ(knownPassageValidationReasonName(KnownPassageValidationReason::kNoMap),
               "no_map");
  EXPECT_STREQ(knownPassageValidationReasonName(
                   KnownPassageValidationReason::kInvalidTrajectory),
               "invalid_trajectory");
  EXPECT_STREQ(knownPassageValidationReasonName(
                   KnownPassageValidationReason::kNoStructureIntersection),
               "no_structure_intersection");
  EXPECT_STREQ(
      knownPassageValidationReasonName(KnownPassageValidationReason::kMatchedOpening),
      "matched_opening");
  EXPECT_STREQ(knownPassageValidationReasonName(
                   KnownPassageValidationReason::kStructureWithoutOpening),
               "structure_without_opening");
  EXPECT_STREQ(knownPassageValidationReasonName(
                   KnownPassageValidationReason::kOpeningVolumeMiss),
               "opening_volume_miss");
}

TEST(KnownPassageValidation, DisabledSkipsValidation) {
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);
  KnownPassageValidationConfig config{};
  config.enabled = false;

  const KnownPassageValidationSummary summary = validate(samples, nullptr, config);

  EXPECT_FALSE(summary.enabled);
  EXPECT_TRUE(summary.valid);
  EXPECT_EQ(summary.worst_reason, KnownPassageValidationReason::kDisabled);
  EXPECT_TRUE(summary.diagnostics.empty());
}

TEST(KnownPassageValidation, NoMapIsValidWithNoMapReason) {
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, nullptr);

  EXPECT_TRUE(summary.enabled);
  EXPECT_TRUE(summary.valid);
  EXPECT_EQ(summary.worst_reason, KnownPassageValidationReason::kNoMap);
  EXPECT_EQ(summary.structures_checked, 0U);
}

TEST(KnownPassageValidation, NoStructureIntersectionIsValid) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 8.0}, Point2{8.0, 8.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_TRUE(summary.valid);
  EXPECT_EQ(summary.structures_checked, 1U);
  EXPECT_EQ(summary.structures_intersected, 0U);
  EXPECT_EQ(summary.worst_reason,
            KnownPassageValidationReason::kNoStructureIntersection);
}

TEST(KnownPassageValidation, StructureWithoutOpeningReportsViolation) {
  PassageStructure structure = makeStructure();
  structure.openings.clear();
  const KnownPassageMap map = makeMap(structure);
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.structures_intersected, 1U);
  EXPECT_EQ(summary.violations, 1U);
  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_EQ(summary.diagnostics.front().structure_id, "arch");
  EXPECT_EQ(summary.diagnostics.front().reason,
            KnownPassageValidationReason::kStructureWithoutOpening);
  EXPECT_FALSE(summary.diagnostics.front().valid);
}

TEST(KnownPassageValidation, OpeningVolumeMatchIsValid) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_TRUE(summary.valid);
  EXPECT_EQ(summary.structures_intersected, 1U);
  EXPECT_EQ(summary.opening_matches, 1U);
  EXPECT_EQ(summary.violations, 0U);
  EXPECT_EQ(summary.worst_reason, KnownPassageValidationReason::kMatchedOpening);
  ASSERT_EQ(summary.diagnostics.size(), 1U);
  const KnownPassageValidationSpan& span = summary.diagnostics.front();
  EXPECT_TRUE(span.valid);
  EXPECT_EQ(span.opening_id, "main");
  EXPECT_EQ(span.reason, KnownPassageValidationReason::kMatchedOpening);
  EXPECT_NEAR(span.entry_s_m, 3.0, 1.0e-6);
  EXPECT_NEAR(span.exit_s_m, 13.0, 1.0e-6);
  EXPECT_NEAR(span.overlap_m, 4.0, 1.0e-6);
  EXPECT_NEAR(span.clearance_m, 2.0, 1.0e-6);
}

TEST(KnownPassageValidation, OpeningVolumeMissesByAltitude) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 15.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.violations, 1U);
  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_EQ(summary.diagnostics.front().reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
  EXPECT_LT(summary.diagnostics.front().clearance_m, 0.0);
}

TEST(KnownPassageValidation, OpeningVolumeMissesByLateralOffset) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 3.0}, Point2{8.0, 3.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.violations, 1U);
  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_EQ(summary.diagnostics.front().reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
  EXPECT_LT(summary.diagnostics.front().clearance_m, 0.0);
}

TEST(KnownPassageValidation, OpeningVolumeMissesByDepth) {
  PassageStructure structure = makeStructure();
  structure.openings.front().center.x = 20.0;
  const KnownPassageMap map = makeMap(structure);
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.violations, 1U);
  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_EQ(summary.diagnostics.front().reason,
            KnownPassageValidationReason::kOpeningVolumeMiss);
}

TEST(KnownPassageValidation, ChoosesBestOpeningByOverlapThenClearance) {
  PassageStructure structure = makeStructure();
  structure.openings.clear();
  structure.openings.push_back(makeOpening("narrow", 0.0, 9.0, 11.0, 2.0, 4.0));
  structure.openings.push_back(makeOpening("wide", 0.0, 8.0, 12.0, 6.0, 4.0));
  const KnownPassageMap map = makeMap(structure);
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_TRUE(summary.diagnostics.front().valid);
  EXPECT_EQ(summary.diagnostics.front().opening_id, "wide");
  EXPECT_NEAR(summary.diagnostics.front().overlap_m, 4.0, 1.0e-6);
  EXPECT_NEAR(summary.diagnostics.front().clearance_m, 2.0, 1.0e-6);
}

TEST(KnownPassageValidation, HandlesSegmentStartingInsideFootprint) {
  const KnownPassageMap map = makeMap();
  const std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{0.0, 0.0}, Point2{8.0, 0.0}, 10.0);

  const KnownPassageValidationSummary summary = validate(samples, &map);

  ASSERT_EQ(summary.diagnostics.size(), 1U);
  EXPECT_TRUE(summary.diagnostics.front().valid);
  EXPECT_NEAR(summary.diagnostics.front().entry_s_m, 0.0, 1.0e-6);
  EXPECT_NEAR(summary.diagnostics.front().exit_s_m, 5.0, 1.0e-6);
}

TEST(KnownPassageValidation, RejectsInvalidTrajectorySamples) {
  const KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples =
      makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);
  samples[2].s_m = samples[1].s_m - 1.0;

  KnownPassageValidationSummary summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.violations, 1U);
  EXPECT_EQ(summary.worst_reason, KnownPassageValidationReason::kInvalidTrajectory);

  samples = makeLineSamples(Point2{-8.0, 0.0}, Point2{8.0, 0.0}, 10.0);
  samples[2].z_m = std::numeric_limits<double>::quiet_NaN();

  summary = validate(samples, &map);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.worst_reason, KnownPassageValidationReason::kInvalidTrajectory);
}

TEST(KnownPassageValidation, CapsDiagnostics) {
  KnownPassageMap map{};
  map.frame_id = "map";
  for (std::size_t i = 0U; i < 3U; ++i) {
    PassageStructure structure = makeStructure();
    structure.id = "arch_" + std::to_string(i);
    structure.center.y = static_cast<double>(i) * 8.0;
    structure.openings.clear();
    map.structures.push_back(structure);
  }
  std::vector<TrajectoryPointSample> samples;
  samples.reserve(7U);
  for (std::size_t i = 0U; i < 7U; ++i) {
    TrajectoryPointSample sample{};
    sample.point = Point2{0.0, -4.0 + static_cast<double>(i) * 4.0};
    sample.s_m = static_cast<double>(i) * 4.0;
    sample.tangent = Point2{0.0, 1.0};
    sample.z_m = 10.0;
    samples.push_back(sample);
  }
  KnownPassageValidationConfig config{};
  config.max_diagnostics = 2U;

  const KnownPassageValidationSummary summary = validate(samples, &map, config);

  EXPECT_FALSE(summary.valid);
  EXPECT_EQ(summary.structures_intersected, 3U);
  EXPECT_EQ(summary.violations, 3U);
  EXPECT_EQ(summary.diagnostics.size(), 2U);
}

} // namespace drone_city_nav

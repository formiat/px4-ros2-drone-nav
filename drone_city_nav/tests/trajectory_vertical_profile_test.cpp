#include "drone_city_nav/trajectory_vertical_profile.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageOpening makeOpening(const double min_z_m = 8.0,
                                         const double max_z_m = 12.0) {
  PassageOpening opening{};
  opening.id = "window";
  opening.structure_id = "arch";
  opening.center = Point3{0.0, 0.0, 0.5 * (min_z_m + max_z_m)};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 6.0;
  opening.height_m = max_z_m - min_z_m;
  opening.depth_m = 6.0;
  opening.min_z_m = min_z_m;
  opening.max_z_m = max_z_m;
  opening.approach_distance_m = 8.0;
  opening.exit_distance_m = 8.0;
  return opening;
}

[[nodiscard]] KnownPassageMap makeMap() {
  PassageStructure structure{};
  structure.id = "arch";
  structure.center = Point2{0.0, 0.0};
  structure.size_x_m = 10.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(makeOpening());

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(std::move(structure));
  return map;
}

[[nodiscard]] PassageStructure makeStructure(std::string id, const double center_x_m,
                                             PassageOpening opening) {
  opening.structure_id = id;
  opening.center.x = center_x_m;
  PassageStructure structure{};
  structure.id = std::move(id);
  structure.center = Point2{center_x_m, 0.0};
  structure.size_x_m = 8.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 20.0;
  structure.openings.push_back(std::move(opening));
  return structure;
}

[[nodiscard]] KnownPassageMap makeOverlappingIncompatibleMap() {
  PassageOpening low = makeOpening(8.0, 12.0);
  low.id = "low_window";
  PassageOpening high = makeOpening(13.0, 15.0);
  high.id = "high_window";

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(makeStructure("low_arch", -4.0, std::move(low)));
  map.structures.push_back(makeStructure("high_arch", 4.0, std::move(high)));
  return map;
}

[[nodiscard]] KnownPassageMap makeOverlappingCompatibleMap() {
  PassageOpening first = makeOpening(8.0, 12.0);
  first.id = "first_window";
  PassageOpening second = makeOpening(8.0, 12.0);
  second.id = "second_window";

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(makeStructure("first_arch", -4.0, std::move(first)));
  map.structures.push_back(makeStructure("second_arch", 4.0, std::move(second)));
  return map;
}

[[nodiscard]] KnownPassageMap makeCloseSequentialPassagesMap() {
  PassageOpening first = makeOpening(13.0, 15.0);
  first.id = "first_mid_window";
  PassageOpening second = makeOpening(20.0, 22.0);
  second.id = "second_high_window";

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(makeStructure("first_arch", 0.0, std::move(first)));
  map.structures.push_back(makeStructure("second_arch", 50.0, std::move(second)));
  return map;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
makeSamplesRange(const double start_x_m, const double end_x_m, const double step_m,
                 const double z_m) {
  std::vector<TrajectoryPointSample> samples;
  const std::size_t count =
      static_cast<std::size_t>(std::floor((end_x_m - start_x_m) / step_m));
  samples.reserve(count + 1U);
  for (std::size_t i = 0U; i <= count; ++i) {
    const double x = start_x_m + static_cast<double>(i) * step_m;
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * step_m;
    sample.point = Point2{x, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = z_m;
    samples.push_back(sample);
  }
  return samples;
}

[[nodiscard]] std::vector<TrajectoryPointSample> makeSamples(const double z_m) {
  return makeSamplesRange(-20.0, 20.0, 2.0, z_m);
}

} // namespace

TEST(TrajectoryVerticalProfile, DisabledKeepsInitialAltitude) {
  std::vector<TrajectoryPointSample> samples = makeSamples(0.0);
  VerticalProfileConfig config{};
  config.enabled = false;

  const VerticalProfileResult result = applyVerticalProfile(
      samples, nullptr, KnownPassageValidationConfig{}, config, 18.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.applied);
  EXPECT_FALSE(result.stats.active);
  for (const TrajectoryPointSample& sample : samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 18.0);
    EXPECT_FALSE(sample.vertical_constraint_active);
  }
}

TEST(TrajectoryVerticalProfile, EnabledWithoutMapKeepsInitialAltitude) {
  std::vector<TrajectoryPointSample> samples = makeSamples(0.0);
  VerticalProfileConfig config{};
  config.enabled = true;

  const VerticalProfileResult result = applyVerticalProfile(
      samples, nullptr, KnownPassageValidationConfig{}, config, 18.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.applied);
  EXPECT_FALSE(result.stats.active);
  for (const TrajectoryPointSample& sample : samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 18.0);
    EXPECT_TRUE(sample.vertical_profile_passage_id.empty());
  }
}

TEST(TrajectoryVerticalProfile, MatchedOpeningBuildsSmoothInitialGateCarryProfile) {
  KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples = makeSamplesRange(-50.0, 20.0, 2.0, 18.0);

  VerticalProfileConfig config{};
  config.max_climb_angle_rad = 80.0 * std::numbers::pi / 180.0;
  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_EQ(result.stats.passages_matched, 1U);
  EXPECT_EQ(result.stats.passages_profiled, 1U);
  ASSERT_EQ(result.stats.diagnostics.size(), 1U);
  EXPECT_LT(result.stats.diagnostics.front().gate_hold_start_s_m,
            result.stats.diagnostics.front().entry_s_m);
  EXPECT_NEAR(result.stats.min_z_m, 10.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(samples.front().z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 10.0);

  const KnownPassageValidationSummary validation =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.worst_reason, KnownPassageValidationReason::kMatchedOpening);
  EXPECT_EQ(validation.opening_matches, 1U);
  EXPECT_TRUE(std::ranges::any_of(samples, [](const TrajectoryPointSample& sample) {
    return sample.vertical_constraint_active &&
           sample.vertical_profile_passage_id == "window";
  }));
  EXPECT_TRUE(std::ranges::any_of(samples, [](const TrajectoryPointSample& sample) {
    return sample.vertical_hard_window_active &&
           sample.vertical_profile_passage_id == "window" &&
           std::abs(sample.vertical_safe_min_z_m - 8.5) <= 1.0e-9 &&
           std::abs(sample.vertical_safe_max_z_m - 11.5) <= 1.0e-9 &&
           std::abs(sample.vertical_gate_z_m - 10.0) <= 1.0e-9;
  }));
}

TEST(TrajectoryVerticalProfile, DefaultClimbAngleAccountsForSmootherstepPeakSlope) {
  KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples =
      makeSamplesRange(-140.0, 140.0, 2.0, 18.0);
  VerticalProfileConfig config{};

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_DOUBLE_EQ(samples.front().z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 10.0);
  EXPECT_NEAR(result.stats.min_z_m, 10.0, 1.0e-9);
  EXPECT_LE(result.stats.max_abs_dz_ds, std::tan(config.max_climb_angle_rad) + 1.0e-9);
}

TEST(TrajectoryVerticalProfile, GateAltitudeTargetsOpeningCenterOutsideMargin) {
  KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples =
      makeSamplesRange(-140.0, 140.0, 2.0, 4.0);
  VerticalProfileConfig config{};

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 4.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_NEAR(result.stats.max_z_m, 10.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(samples.front().z_m, 4.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 10.0);
  EXPECT_LE(result.stats.max_abs_dz_ds, std::tan(config.max_climb_angle_rad) + 1.0e-9);
}

TEST(TrajectoryVerticalProfile, OverlappingIncompatibleWindowsAreRejected) {
  KnownPassageMap map = makeOverlappingIncompatibleMap();
  std::vector<TrajectoryPointSample> samples =
      makeSamplesRange(-120.0, 120.0, 2.0, 18.0);
  VerticalProfileConfig config{};

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.stats.valid);
  EXPECT_FALSE(result.stats.active);
  EXPECT_EQ(result.stats.passages_matched, 2U);
  EXPECT_EQ(result.stats.passages_profiled, 0U);
  EXPECT_TRUE(std::ranges::any_of(
      result.stats.diagnostics, [](const VerticalProfilePassageDiagnostic& diagnostic) {
        return diagnostic.reason == "insufficient_transition_distance" &&
               !diagnostic.valid;
      }));
  for (const TrajectoryPointSample& sample : samples) {
    EXPECT_DOUBLE_EQ(sample.z_m, 18.0);
    EXPECT_TRUE(sample.vertical_profile_passage_id.empty());
  }
}

TEST(TrajectoryVerticalProfile, OverlappingCompatibleWindowsShareCarriedAltitude) {
  KnownPassageMap map = makeOverlappingCompatibleMap();
  std::vector<TrajectoryPointSample> samples =
      makeSamplesRange(-120.0, 120.0, 2.0, 18.0);
  VerticalProfileConfig config{};

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_EQ(result.stats.passages_matched, 2U);
  EXPECT_EQ(result.stats.passages_profiled, 2U);
  const KnownPassageValidationSummary validation =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.opening_matches, 2U);
  EXPECT_DOUBLE_EQ(samples.front().z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 10.0);
}

TEST(TrajectoryVerticalProfile,
     CloseSequentialPassagesShrinkPreGateHoldBeforeRejecting) {
  KnownPassageMap map = makeCloseSequentialPassagesMap();
  std::vector<TrajectoryPointSample> samples = makeSamplesRange(-20.0, 90.0, 1.0, 14.0);
  VerticalProfileConfig config{};
  config.pre_gate_hold_time_s = 1.0;
  config.pre_gate_hold_min_distance_m = 22.0;
  config.pre_gate_hold_max_distance_m = 22.0;

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 14.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_EQ(result.stats.passages_matched, 2U);
  EXPECT_EQ(result.stats.passages_profiled, 2U);
  ASSERT_EQ(result.stats.diagnostics.size(), 2U);
  const VerticalProfilePassageDiagnostic& second = result.stats.diagnostics.back();
  EXPECT_EQ(second.opening_id, "second_high_window");
  EXPECT_LT(second.actual_gate_hold_m, second.desired_gate_hold_m);
  EXPECT_GT(second.actual_gate_hold_m, 0.0);
  EXPECT_GE(second.transition_available_m, second.transition_required_m);
  EXPECT_NEAR(samples.back().z_m, 21.0, 1.0e-9);

  const KnownPassageValidationSummary validation =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.opening_matches, 2U);
}

TEST(TrajectoryVerticalProfile, InfeasibleClimbAngleMarksResultInvalid) {
  KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples = makeSamples(18.0);
  VerticalProfileConfig config{};
  config.max_climb_angle_rad = 1.0 * std::numbers::pi / 180.0;
  config.min_transition_distance_m = 2.0;
  config.max_transition_distance_m = 2.0;

  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.stats.valid);
  EXPECT_FALSE(result.stats.active);
  EXPECT_TRUE(std::ranges::any_of(
      result.stats.diagnostics, [](const VerticalProfilePassageDiagnostic& diagnostic) {
        return diagnostic.reason == "insufficient_transition_distance" &&
               !diagnostic.valid;
      }));
}

} // namespace drone_city_nav

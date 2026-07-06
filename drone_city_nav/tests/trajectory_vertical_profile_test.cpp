#include "drone_city_nav/trajectory_vertical_profile.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
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

[[nodiscard]] std::vector<TrajectoryPointSample> makeSamples(const double z_m) {
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i <= 20U; ++i) {
    const double x = -20.0 + static_cast<double>(i) * 2.0;
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 2.0;
    sample.point = Point2{x, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = z_m;
    samples.push_back(sample);
  }
  return samples;
}

} // namespace

TEST(TrajectoryVerticalProfile, DisabledKeepsCruiseAltitude) {
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

TEST(TrajectoryVerticalProfile, MatchedOpeningBuildsSmoothCruiseGateCruiseProfile) {
  KnownPassageMap map = makeMap();
  std::vector<TrajectoryPointSample> samples = makeSamples(18.0);

  VerticalProfileConfig config{};
  config.max_climb_angle_rad = 80.0 * std::numbers::pi / 180.0;
  const VerticalProfileResult result =
      applyVerticalProfile(samples, &map, KnownPassageValidationConfig{}, config, 18.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.stats.active);
  EXPECT_EQ(result.stats.passages_matched, 1U);
  EXPECT_EQ(result.stats.passages_profiled, 1U);
  EXPECT_LT(result.stats.min_z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.front().z_m, 18.0);
  EXPECT_DOUBLE_EQ(samples.back().z_m, 18.0);

  const KnownPassageValidationSummary validation =
      validateKnownPassageTraversal(samples, &map, KnownPassageValidationConfig{});
  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.worst_reason, KnownPassageValidationReason::kMatchedOpening);
  EXPECT_EQ(validation.opening_matches, 1U);
  EXPECT_TRUE(std::ranges::any_of(samples, [](const TrajectoryPointSample& sample) {
    return sample.vertical_constraint_active &&
           sample.vertical_profile_passage_id == "window";
  }));
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
  EXPECT_TRUE(result.stats.active);
}

} // namespace drone_city_nav

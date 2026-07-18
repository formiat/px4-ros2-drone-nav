#include "drone_city_nav/trajectory_vertical_handover.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<TrajectoryPointSample>
straightSamples(const double altitude_m) {
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i <= 10U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 10.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = altitude_m;
    samples.push_back(sample);
  }
  return samples;
}

void markHardWindow(std::vector<TrajectoryPointSample>& samples,
                    const std::size_t begin_index, const std::size_t end_index,
                    const double altitude_m) {
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    samples[i].z_m = altitude_m;
    samples[i].vertical_hard_window_active = true;
    samples[i].vertical_constraint_active = true;
    samples[i].vertical_safe_min_z_m = altitude_m - 1.0;
    samples[i].vertical_safe_max_z_m = altitude_m + 1.0;
    samples[i].vertical_gate_z_m = altitude_m;
    samples[i].vertical_profile_passage_id = "low_opening";
  }
}

} // namespace

TEST(TrajectoryVerticalHandover, JoinsCurrentTargetToUpcomingHardWindow) {
  const std::vector<TrajectoryPointSample> current = straightSamples(15.0);
  std::vector<TrajectoryPointSample> candidate = straightSamples(20.0);
  markHardWindow(candidate, 6U, 7U, 8.0);

  const VerticalTrajectoryHandoverResult result = reanchorTrajectoryVerticalPrefix(
      current, candidate, Point2{20.0, 0.0},
      VerticalTrajectoryHandoverState{.current_altitude_m = 14.5,
                                      .current_vertical_velocity_mps = -1.0,
                                      .current_horizontal_speed_mps = 10.0,
                                      .altitude_valid = true,
                                      .vertical_velocity_valid = true});

  ASSERT_TRUE(result.applied);
  EXPECT_STREQ(result.reason, "joined_upcoming_hard_window");
  EXPECT_DOUBLE_EQ(result.candidate_s_m, 20.0);
  EXPECT_DOUBLE_EQ(result.join_s_m, 60.0);
  EXPECT_DOUBLE_EQ(candidate[2].z_m, 15.0);
  EXPECT_GT(candidate[4].z_m, 8.0);
  EXPECT_LT(candidate[4].z_m, 15.0);
  EXPECT_DOUBLE_EQ(candidate[6].z_m, 8.0);
  EXPECT_TRUE(candidate[6].vertical_hard_window_active);
}

TEST(TrajectoryVerticalHandover, CarriesCurrentTargetWithoutUpcomingHardWindow) {
  const std::vector<TrajectoryPointSample> current = straightSamples(13.0);
  std::vector<TrajectoryPointSample> candidate = straightSamples(18.0);

  const VerticalTrajectoryHandoverResult result = reanchorTrajectoryVerticalPrefix(
      current, candidate, Point2{20.0, 0.0},
      VerticalTrajectoryHandoverState{.current_altitude_m = 12.5,
                                      .altitude_valid = true});

  ASSERT_TRUE(result.applied);
  EXPECT_STREQ(result.reason, "carry_current_target");
  for (const TrajectoryPointSample& sample : candidate) {
    EXPECT_DOUBLE_EQ(sample.z_m, 13.0);
  }
}

TEST(TrajectoryVerticalHandover, DoesNotRewriteActiveHardWindow) {
  const std::vector<TrajectoryPointSample> current = straightSamples(15.0);
  std::vector<TrajectoryPointSample> candidate = straightSamples(20.0);
  markHardWindow(candidate, 5U, 7U, 8.0);

  const VerticalTrajectoryHandoverResult result = reanchorTrajectoryVerticalPrefix(
      current, candidate, Point2{55.0, 0.0},
      VerticalTrajectoryHandoverState{.current_altitude_m = 9.0,
                                      .altitude_valid = true});

  EXPECT_FALSE(result.applied);
  EXPECT_STREQ(result.reason, "inside_hard_window");
  EXPECT_DOUBLE_EQ(candidate[5].z_m, 8.0);
}

} // namespace drone_city_nav

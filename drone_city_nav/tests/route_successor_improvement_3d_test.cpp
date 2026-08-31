#include "drone_city_nav/route_successor_improvement_3d.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "compiled_trajectory_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::shared_ptr<const CompiledTrajectory3D>
terminalGeometry(const std::vector<Point3>& points) {
  std::vector<RouteSample3D> route = sampleRoute3D(points, 0.5, 5.0);
  route.back().reference_speed_mps = 0.0;
  return makeGeometry(route, routeFingerprint(route));
}

TEST(RouteSuccessorImprovement3DTest, RequiresBothAbsoluteAndRelativeHysteresis) {
  const std::shared_ptr<const CompiledTrajectory3D> resident = terminalGeometry(
      {{0.0, 0.0, 5.0}, {0.0, 20.0, 5.0}, {20.0, 20.0, 5.0}, {20.0, 0.0, 5.0}});
  const std::shared_ptr<const CompiledTrajectory3D> candidate =
      terminalGeometry({{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}});
  ASSERT_NE(resident, nullptr);
  ASSERT_NE(candidate, nullptr);

  const RouteSuccessorImprovementAssessment3D accepted =
      assessRouteSuccessorImprovement3D(*resident, resident->route->front().station_m,
                                        *candidate, candidate->route->front().station_m,
                                        RouteSuccessorImprovementConfig3D{
                                            .minimum_absolute_improvement_s = 0.1,
                                            .minimum_relative_improvement = 0.01,
                                        });
  ASSERT_TRUE(accepted.accepted());
  EXPECT_GT(accepted.absolute_improvement_s, 0.0);
  EXPECT_GT(accepted.relative_improvement, 0.0);

  const RouteSuccessorImprovementAssessment3D absolute_rejected =
      assessRouteSuccessorImprovement3D(
          *resident, resident->route->front().station_m, *candidate,
          candidate->route->front().station_m,
          RouteSuccessorImprovementConfig3D{
              .minimum_absolute_improvement_s = accepted.absolute_improvement_s + 0.1,
              .minimum_relative_improvement = 0.01,
          });
  EXPECT_EQ(absolute_rejected.status,
            RouteSuccessorImprovementStatus3D::kInsufficientAbsoluteImprovement);

  const RouteSuccessorImprovementAssessment3D relative_rejected =
      assessRouteSuccessorImprovement3D(
          *resident, resident->route->front().station_m, *candidate,
          candidate->route->front().station_m,
          RouteSuccessorImprovementConfig3D{
              .minimum_absolute_improvement_s = 0.1,
              .minimum_relative_improvement = accepted.relative_improvement + 0.01,
          });
  EXPECT_EQ(relative_rejected.status,
            RouteSuccessorImprovementStatus3D::kInsufficientRelativeImprovement);
}

TEST(RouteSuccessorImprovement3DTest, UsesTheIndependentlyProjectedRemainders) {
  const std::shared_ptr<const CompiledTrajectory3D> resident = terminalGeometry(
      {{0.0, 0.0, 5.0}, {0.0, 20.0, 5.0}, {20.0, 20.0, 5.0}, {20.0, 0.0, 5.0}});
  const std::shared_ptr<const CompiledTrajectory3D> candidate =
      terminalGeometry({{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}});
  ASSERT_NE(resident, nullptr);
  ASSERT_NE(candidate, nullptr);

  const RouteSuccessorImprovementAssessment3D assessment =
      assessRouteSuccessorImprovement3D(*resident, 10.0, *candidate, 10.0,
                                        RouteSuccessorImprovementConfig3D{
                                            .minimum_absolute_improvement_s = 0.1,
                                            .minimum_relative_improvement = 0.01,
                                        });

  ASSERT_TRUE(assessment.accepted());
  EXPECT_LT(assessment.resident_remaining_time_s, resident->time_profile.travel_time_s);
  EXPECT_LT(assessment.candidate_remaining_time_s,
            candidate->time_profile.travel_time_s);
}

TEST(RouteSuccessorImprovement3DTest, RejectsDifferentEndpointSemantics) {
  const std::shared_ptr<const CompiledTrajectory3D> resident =
      terminalGeometry({{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}});
  ASSERT_NE(resident, nullptr);
  const std::shared_ptr<const CompiledTrajectory3D> continuation =
      withEndpointSemantics(resident, RouteEndpointSemantics3D::kContinuation);
  ASSERT_NE(continuation, nullptr);

  const RouteSuccessorImprovementAssessment3D assessment =
      assessRouteSuccessorImprovement3D(*resident, 0.0, *continuation, 0.0,
                                        RouteSuccessorImprovementConfig3D{});

  EXPECT_EQ(assessment.status,
            RouteSuccessorImprovementStatus3D::kIncomparableEndpointSemantics);
}

} // namespace
} // namespace drone_city_nav

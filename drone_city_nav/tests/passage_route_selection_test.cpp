#include "drone_city_nav/passage_route_selection.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

PassageOpening opening() {
  PassageOpening result;
  result.center = Point3{10.0, 10.0, 5.0};
  result.normal_xy = Point2{1.0, 0.0};
  result.width_m = 10.0;
  result.depth_m = 2.0;
  return result;
}

TEST(PassageRouteSelectionTest, SelectsGuideThatCrossesPortalAhead) {
  const std::array<Point2, 3> guide{Point2{0.0, 10.0}, Point2{10.0, 10.0},
                                    Point2{20.0, 10.0}};

  EXPECT_TRUE(guideCrossesPassageAhead(mppi::State{0.0F, 10.0F}, guide, opening(),
                                       PassageRouteSelectionConfig{}));
}

TEST(PassageRouteSelectionTest, RejectsNearbyPassageBesideRoute) {
  const std::array<Point2, 3> guide{Point2{0.0, 20.0}, Point2{10.0, 20.0},
                                    Point2{20.0, 20.0}};

  EXPECT_FALSE(guideCrossesPassageAhead(mppi::State{0.0F, 20.0F}, guide, opening(),
                                        PassageRouteSelectionConfig{}));
}

TEST(PassageRouteSelectionTest, RejectsCrossingBehindCurrentProgress) {
  const std::array<Point2, 4> guide{Point2{0.0, 10.0}, Point2{10.0, 10.0},
                                    Point2{20.0, 10.0}, Point2{30.0, 10.0}};

  EXPECT_FALSE(guideCrossesPassageAhead(mppi::State{21.0F, 10.0F}, guide, opening(),
                                        PassageRouteSelectionConfig{}));
}

TEST(PassageRouteSelectionTest, KeepsPassageSelectedInsideOpening) {
  const std::array<Point2, 2> guide{Point2{10.0, 10.0}, Point2{20.0, 10.0}};

  EXPECT_TRUE(guideCrossesPassageAhead(mppi::State{10.0F, 10.0F}, guide, opening(),
                                       PassageRouteSelectionConfig{}));
}

} // namespace
} // namespace drone_city_nav

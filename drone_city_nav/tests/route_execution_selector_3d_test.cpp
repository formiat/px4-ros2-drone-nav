#include <gtest/gtest.h>

#include <optional>

#include "route_execution_selector_3d.hpp"

namespace drone_city_nav {
namespace {

TEST(RouteExecutionSelector3DTest,
     CapturesResidentAuthorityAndClampsFallbackHoldWithoutNodeEffects) {
  ExecutionSupervisor3D supervisor;
  RouteExecutionSelector3D selector{supervisor, RouteExecutionSelectorConfig3D{
                                                    .flight_envelope =
                                                        FlightEnvelopeConfig{
                                                            .minimum_target_z_m = 1.0,
                                                            .maximum_target_z_m = 32.0,
                                                        },
                                                }};
  WorldSnapshot3D world;
  ProductionMppiNavigation navigation;
  navigation.state = mppi::State{.x = 4.0F, .y = 5.0F, .z = 40.0F};

  const RouteExecutionSelectorResult3D result =
      selector.select(RouteExecutionSelectorRequest3D{
          .world = &world,
          .objective = nullptr,
          .navigation = navigation,
          .execution_input = nullptr,
          .latest_raw_world = nullptr,
          .latest_lidar_evidence = nullptr,
          .validation_stamp_ns = 0,
          .minimum_tracking_sample_sequence = 0U,
          .physically_invalidated_through_generation = 0U,
          .direct_tracking_identity = std::nullopt,
          .observed_3d_world = false,
      });

  ASSERT_NE(result.selection.source_authority, nullptr);
  EXPECT_EQ(result.selection.source_snapshot, supervisor.plan());
  EXPECT_DOUBLE_EQ(result.selection.hold_position.x, 4.0);
  EXPECT_DOUBLE_EQ(result.selection.hold_position.y, 5.0);
  EXPECT_GE(result.selection.hold_position.z, 1.0);
  EXPECT_LT(result.selection.hold_position.z, 32.0);
  EXPECT_FALSE(result.selection.route_usable);
  EXPECT_TRUE(result.effects.empty());
}

TEST(RouteExecutionSelector3DTest, InvalidRequestCannotObserveOrMutateSupervisor) {
  ExecutionSupervisor3D supervisor;
  RouteExecutionSelector3D selector{supervisor, {}};
  const std::shared_ptr<const CommittedExecutionAuthority3D> before =
      supervisor.authority();

  const RouteExecutionSelectorResult3D result = selector.select({});

  EXPECT_EQ(result.selection.source_authority, nullptr);
  EXPECT_EQ(supervisor.authority(), before);
  EXPECT_TRUE(result.effects.empty());
}

} // namespace
} // namespace drone_city_nav

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "route_planner_3d.hpp"

namespace drone_city_nav {
namespace {

struct PlannerFixture3D {
  std::shared_ptr<const OccupancyGrid3D> occupancy;
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
};

[[nodiscard]] RoutePlannerConfig3D plannerConfig() {
  RoutePlannerConfig3D config;
  config.planner.minimum_horizontal_step_m = 1.0;
  config.planner.minimum_vertical_step_m = 1.0;
  config.planner.maximum_adaptive_lattice_level = 0U;
  config.planner.time_model.maximum_horizontal_speed_mps = 5.0;
  config.planner.time_model.maximum_vertical_speed_mps = 2.0;
  config.planner.goal_tolerance_m = 0.01;
  config.planner.feasibility_first_enabled = false;
  config.planner.maximum_compute_time_ms = 1000.0;
  config.planner.maximum_no_route_compute_time_ms = 1000.0;
  config.planner.maximum_expansions_per_update = 100000U;
  config.planner.physical_footprint.radius_m = 0.0;
  config.planner.physical_footprint.lower_extent_m = 0.0;
  config.planner.physical_footprint.upper_extent_m = 0.0;
  config.planner.physical_footprint.perimeter_samples = 0U;
  config.planner.physical_footprint.radial_rings = 0U;
  config.planner.physical_footprint.axial_samples = 0U;
  config.planner.physical_footprint.sweep_step_m = 0.2;
  config.planner.flight_envelope.minimum_target_z_m = 0.0;
  config.planner.flight_envelope.maximum_target_z_m = 20.0;
  config.route_sampling_step_m = 0.5;
  config.cruise_speed_mps = 5.0;
  return config;
}

[[nodiscard]] PlannerFixture3D fixture(const Point3& goal = {12.5, 12.5, 4.5},
                                       const std::uint64_t mission_epoch = 7U) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 8};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 41U);
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = occupancy->fingerprint();
  world->raw_occupied_fingerprint = occupied_fingerprint;
  world->source_occupied_fingerprint = occupied_fingerprint;
  world->grid = EsdfGrid3D{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
  };
  const std::size_t voxel_count = static_cast<std::size_t>(bounds.width_cells) *
                                  static_cast<std::size_t>(bounds.height_cells) *
                                  static_cast<std::size_t>(bounds.depth_cells);
  world->distances_m = std::make_shared<const std::vector<float>>(voxel_count, 20.0F);
  world->static_occupancy = occupancy;
  auto planner_world =
      std::make_shared<const PersistentPlannerWorld3D>(PersistentPlannerWorld3D{
          .observed_occupancy = nullptr,
          .static_occupancy = occupancy,
          .proprioceptive_free_space_seed = std::nullopt,
          .launch_support_contact = std::nullopt,
          .dirty_chunks = {},
          .producer_instance_id = occupancy->fingerprint(),
          .revision = 1U,
          .incremental_parent_revision = 0U,
          .occupied_fingerprint = occupied_fingerprint,
          .full_reset = false,
      });
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(world, planner_world,
                                     StaticRouteObjective{
                                         .goal = goal,
                                         .mission_epoch = mission_epoch,
                                         .available = true,
                                     },
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                     });
  return PlannerFixture3D{
      .occupancy = std::move(occupancy),
      .world = std::move(world),
      .planner_world = std::move(planner_world),
      .transaction = transaction,
  };
}

[[nodiscard]] RoutePlannerVehicleState3D vehicleState() {
  return RoutePlannerVehicleState3D{
      .position = Point3{1.5, 1.5, 1.5},
      .velocity = Vec3{0.0, 0.0, 0.0},
      .valid = true,
  };
}

[[nodiscard]] const RouteSearchCandidate3D&
candidate(const RoutePlannerUpdate3D& update) {
  if (!update.improved_incumbent.has_value()) {
    throw std::logic_error{"route planner update has no improved incumbent"};
  }
  return *update.improved_incumbent;
}

TEST(RoutePlanner3DTest, RejectsInvalidVehicleStateWithoutInvokingPlanner) {
  const PlannerFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  RoutePlanner3D planner{plannerConfig()};

  RoutePlannerVehicleState3D invalid = vehicleState();
  invalid.valid = false;
  const RoutePlannerUpdate3D update = planner.update(*input.transaction, invalid);

  EXPECT_EQ(update.status, RoutePlannerUpdateStatus3D::kInvalidRequest);
  EXPECT_FALSE(update.planner_invoked);
  EXPECT_EQ(update.planner_session, nullptr);
  EXPECT_FALSE(update.improved_incumbent.has_value());
}

TEST(RoutePlanner3DTest,
     OwnsOnePersistentSessionAndEmitsTypedCandidateWithRawEvidence) {
  const PlannerFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  RoutePlanner3D planner{plannerConfig()};

  const RoutePlannerUpdate3D first = planner.update(*input.transaction, vehicleState());

  EXPECT_EQ(first.status, RoutePlannerUpdateStatus3D::kUpdated);
  EXPECT_TRUE(first.planner_invoked);
  ASSERT_NE(first.planner_session, nullptr);
  ASSERT_TRUE(first.improved_incumbent.has_value());
  const RouteSearchCandidate3D& improved = candidate(first);
  EXPECT_TRUE(improved.spatial_route.valid());
  EXPECT_GE(improved.route.size(), 2U);
  EXPECT_EQ(improved.evidence.status, SegmentEvidenceStatus3D::kValid);
  EXPECT_TRUE(improved.evidence.physical_executable);
  EXPECT_EQ(improved.intent.id, first.planner_session->intent.id);

  const RoutePlannerUpdate3D continuation =
      planner.update(*input.transaction, vehicleState(), first.planner_session);
  EXPECT_EQ(continuation.status, RoutePlannerUpdateStatus3D::kUpdated);
  EXPECT_TRUE(continuation.planner_invoked);
  EXPECT_EQ(continuation.planner_session, first.planner_session);
  EXPECT_TRUE(continuation.planner_telemetry.search_state_reused);
}

TEST(RoutePlanner3DTest, RejectsAContinuationCapturedForAnotherMission) {
  const PlannerFixture3D first_input = fixture();
  const PlannerFixture3D second_input = fixture({12.5, 3.5, 4.5}, 8U);
  ASSERT_NE(first_input.transaction, nullptr);
  ASSERT_NE(second_input.transaction, nullptr);
  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerUpdate3D first =
      planner.update(*first_input.transaction, vehicleState());
  ASSERT_NE(first.planner_session, nullptr);

  const RoutePlannerUpdate3D mismatched =
      planner.update(*second_input.transaction, vehicleState(), first.planner_session);

  EXPECT_EQ(mismatched.status, RoutePlannerUpdateStatus3D::kInvalidRequest);
  EXPECT_FALSE(mismatched.planner_invoked);
  EXPECT_FALSE(mismatched.improved_incumbent.has_value());
}

TEST(RoutePlanner3DTest, ARenewedConsumerSessionReceivesTheIncumbentAgain) {
  // The first update delivers the incumbent; a continuation of the same
  // session gets improvements only, and a converged search has none. A
  // continuation that renews the consumer session is a consumer that holds no
  // route of this search: it receives the incumbent again, under a new
  // session id, and later continuations of that session are improvement-only.
  const PlannerFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerUpdate3D first = planner.update(*input.transaction, vehicleState());
  ASSERT_NE(first.planner_session, nullptr);
  ASSERT_TRUE(first.improved_incumbent.has_value());
  const std::uint64_t first_session = first.planner_session->request.session_id;
  ASSERT_NE(first_session, 0U);

  const RoutePlannerUpdate3D same_session =
      planner.update(*input.transaction, vehicleState(), first.planner_session);
  EXPECT_EQ(same_session.status, RoutePlannerUpdateStatus3D::kUpdated);
  EXPECT_FALSE(same_session.improved_incumbent.has_value());
  EXPECT_EQ(same_session.planner_session->request.session_id, first_session);

  const RoutePlannerUpdate3D renewed = planner.update(
      *input.transaction, vehicleState(), same_session.planner_session, true);
  EXPECT_EQ(renewed.status, RoutePlannerUpdateStatus3D::kUpdated);
  ASSERT_NE(renewed.planner_session, nullptr);
  EXPECT_NE(renewed.planner_session->request.session_id, first_session);
  ASSERT_TRUE(renewed.improved_incumbent.has_value());
  EXPECT_TRUE(candidate(renewed).spatial_route.valid());

  const RoutePlannerUpdate3D after =
      planner.update(*input.transaction, vehicleState(), renewed.planner_session);
  EXPECT_FALSE(after.improved_incumbent.has_value());
}

} // namespace
} // namespace drone_city_nav

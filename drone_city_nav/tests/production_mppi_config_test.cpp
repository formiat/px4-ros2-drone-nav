#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "production_mppi_config_ros.hpp"

namespace drone_city_nav {
namespace {

class ProductionMppiConfigTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  [[nodiscard]] static std::shared_ptr<rclcpp::Node>
  makeNode(const std::string& name, std::vector<rclcpp::Parameter> parameters = {}) {
    parameters.emplace_back("mission_goal_sequence_xyz_m",
                            std::vector<double>{216.0, 378.0, 18.0});
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<rclcpp::Node>(name, options);
  }
};

TEST_F(ProductionMppiConfigTest, LoadsGroupedDefaultsAndDerivedContracts) {
  const auto node = makeNode("production_mppi_config_defaults");

  const ProductionMppiConfig config = declareProductionMppiConfig(*node);

  EXPECT_TRUE(config.valid());
  EXPECT_TRUE(config.world.use_static_map);
  EXPECT_EQ(config.world.frame_id, "map");
  EXPECT_EQ(config.world.topics.raw_obstacle_snapshot_3d,
            "/drone_city_nav/raw_obstacle_snapshot_3d");
  EXPECT_DOUBLE_EQ(config.planning.tick_rate_hz, 50.0);
  EXPECT_EQ(config.planning.planner_worker_count, 4U);
  ASSERT_EQ(config.planning.mission_waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(config.planning.mission_waypoints.front().x, 216.0);
  EXPECT_DOUBLE_EQ(config.planning.mission_waypoints.front().y, 378.0);
  EXPECT_DOUBLE_EQ(config.planning.mission_waypoints.front().z, 18.0);
  EXPECT_EQ(config.control.mppi.rollouts, 8192U);
  EXPECT_EQ(config.control.mppi.steps, 120U);
  EXPECT_DOUBLE_EQ(config.control.speed_policy.horizon_duration_s, 6.0);
  EXPECT_DOUBLE_EQ(config.execution.stale_esdf_execution_window_ms, 6000.0);
  EXPECT_EQ(config.execution.stationary_hold_validity_ns, 1'000'000'000LL);
  EXPECT_EQ(config.execution.mission_goal_capture_hold_validity_ns, 2'240'000'000LL);
  EXPECT_NE(config.execution.validation_policy, nullptr);
  EXPECT_EQ(config.diagnostics.rviz_period_ns, 100'000'000LL);
  EXPECT_EQ(config.diagnostics.topics.status, "/drone_city_nav/mppi/status");
}

TEST_F(ProductionMppiConfigTest, SelectsNoStaticDerivedValuesAndTopics) {
  const auto node = makeNode(
      "production_mppi_config_no_static",
      {rclcpp::Parameter{"use_static_map", false}, rclcpp::Parameter{"dt_s", 0.1},
       rclcpp::Parameter{"no_static_horizon_duration_s", 2.0},
       rclcpp::Parameter{"no_static_stale_esdf_execution_window_s", 0.75},
       rclcpp::Parameter{"raw_obstacle_snapshot_3d_topic", "/custom/raw3d"},
       rclcpp::Parameter{"diagnostics_output_dir", "log/custom_mppi"}});

  const ProductionMppiConfig config = declareProductionMppiConfig(*node);

  EXPECT_TRUE(config.valid());
  EXPECT_FALSE(config.world.use_static_map);
  EXPECT_EQ(config.world.topics.raw_obstacle_snapshot_3d, "/custom/raw3d");
  EXPECT_EQ(config.control.mppi.steps, 20U);
  EXPECT_DOUBLE_EQ(config.control.speed_policy.horizon_duration_s, 2.0);
  EXPECT_DOUBLE_EQ(config.execution.stale_esdf_execution_window_ms, 750.0);
  EXPECT_DOUBLE_EQ(config.planning.route_tracking_policy.minimum_remaining_m, 15.0);
  EXPECT_EQ(config.diagnostics.output_dir, "log/custom_mppi");
}

TEST_F(ProductionMppiConfigTest, RejectsInvalidBoundedWorkerCount) {
  const auto node =
      makeNode("production_mppi_config_invalid_workers",
               {rclcpp::Parameter{"planner_worker_count", std::int64_t{0}}});

  EXPECT_THROW((void)declareProductionMppiConfig(*node), std::invalid_argument);
}

TEST_F(ProductionMppiConfigTest, RejectsNegativeRolloutCountBeforeConversion) {
  const auto node = makeNode("production_mppi_config_invalid_rollouts",
                             {rclcpp::Parameter{"rollouts", std::int64_t{-1}}});

  EXPECT_THROW((void)declareProductionMppiConfig(*node), std::invalid_argument);
}

TEST_F(ProductionMppiConfigTest, RejectsInvalidDerivedHorizonInputs) {
  const auto zero_dt_node =
      makeNode("production_mppi_config_zero_dt", {rclcpp::Parameter{"dt_s", 0.0}});
  const auto invalid_phase_node =
      makeNode("production_mppi_config_invalid_phase",
               {rclcpp::Parameter{"tick_rate_hz", 50.0},
                rclcpp::Parameter{"planning_tick_phase_offset_s", 0.02}});

  EXPECT_THROW((void)declareProductionMppiConfig(*zero_dt_node), std::invalid_argument);
  EXPECT_THROW((void)declareProductionMppiConfig(*invalid_phase_node),
               std::invalid_argument);
}

TEST_F(ProductionMppiConfigTest, DisablesOptionalClearanceCostsAtComposition) {
  const auto disabled_node =
      makeNode("production_mppi_config_clearance_disabled",
               {rclcpp::Parameter{"planning_exposure_weight", 7.0},
                rclcpp::Parameter{"critical_exposure_weight", 8.0},
                rclcpp::Parameter{"critical_clearance_proximity_weight", 9.0},
                rclcpp::Parameter{"obstacle_approach_weight", 10.0}});
  const auto enabled_node =
      makeNode("production_mppi_config_clearance_enabled",
               {rclcpp::Parameter{"clearance_costs_enabled", true},
                rclcpp::Parameter{"planning_exposure_weight", 7.0},
                rclcpp::Parameter{"critical_exposure_weight", 8.0},
                rclcpp::Parameter{"critical_clearance_proximity_weight", 9.0},
                rclcpp::Parameter{"obstacle_approach_weight", 10.0}});

  const ProductionMppiConfig disabled = declareProductionMppiConfig(*disabled_node);
  const ProductionMppiConfig enabled = declareProductionMppiConfig(*enabled_node);

  EXPECT_FLOAT_EQ(disabled.control.mppi.costs.planning_exposure_weight, 0.0F);
  EXPECT_FLOAT_EQ(disabled.control.mppi.costs.critical_exposure_weight, 0.0F);
  EXPECT_FLOAT_EQ(disabled.control.mppi.costs.critical_clearance_proximity_weight,
                  0.0F);
  EXPECT_FLOAT_EQ(disabled.control.mppi.costs.obstacle_approach_weight, 0.0F);
  EXPECT_FLOAT_EQ(enabled.control.mppi.costs.planning_exposure_weight, 7.0F);
  EXPECT_FLOAT_EQ(enabled.control.mppi.costs.critical_exposure_weight, 8.0F);
  EXPECT_FLOAT_EQ(enabled.control.mppi.costs.critical_clearance_proximity_weight, 9.0F);
  EXPECT_FLOAT_EQ(enabled.control.mppi.costs.obstacle_approach_weight, 10.0F);
}

} // namespace
} // namespace drone_city_nav

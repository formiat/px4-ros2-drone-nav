#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "production_mppi_route_world.hpp"
#include "route_planning_coordinator_3d.hpp"

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

struct CoordinatorInput3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
};

struct UpdateRecord3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  RoutePlannerVehicleState3D vehicle_state{};
  RoutePlannerUpdateStatus3D status{RoutePlannerUpdateStatus3D::kInvalidRequest};
  std::shared_ptr<const RoutePlannerSession3D> planner_session;
  bool planner_invoked{false};
};

struct RejectionRecord3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  RoutePlanningRejectionReason3D reason{
      RoutePlanningRejectionReason3D::kProcessingFailed};
  ProductionWorldGenerationStatus world_status{
      ProductionWorldGenerationStatus::kCoherent};
  StaticRouteSearchCurrencyAssessment currency{};
  std::exception_ptr failure;
  int world_depth{0};
};

class CoordinatorRecorder3D final {
public:
  void recordUpdate(const RoutePlanningUpdateEvent3D& event) {
    {
      const std::scoped_lock lock{mutex_};
      updates_.push_back(UpdateRecord3D{
          .transaction = event.request.transaction,
          .vehicle_state = event.vehicle_state,
          .status = event.update.status,
          .planner_session = event.update.planner_session,
          .planner_invoked = event.update.planner_invoked,
      });
    }
    condition_.notify_all();
  }

  void recordRejection(const RoutePlanningRejection3D& rejection) {
    {
      const std::scoped_lock lock{mutex_};
      rejections_.push_back(RejectionRecord3D{
          .transaction = rejection.request.transaction,
          .reason = rejection.reason,
          .world_status = rejection.world_status,
          .currency = rejection.currency,
          .failure = rejection.failure,
          .world_depth = rejection.world_depth,
      });
    }
    condition_.notify_all();
  }

  void recordFailure(const std::exception_ptr failure) {
    {
      const std::scoped_lock lock{mutex_};
      failures_.push_back(failure);
    }
    condition_.notify_all();
  }

  [[nodiscard]] bool waitForUpdates(const std::size_t count) {
    std::unique_lock lock{mutex_};
    return condition_.wait_for(lock, 10s,
                               [this, count]() { return updates_.size() >= count; });
  }

  [[nodiscard]] bool waitForRejections(const std::size_t count) {
    std::unique_lock lock{mutex_};
    return condition_.wait_for(lock, 10s,
                               [this, count]() { return rejections_.size() >= count; });
  }

  [[nodiscard]] bool waitForFailures(const std::size_t count) {
    std::unique_lock lock{mutex_};
    return condition_.wait_for(lock, 10s,
                               [this, count]() { return failures_.size() >= count; });
  }

  [[nodiscard]] std::vector<UpdateRecord3D> updates() const {
    const std::scoped_lock lock{mutex_};
    return updates_;
  }

  [[nodiscard]] std::vector<RejectionRecord3D> rejections() const {
    const std::scoped_lock lock{mutex_};
    return rejections_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<UpdateRecord3D> updates_;
  std::vector<RejectionRecord3D> rejections_;
  std::vector<std::exception_ptr> failures_;
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

[[nodiscard]] RoutePlannerVehicleState3D vehicleState() {
  return RoutePlannerVehicleState3D{
      .position = Point3{1.5, 1.5, 1.5},
      .velocity = Vec3{0.0, 0.0, 0.0},
      .valid = true,
  };
}

[[nodiscard]] CoordinatorInput3D
coordinatorInput(const std::uint64_t mission_epoch = 7U, const int depth = 8,
                 const bool coherent_generation = true) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, depth};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 41U + mission_epoch);
  const std::uint64_t revision = occupancy->fingerprint();
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = occupied_fingerprint;
  world->raw_occupied_fingerprint = occupied_fingerprint;
  world->grid = mppi::EsdfGrid{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
      .outside_is_unknown = false,
  };
  const std::size_t voxel_count = static_cast<std::size_t>(bounds.width_cells) *
                                  static_cast<std::size_t>(bounds.height_cells) *
                                  static_cast<std::size_t>(bounds.depth_cells);
  world->distances_m = std::make_shared<const std::vector<float>>(voxel_count, 20.0F);
  world->static_occupancy = occupancy;
  world->local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map = {.producer_instance_id = 0U,
                  .base_snapshot_revision = revision,
                  .revision = revision},
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  if (!coherent_generation) {
    world->local_world_generation.generation = 0U;
  }
  std::shared_ptr<const WorldSnapshot3D> immutable_world = std::move(world);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(immutable_world, planner_world,
                                     StaticRouteObjective{
                                         .goal = {12.5, 12.5, depth > 1 ? 4.5 : 0.5},
                                         .mission_epoch = mission_epoch,
                                         .available = true,
                                     },
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                         .base_route_generation = 0U,
                                     });
  return CoordinatorInput3D{
      .world = std::move(immutable_world),
      .planner_world = planner_world,
      .transaction = transaction,
  };
}

[[nodiscard]] RoutePlanningRequest3D requestFor(const CoordinatorInput3D& input) {
  return RoutePlanningRequest3D{
      .transaction = input.transaction,
      .world_telemetry = {},
      .continuation_session = nullptr,
  };
}

[[nodiscard]] RoutePlanningCoordinatorConfig3D
coordinatorConfig(CoordinatorRecorder3D& recorder) {
  return RoutePlanningCoordinatorConfig3D{
      .planner = plannerConfig(),
      .vehicle_state_provider = []() { return vehicleState(); },
      .resident_route_generation_provider = []() { return 0U; },
      .update_handler =
          [&recorder](const RoutePlanningUpdateEvent3D event) {
            recorder.recordUpdate(event);
          },
      .rejection_handler =
          [&recorder](const RoutePlanningRejection3D& rejection) {
            recorder.recordRejection(rejection);
          },
      .failure_handler =
          [&recorder](const std::exception_ptr failure) {
            recorder.recordFailure(failure);
          },
  };
}

TEST(RoutePlanningCoordinator3DTest,
     RejectsInvalidOrStoppedSubmissionsWithoutRunningWorker) {
  CoordinatorRecorder3D recorder;
  RoutePlanningCoordinator3D coordinator{coordinatorConfig(recorder)};
  const CoordinatorInput3D input = coordinatorInput();
  ASSERT_NE(input.transaction, nullptr);

  EXPECT_EQ(coordinator.enqueue({}).status,
            RoutePlanningEnqueueStatus3D::kInvalidRequest);
  EXPECT_EQ(coordinator.enqueue(requestFor(input)).status,
            RoutePlanningEnqueueStatus3D::kStopped);
  coordinator.start();
  coordinator.stop();
  EXPECT_EQ(coordinator.enqueue(requestFor(input)).status,
            RoutePlanningEnqueueStatus3D::kStopped);

  const RoutePlanningCoordinatorStatistics3D statistics = coordinator.statistics();
  EXPECT_EQ(statistics.invalid_rejections, 1U);
  EXPECT_EQ(statistics.stopped_rejections, 2U);
  EXPECT_EQ(statistics.queued, 0U);
}

TEST(RoutePlanningCoordinator3DTest,
     AppliesKeepAndReplacePoliciesToTheSinglePendingRequest) {
  CoordinatorRecorder3D recorder;
  RoutePlanningCoordinatorConfig3D config = coordinatorConfig(recorder);
  std::mutex gate_mutex;
  std::condition_variable gate_condition;
  bool provider_entered{false};
  bool provider_released{false};
  std::atomic<std::size_t> provider_calls{0U};
  config.vehicle_state_provider = [&]() {
    if (provider_calls.fetch_add(1U, std::memory_order_relaxed) == 0U) {
      std::unique_lock lock{gate_mutex};
      provider_entered = true;
      gate_condition.notify_all();
      static_cast<void>(gate_condition.wait_for(
          lock, 10s, [&provider_released]() { return provider_released; }));
    }
    return vehicleState();
  };
  RoutePlanningCoordinator3D coordinator{std::move(config)};
  const CoordinatorInput3D first = coordinatorInput(7U);
  const CoordinatorInput3D pending = coordinatorInput(8U);
  const CoordinatorInput3D retained = coordinatorInput(9U);
  const CoordinatorInput3D replacement = coordinatorInput(10U);
  ASSERT_NE(first.transaction, nullptr);
  ASSERT_NE(pending.transaction, nullptr);
  ASSERT_NE(retained.transaction, nullptr);
  ASSERT_NE(replacement.transaction, nullptr);

  coordinator.start();
  ASSERT_TRUE(coordinator.enqueue(requestFor(first)).queued());
  bool entered{false};
  {
    std::unique_lock lock{gate_mutex};
    entered = gate_condition.wait_for(
        lock, 10s, [&provider_entered]() { return provider_entered; });
  }
  if (!entered) {
    {
      const std::scoped_lock lock{gate_mutex};
      provider_released = true;
    }
    gate_condition.notify_all();
    coordinator.stop();
    FAIL() << "vehicle provider was not entered";
  }

  EXPECT_TRUE(coordinator.enqueue(requestFor(pending)).queued());
  EXPECT_TRUE(coordinator.pending());
  const RoutePlanningEnqueueResult3D keep = coordinator.enqueue(requestFor(retained));
  EXPECT_EQ(keep.status, RoutePlanningEnqueueStatus3D::kPendingRequestRetained);
  EXPECT_FALSE(keep.displaced.has_value());
  const RoutePlanningEnqueueResult3D replace = coordinator.enqueue(
      requestFor(replacement), RoutePlanningQueuePolicy3D::kReplacePending);
  ASSERT_TRUE(replace.queued());
  ASSERT_TRUE(replace.displaced.has_value());
  const RoutePlanningRequest3D displaced =
      replace.displaced.value_or(RoutePlanningRequest3D{});
  EXPECT_EQ(displaced.transaction, pending.transaction);
  EXPECT_TRUE(replace.lifecycleTransferredTo(*replacement.transaction));
  {
    const std::scoped_lock lock{gate_mutex};
    provider_released = true;
  }
  gate_condition.notify_all();

  EXPECT_TRUE(recorder.waitForUpdates(2U));
  coordinator.stop();
  const std::vector<UpdateRecord3D> updates = recorder.updates();
  ASSERT_EQ(updates.size(), 2U);
  EXPECT_EQ(updates[0].transaction, first.transaction);
  EXPECT_EQ(updates[1].transaction, replacement.transaction);
  EXPECT_TRUE(updates[0].planner_invoked);
  EXPECT_TRUE(updates[1].planner_invoked);
  const RoutePlanningCoordinatorStatistics3D statistics = coordinator.statistics();
  EXPECT_EQ(statistics.queued, 3U);
  EXPECT_EQ(statistics.processed, 2U);
  EXPECT_EQ(statistics.displaced, 1U);
  EXPECT_EQ(statistics.busy_rejections, 1U);
  EXPECT_EQ(provider_calls.load(std::memory_order_relaxed), 2U);
}

TEST(RoutePlanningCoordinator3DTest,
     RejectsInvalidWorldDepthCurrencyAndVehicleWithTypedReasons) {
  CoordinatorRecorder3D recorder;
  RoutePlanningCoordinatorConfig3D config = coordinatorConfig(recorder);
  std::atomic<std::uint64_t> resident_generation{0U};
  std::atomic<bool> vehicle_valid{true};
  config.resident_route_generation_provider = [&resident_generation]() {
    return resident_generation.load(std::memory_order_relaxed);
  };
  config.vehicle_state_provider = [&vehicle_valid]() {
    RoutePlannerVehicleState3D state = vehicleState();
    state.valid = vehicle_valid.load(std::memory_order_relaxed);
    return state;
  };
  RoutePlanningCoordinator3D coordinator{std::move(config)};
  const CoordinatorInput3D incoherent = coordinatorInput(7U, 8, false);
  const CoordinatorInput3D flat = coordinatorInput(8U, 1, true);
  const CoordinatorInput3D superseded = coordinatorInput(9U);
  const CoordinatorInput3D unavailable = coordinatorInput(10U);
  ASSERT_NE(incoherent.transaction, nullptr);
  ASSERT_NE(flat.transaction, nullptr);
  ASSERT_NE(superseded.transaction, nullptr);
  ASSERT_NE(unavailable.transaction, nullptr);

  coordinator.start();
  ASSERT_TRUE(coordinator.enqueue(requestFor(incoherent)).queued());
  ASSERT_TRUE(recorder.waitForRejections(1U));
  ASSERT_TRUE(coordinator.enqueue(requestFor(flat)).queued());
  ASSERT_TRUE(recorder.waitForRejections(2U));
  resident_generation.store(1U, std::memory_order_relaxed);
  ASSERT_TRUE(coordinator.enqueue(requestFor(superseded)).queued());
  ASSERT_TRUE(recorder.waitForRejections(3U));
  resident_generation.store(0U, std::memory_order_relaxed);
  vehicle_valid.store(false, std::memory_order_relaxed);
  ASSERT_TRUE(coordinator.enqueue(requestFor(unavailable)).queued());
  ASSERT_TRUE(recorder.waitForRejections(4U));
  coordinator.stop();

  const std::vector<RejectionRecord3D> rejections = recorder.rejections();
  ASSERT_EQ(rejections.size(), 4U);
  EXPECT_EQ(rejections[0].reason,
            RoutePlanningRejectionReason3D::kInvalidWorldGeneration);
  EXPECT_EQ(rejections[0].world_status,
            ProductionWorldGenerationStatus::kInvalidGeneration);
  EXPECT_EQ(rejections[1].reason,
            RoutePlanningRejectionReason3D::kFullThreeDimensionalWorldRequired);
  EXPECT_EQ(rejections[1].world_depth, 1);
  EXPECT_EQ(rejections[2].reason,
            RoutePlanningRejectionReason3D::kSupersededRouteGeneration);
  EXPECT_EQ(rejections[2].currency.status,
            StaticRouteSearchCurrencyStatus::kSupersededByResidentRoute);
  EXPECT_EQ(rejections[3].reason,
            RoutePlanningRejectionReason3D::kVehicleStateUnavailable);
  EXPECT_EQ(coordinator.statistics().processed, 0U);
}

TEST(RoutePlanningCoordinator3DTest,
     IsolatesProcessingAndUpdateHandlerFailuresAndContinues) {
  CoordinatorRecorder3D recorder;
  RoutePlanningCoordinatorConfig3D config = coordinatorConfig(recorder);
  std::atomic<std::size_t> provider_calls{0U};
  std::atomic<std::size_t> handler_calls{0U};
  config.resident_route_generation_provider = [&provider_calls]() {
    if (provider_calls.fetch_add(1U, std::memory_order_relaxed) == 0U) {
      throw std::runtime_error{"injected request provider failure"};
    }
    return 0U;
  };
  config.update_handler = [&recorder,
                           &handler_calls](const RoutePlanningUpdateEvent3D event) {
    if (handler_calls.fetch_add(1U, std::memory_order_relaxed) == 0U) {
      throw std::runtime_error{"injected update handler failure"};
    }
    recorder.recordUpdate(event);
  };
  RoutePlanningCoordinator3D coordinator{std::move(config)};
  const CoordinatorInput3D processing_failed = coordinatorInput(7U);
  const CoordinatorInput3D handler_failed = coordinatorInput(8U);
  const CoordinatorInput3D recovered = coordinatorInput(9U);
  ASSERT_NE(processing_failed.transaction, nullptr);
  ASSERT_NE(handler_failed.transaction, nullptr);
  ASSERT_NE(recovered.transaction, nullptr);

  coordinator.start();
  ASSERT_TRUE(coordinator.enqueue(requestFor(processing_failed)).queued());
  ASSERT_TRUE(recorder.waitForRejections(1U));
  ASSERT_TRUE(recorder.waitForFailures(1U));
  ASSERT_TRUE(coordinator.enqueue(requestFor(handler_failed)).queued());
  ASSERT_TRUE(recorder.waitForRejections(2U));
  ASSERT_TRUE(recorder.waitForFailures(2U));
  ASSERT_TRUE(coordinator.enqueue(requestFor(recovered)).queued());
  ASSERT_TRUE(recorder.waitForUpdates(1U));
  coordinator.stop();

  const std::vector<RejectionRecord3D> rejections = recorder.rejections();
  ASSERT_EQ(rejections.size(), 2U);
  EXPECT_EQ(rejections[0].reason, RoutePlanningRejectionReason3D::kProcessingFailed);
  EXPECT_NE(rejections[0].failure, nullptr);
  EXPECT_EQ(rejections[1].reason, RoutePlanningRejectionReason3D::kUpdateHandlerFailed);
  EXPECT_NE(rejections[1].failure, nullptr);
  const RoutePlanningCoordinatorStatistics3D statistics = coordinator.statistics();
  EXPECT_EQ(statistics.processed, 1U);
  EXPECT_EQ(statistics.processing_failures, 1U);
  EXPECT_EQ(statistics.handler_failures, 1U);
}

TEST(RoutePlanningCoordinator3DTest,
     AllowsReentrantContinuationSubmissionAndCleanRestart) {
  CoordinatorRecorder3D recorder;
  RoutePlanningCoordinatorConfig3D config = coordinatorConfig(recorder);
  RoutePlanningCoordinator3D* coordinator_pointer{nullptr};
  std::atomic<std::size_t> handler_calls{0U};
  std::atomic<RoutePlanningEnqueueStatus3D> continuation_status{
      RoutePlanningEnqueueStatus3D::kInvalidRequest};
  config.update_handler = [&](const RoutePlanningUpdateEvent3D event) {
    recorder.recordUpdate(event);
    if (handler_calls.fetch_add(1U, std::memory_order_relaxed) == 0U) {
      continuation_status.store(
          coordinator_pointer
              ->enqueue(RoutePlanningRequest3D{
                  .transaction = event.request.transaction,
                  .world_telemetry = event.request.world_telemetry,
                  .continuation_session = event.update.planner_session,
              })
              .status,
          std::memory_order_relaxed);
    }
  };
  RoutePlanningCoordinator3D coordinator{std::move(config)};
  coordinator_pointer = &coordinator;
  const CoordinatorInput3D first = coordinatorInput(7U);
  const CoordinatorInput3D restarted = coordinatorInput(8U);
  ASSERT_NE(first.transaction, nullptr);
  ASSERT_NE(restarted.transaction, nullptr);

  coordinator.start();
  ASSERT_TRUE(coordinator.enqueue(requestFor(first)).queued());
  ASSERT_TRUE(recorder.waitForUpdates(2U));
  coordinator.stop();
  EXPECT_EQ(continuation_status.load(std::memory_order_relaxed),
            RoutePlanningEnqueueStatus3D::kQueued);

  coordinator.start();
  ASSERT_TRUE(coordinator.enqueue(requestFor(restarted)).queued());
  ASSERT_TRUE(recorder.waitForUpdates(3U));
  coordinator.stop();
  const RoutePlanningCoordinatorStatistics3D statistics = coordinator.statistics();
  EXPECT_EQ(statistics.queued, 3U);
  EXPECT_EQ(statistics.processed, 3U);
  EXPECT_FALSE(coordinator.pending());
}

} // namespace
} // namespace drone_city_nav

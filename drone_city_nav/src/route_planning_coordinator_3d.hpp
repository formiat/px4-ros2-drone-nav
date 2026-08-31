#pragma once

#include "drone_city_nav/static_route_extension.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string_view>
#include <thread>

#include "production_mppi_route_world.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_planner_3d.hpp"

namespace drone_city_nav {

struct RoutePlanningRequest3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  std::shared_ptr<const RoutePlannerSession3D> continuation_session;

  [[nodiscard]] bool valid() const noexcept {
    return transaction != nullptr && transaction->valid();
  }
};

enum class RoutePlanningQueuePolicy3D : std::uint8_t {
  kKeepPending,
  kReplacePending,
};

enum class RoutePlanningEnqueueStatus3D : std::uint8_t {
  kQueued,
  kInvalidRequest,
  kPendingRequestRetained,
  kStopped,
};

[[nodiscard]] std::string_view
routePlanningEnqueueStatus3DName(RoutePlanningEnqueueStatus3D status) noexcept;

struct RoutePlanningEnqueueResult3D {
  RoutePlanningEnqueueStatus3D status{RoutePlanningEnqueueStatus3D::kInvalidRequest};
  std::optional<RoutePlanningRequest3D> displaced;

  [[nodiscard]] bool queued() const noexcept {
    return status == RoutePlanningEnqueueStatus3D::kQueued;
  }

  [[nodiscard]] bool
  lifecycleTransferredTo(const PlannerSearchTransaction3D& replacement) const noexcept;
};

enum class RoutePlanningRejectionReason3D : std::uint8_t {
  kInvalidWorldGeneration,
  kFullThreeDimensionalWorldRequired,
  kSupersededRouteGeneration,
  kVehicleStateUnavailable,
  kProcessingFailed,
  kUpdateHandlerFailed,
};

[[nodiscard]] std::string_view
routePlanningRejectionReason3DName(RoutePlanningRejectionReason3D reason) noexcept;

struct RoutePlanningRejection3D {
  RoutePlanningRequest3D request{};
  RoutePlanningRejectionReason3D reason{
      RoutePlanningRejectionReason3D::kProcessingFailed};
  ProductionWorldGenerationStatus world_status{
      ProductionWorldGenerationStatus::kCoherent};
  StaticRouteSearchCurrencyAssessment currency{};
  std::exception_ptr failure;
  int world_depth{0};
};

struct RoutePlanningUpdateEvent3D {
  RoutePlanningRequest3D request{};
  RoutePlannerVehicleState3D vehicle_state{};
  RoutePlannerUpdate3D update{};
};

struct RoutePlanningCoordinatorStatistics3D {
  std::uint64_t queued{0U};
  std::uint64_t processed{0U};
  std::uint64_t displaced{0U};
  std::uint64_t busy_rejections{0U};
  std::uint64_t invalid_rejections{0U};
  std::uint64_t stopped_rejections{0U};
  std::uint64_t processing_failures{0U};
  std::uint64_t handler_failures{0U};
};

struct RoutePlanningCoordinatorConfig3D {
  RoutePlannerConfig3D planner{};
  std::function<RoutePlannerVehicleState3D()> vehicle_state_provider;
  std::function<std::uint64_t()> resident_route_generation_provider;
  std::function<void(RoutePlanningUpdateEvent3D)> update_handler;
  std::function<void(const RoutePlanningRejection3D&)> rejection_handler;
  std::function<void(std::exception_ptr)> failure_handler;
};

class RoutePlanningCoordinator3D final {
public:
  explicit RoutePlanningCoordinator3D(RoutePlanningCoordinatorConfig3D config);
  ~RoutePlanningCoordinator3D();

  RoutePlanningCoordinator3D(const RoutePlanningCoordinator3D&) = delete;
  RoutePlanningCoordinator3D& operator=(const RoutePlanningCoordinator3D&) = delete;
  RoutePlanningCoordinator3D(RoutePlanningCoordinator3D&&) = delete;
  RoutePlanningCoordinator3D& operator=(RoutePlanningCoordinator3D&&) = delete;

  void start();
  void stop() noexcept;

  [[nodiscard]] RoutePlanningEnqueueResult3D
  enqueue(RoutePlanningRequest3D request,
          RoutePlanningQueuePolicy3D policy = RoutePlanningQueuePolicy3D::kKeepPending);
  [[nodiscard]] std::optional<RoutePlanningRequest3D> cancelPending();
  [[nodiscard]] bool pending() const noexcept;
  [[nodiscard]] RoutePlanningCoordinatorStatistics3D statistics() const noexcept;

private:
  void run(std::stop_token stop_token) noexcept;
  void reject(RoutePlanningRejection3D rejection) noexcept;
  void handleFailure(std::exception_ptr failure) noexcept;

  RoutePlanningCoordinatorConfig3D config_{};
  RoutePlanner3D planner_;

  mutable std::mutex lifecycle_mutex_;
  bool accepting_{false};
  bool stopping_{false};
  std::jthread worker_;

  mutable std::mutex queue_mutex_;
  std::condition_variable_any queue_condition_;
  std::optional<RoutePlanningRequest3D> pending_request_;

  std::atomic<std::uint64_t> queued_{0U};
  std::atomic<std::uint64_t> processed_{0U};
  std::atomic<std::uint64_t> displaced_{0U};
  std::atomic<std::uint64_t> busy_rejections_{0U};
  std::atomic<std::uint64_t> invalid_rejections_{0U};
  std::atomic<std::uint64_t> stopped_rejections_{0U};
  std::atomic<std::uint64_t> processing_failures_{0U};
  std::atomic<std::uint64_t> handler_failures_{0U};
};

} // namespace drone_city_nav

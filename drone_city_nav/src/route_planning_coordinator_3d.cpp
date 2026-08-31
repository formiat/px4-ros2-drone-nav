#include "route_planning_coordinator_3d.hpp"

#include <stdexcept>
#include <utility>

namespace drone_city_nav {

std::string_view
routePlanningEnqueueStatus3DName(const RoutePlanningEnqueueStatus3D status) noexcept {
  switch (status) {
    case RoutePlanningEnqueueStatus3D::kQueued:
      return "queued";
    case RoutePlanningEnqueueStatus3D::kInvalidRequest:
      return "invalid_request";
    case RoutePlanningEnqueueStatus3D::kPendingRequestRetained:
      return "pending_request_retained";
    case RoutePlanningEnqueueStatus3D::kStopped:
      return "stopped";
  }
  return "unknown";
}

bool RoutePlanningEnqueueResult3D::lifecycleTransferredTo(
    const PlannerSearchTransaction3D& replacement) const noexcept {
  if (!displaced.has_value() || displaced->transaction == nullptr) {
    return false;
  }
  const StaticRouteSearchRequestIdentity& previous = displaced->transaction->request;
  return previous.kind == replacement.request.kind &&
         previous.base_route_generation == replacement.request.base_route_generation;
}

std::string_view routePlanningRejectionReason3DName(
    const RoutePlanningRejectionReason3D reason) noexcept {
  switch (reason) {
    case RoutePlanningRejectionReason3D::kInvalidWorldGeneration:
      return "invalid_world_generation";
    case RoutePlanningRejectionReason3D::kFullThreeDimensionalWorldRequired:
      return "full_3d_world_required";
    case RoutePlanningRejectionReason3D::kSupersededRouteGeneration:
      return "superseded_route_generation";
    case RoutePlanningRejectionReason3D::kVehicleStateUnavailable:
      return "vehicle_state_unavailable";
    case RoutePlanningRejectionReason3D::kProcessingFailed:
      return "processing_failed";
    case RoutePlanningRejectionReason3D::kUpdateHandlerFailed:
      return "update_handler_failed";
  }
  return "unknown";
}

RoutePlanningCoordinator3D::RoutePlanningCoordinator3D(
    RoutePlanningCoordinatorConfig3D config)
    : config_{std::move(config)},
      planner_{config_.planner} {
  if (!config_.vehicle_state_provider || !config_.resident_route_generation_provider ||
      !config_.update_handler || !config_.rejection_handler) {
    throw std::invalid_argument{"route planning coordinator ports are incomplete"};
  }
}

RoutePlanningCoordinator3D::~RoutePlanningCoordinator3D() {
  stop();
}

void RoutePlanningCoordinator3D::start() {
  const std::scoped_lock lock{lifecycle_mutex_};
  if (accepting_) {
    return;
  }
  if (stopping_ || worker_.joinable()) {
    throw std::logic_error{"route planning coordinator worker was not joined"};
  }
  accepting_ = true;
  try {
    worker_ =
        std::jthread{[this](const std::stop_token stop_token) { run(stop_token); }};
  } catch (...) {
    accepting_ = false;
    throw;
  }
}

void RoutePlanningCoordinator3D::stop() noexcept {
  std::jthread worker;
  {
    const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
    if (stopping_) {
      return;
    }
    accepting_ = false;
    stopping_ = true;
    worker = std::move(worker_);
  }
  if (worker.joinable()) {
    worker.request_stop();
    queue_condition_.notify_all();
    worker.join();
  }
  {
    const std::scoped_lock queue_lock{queue_mutex_};
    pending_request_.reset();
  }
  const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
  stopping_ = false;
}

RoutePlanningEnqueueResult3D
RoutePlanningCoordinator3D::enqueue(RoutePlanningRequest3D request,
                                    const RoutePlanningQueuePolicy3D policy) {
  RoutePlanningEnqueueResult3D result;
  if (!request.valid()) {
    invalid_rejections_.fetch_add(1U, std::memory_order_relaxed);
    return result;
  }
  const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
  if (!accepting_) {
    stopped_rejections_.fetch_add(1U, std::memory_order_relaxed);
    result.status = RoutePlanningEnqueueStatus3D::kStopped;
    return result;
  }
  {
    const std::scoped_lock queue_lock{queue_mutex_};
    if (pending_request_.has_value()) {
      if (policy == RoutePlanningQueuePolicy3D::kKeepPending) {
        busy_rejections_.fetch_add(1U, std::memory_order_relaxed);
        result.status = RoutePlanningEnqueueStatus3D::kPendingRequestRetained;
        return result;
      }
      result.displaced = std::move(pending_request_);
      displaced_.fetch_add(1U, std::memory_order_relaxed);
    }
    pending_request_ = std::move(request);
  }
  queued_.fetch_add(1U, std::memory_order_relaxed);
  result.status = RoutePlanningEnqueueStatus3D::kQueued;
  queue_condition_.notify_all();
  return result;
}

std::optional<RoutePlanningRequest3D> RoutePlanningCoordinator3D::cancelPending() {
  const std::scoped_lock lock{queue_mutex_};
  return std::exchange(pending_request_, std::nullopt);
}

bool RoutePlanningCoordinator3D::pending() const noexcept {
  const std::scoped_lock lock{queue_mutex_};
  return pending_request_.has_value();
}

RoutePlanningCoordinatorStatistics3D
RoutePlanningCoordinator3D::statistics() const noexcept {
  return RoutePlanningCoordinatorStatistics3D{
      .queued = queued_.load(std::memory_order_relaxed),
      .processed = processed_.load(std::memory_order_relaxed),
      .displaced = displaced_.load(std::memory_order_relaxed),
      .busy_rejections = busy_rejections_.load(std::memory_order_relaxed),
      .invalid_rejections = invalid_rejections_.load(std::memory_order_relaxed),
      .stopped_rejections = stopped_rejections_.load(std::memory_order_relaxed),
      .processing_failures = processing_failures_.load(std::memory_order_relaxed),
      .handler_failures = handler_failures_.load(std::memory_order_relaxed),
  };
}

void RoutePlanningCoordinator3D::run(const std::stop_token stop_token) noexcept {
  while (!stop_token.stop_requested()) {
    std::optional<RoutePlanningRequest3D> request;
    {
      std::unique_lock lock{queue_mutex_};
      queue_condition_.wait(lock, stop_token,
                            [this]() { return pending_request_.has_value(); });
      if (stop_token.stop_requested()) {
        return;
      }
      request = std::exchange(pending_request_, std::nullopt);
    }
    if (!request.has_value() || !request->valid()) {
      continue;
    }
    const std::shared_ptr<const PlannerSearchTransaction3D>& transaction =
        request->transaction;
    const ProductionWorldGenerationStatus world_status =
        assessProductionWorldGeneration(*transaction->world);
    if (world_status != ProductionWorldGenerationStatus::kCoherent) {
      reject(RoutePlanningRejection3D{
          .request = *request,
          .reason = RoutePlanningRejectionReason3D::kInvalidWorldGeneration,
          .world_status = world_status,
          .currency = {},
          .failure = {},
          .world_depth = 0,
      });
      continue;
    }
    if (transaction->world->grid.depth <= 1) {
      reject(RoutePlanningRejection3D{
          .request = *request,
          .reason = RoutePlanningRejectionReason3D::kFullThreeDimensionalWorldRequired,
          .world_status = world_status,
          .currency = {},
          .failure = {},
          .world_depth = transaction->world->grid.depth,
      });
      continue;
    }

    try {
      const std::uint64_t resident_route_generation =
          config_.resident_route_generation_provider();
      const StaticRouteSearchCurrencyAssessment currency =
          assessStaticRouteSearchCurrency(transaction->request,
                                          resident_route_generation);
      if (request->continuation_session == nullptr && !currency.current()) {
        reject(RoutePlanningRejection3D{
            .request = *request,
            .reason = RoutePlanningRejectionReason3D::kSupersededRouteGeneration,
            .world_status = world_status,
            .currency = currency,
            .failure = {},
            .world_depth = 0,
        });
        continue;
      }
      const RoutePlannerVehicleState3D vehicle_state = config_.vehicle_state_provider();
      if (!vehicle_state.valid) {
        reject(RoutePlanningRejection3D{
            .request = *request,
            .reason = RoutePlanningRejectionReason3D::kVehicleStateUnavailable,
            .world_status = world_status,
            .currency = currency,
            .failure = {},
            .world_depth = 0,
        });
        continue;
      }
      RoutePlannerUpdate3D update =
          planner_.update(*transaction, vehicle_state, request->continuation_session);
      try {
        config_.update_handler(RoutePlanningUpdateEvent3D{
            .request = *request,
            .vehicle_state = vehicle_state,
            .update = std::move(update),
        });
        processed_.fetch_add(1U, std::memory_order_relaxed);
      } catch (...) {
        const std::exception_ptr failure = std::current_exception();
        handler_failures_.fetch_add(1U, std::memory_order_relaxed);
        handleFailure(failure);
        reject(RoutePlanningRejection3D{
            .request = *request,
            .reason = RoutePlanningRejectionReason3D::kUpdateHandlerFailed,
            .world_status = world_status,
            .currency = currency,
            .failure = failure,
            .world_depth = 0,
        });
      }
    } catch (...) {
      const std::exception_ptr failure = std::current_exception();
      processing_failures_.fetch_add(1U, std::memory_order_relaxed);
      handleFailure(failure);
      reject(RoutePlanningRejection3D{
          .request = *request,
          .reason = RoutePlanningRejectionReason3D::kProcessingFailed,
          .world_status = world_status,
          .currency = {},
          .failure = failure,
          .world_depth = 0,
      });
    }
  }
}

void RoutePlanningCoordinator3D::reject(RoutePlanningRejection3D rejection) noexcept {
  try {
    config_.rejection_handler(rejection);
  } catch (...) {
    handler_failures_.fetch_add(1U, std::memory_order_relaxed);
    handleFailure(std::current_exception());
  }
}

void RoutePlanningCoordinator3D::handleFailure(
    const std::exception_ptr failure) noexcept {
  if (!config_.failure_handler) {
    return;
  }
  try {
    config_.failure_handler(failure);
  } catch (...) {
    handler_failures_.fetch_add(1U, std::memory_order_relaxed);
  }
}

} // namespace drone_city_nav

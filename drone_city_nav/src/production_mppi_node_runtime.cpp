#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <span>
#include <utility>

#include "navigation_diagnostics_sink.hpp"
#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first, const mppi::State& second) {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

[[nodiscard]] mppi::State interpolateState(const mppi::State& first,
                                           const mppi::State& second,
                                           const double ratio) {
  const float clamped = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  return mppi::State{
      .x = std::lerp(first.x, second.x, clamped),
      .y = std::lerp(first.y, second.y, clamped),
      .z = std::lerp(first.z, second.z, clamped),
      .vx = std::lerp(first.vx, second.vx, clamped),
      .vy = std::lerp(first.vy, second.vy, clamped),
      .vz = std::lerp(first.vz, second.vz, clamped),
      .yaw = std::lerp(first.yaw, second.yaw, clamped),
      .yaw_rate = std::lerp(first.yaw_rate, second.yaw_rate, clamped),
  };
}

[[nodiscard]] mppi::State sampleState(const std::span<const mppi::State> states,
                                      const double offset_steps) {
  if (states.empty()) {
    return {};
  }
  const double source =
      std::clamp(offset_steps, 0.0, static_cast<double>(states.size() - 1U));
  const std::size_t lower = static_cast<std::size_t>(std::floor(source));
  const std::size_t upper = std::min(lower + 1U, states.size() - 1U);
  return interpolateState(states[lower], states[upper],
                          source - static_cast<double>(lower));
}

} // namespace

void ProductionMppiNode::routePlanningWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::optional<ProductionRoutePlanningWork3D> work;
    {
      std::unique_lock lock{route_planning_queue_mutex_};
      route_planning_queue_condition_.wait(lock, stop_token, [this]() {
        return pending_route_planning_work_.has_value();
      });
      if (stop_token.stop_requested()) {
        return;
      }
      work = std::exchange(pending_route_planning_work_, std::nullopt);
    }
    if (!work || !work->valid()) {
      continue;
    }
    const std::shared_ptr<const PlannerSearchTransaction3D>& transaction =
        work->transaction;
    if (!productionWorldGenerationCoherent(*transaction->world)) {
      const ProductionWorldGenerationStatus status =
          assessProductionWorldGeneration(*transaction->world);
      const std::string_view status_name = productionWorldGenerationStatusName(status);
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=%.*s",
                   transaction->world->local_world_generation.generation,
                   static_cast<int>(status_name.size()), status_name.data());
      finishStaticRouteSearch(*transaction);
      continue;
    }
    if (transaction->world->grid.depth <= 1) {
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE rejected local_world_generation=%" PRIu64
                   " reason=full_3d_world_required depth=%d",
                   transaction->world->local_world_generation.generation,
                   transaction->world->grid.depth);
      finishStaticRouteSearch(*transaction);
      continue;
    }

    const StaticRouteSearchRequestIdentity& request = transaction->request;
    std::uint64_t resident_route_generation = 0U;
    const std::shared_ptr<const ExecutionPlan3D> execution_snapshot =
        route_execution_manager_.plan();
    if (execution_snapshot != nullptr) {
      resident_route_generation = execution_snapshot->routeGenerationHighWater();
    }
    const StaticRouteSearchCurrencyAssessment currency =
        assessStaticRouteSearchCurrency(request, resident_route_generation);
    if (!work->continuation_session && !currency.current()) {
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_REQUEST status=%.*s kind=%.*s "
          "request_generation=%" PRIu64 " resident_generation=%" PRIu64,
          static_cast<int>(staticRouteSearchCurrencyStatusName(currency.status).size()),
          staticRouteSearchCurrencyStatusName(currency.status).data(),
          static_cast<int>(staticRouteSearchRequestKindName(request.kind).size()),
          staticRouteSearchRequestKindName(request.kind).data(),
          request.base_route_generation, resident_route_generation);
      finishStaticRouteSearch(*transaction);
      continue;
    }

    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    if (!navigation.valid) {
      finishStaticRouteSearch(*transaction);
      continue;
    }
    processRouteSearch3D(transaction, work->world_telemetry, navigation,
                         std::move(work->continuation_session));
  }
}

mppi::State
ProductionMppiNode::selectTarget(const std::span<const RouteSample3D> route,
                                 const std::span<const mppi::RouteSample3D> mppi_route,
                                 const double current_station_m,
                                 const double lookahead_m, std::string& target_source,
                                 double& target_station_m) const {
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  mppi::State target{static_cast<float>(mission_goal.x),
                     static_cast<float>(mission_goal.y),
                     static_cast<float>(mission_goal.z)};
  target_source = "mission_goal_direct";
  target_station_m = 0.0;
  const double desired_station_m = current_station_m + std::max(0.0, lookahead_m);
  if (!route.empty()) {
    const RouteSample3D sample = sampleRoute3DAtStation(route, desired_station_m);
    target.x = static_cast<float>(sample.position.x);
    target.y = static_cast<float>(sample.position.y);
    target.z = static_cast<float>(sample.position.z);
    target_station_m = sample.station_m;
    target_source = "persistent_route_3d";
    return target;
  }
  if (!mppi_route.empty()) {
    const float desired_station = static_cast<float>(desired_station_m);
    const auto selected = std::ranges::lower_bound(mppi_route, desired_station, {},
                                                   &mppi::RouteSample3D::station_m);
    const mppi::RouteSample3D& sample =
        selected == mppi_route.end() ? mppi_route.back() : *selected;
    target.x = sample.x_m;
    target.y = sample.y_m;
    target.z = sample.z_m;
    target_station_m = sample.station_m;
    target_source = "persistent_route_3d";
    return target;
  }
  return target;
}

void ProductionMppiNode::requestRouteRelease(const RouteReleaseReason3D reason,
                                             const std::uint64_t route_generation) {
  requestStaticRouteReplan(reason, route_generation);
}

void ProductionMppiNode::handlePhysicalTrajectoryCollision(
    const std::uint64_t route_generation,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world,
    const std::string_view source,
    const ProductionMppiPhysicalTrajectoryAuthority authority) {
  if (route_generation == 0U) {
    return;
  }

  if (physicalCollisionAction(authority) ==
      ProductionMppiPhysicalCollisionAction::kRejectCandidate) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TRAJECTORY_COLLISION_CANDIDATE source=%.*s route_generation=%" PRIu64
        " action=reject_candidate_and_retain_resident_owner",
        static_cast<int>(source.size()), source.data(), route_generation);
    return;
  }

  std::uint64_t collision_raw_revision{0U};
  if (observed_raw_world != nullptr && observed_raw_world->valid()) {
    collision_raw_revision = observed_raw_world->version().revision;
    std::uint64_t blocked_raw_revision =
        observed_route_blocked_raw_revision_.load(std::memory_order_relaxed);
    while (blocked_raw_revision < collision_raw_revision &&
           !observed_route_blocked_raw_revision_.compare_exchange_weak(
               blocked_raw_revision, collision_raw_revision, std::memory_order_release,
               std::memory_order_relaxed)) {
    }
  }

  bool first_request_for_route{false};
  std::uint64_t requested_generation =
      physical_trajectory_replan_route_generation_.load(std::memory_order_relaxed);
  while (requested_generation < route_generation) {
    if (physical_trajectory_replan_route_generation_.compare_exchange_weak(
            requested_generation, route_generation, std::memory_order_acq_rel,
            std::memory_order_relaxed)) {
      first_request_for_route = true;
      break;
    }
  }
  if (!first_request_for_route) {
    return;
  }

  RCLCPP_WARN(get_logger(),
              "TRAJECTORY_COLLISION_REPLAN source=%.*s route_generation=%" PRIu64
              " raw_revision=%" PRIu64
              " action=retain_certified_owner_and_request_successor",
              static_cast<int>(source.size()), source.data(), route_generation,
              collision_raw_revision);
  requestRouteRelease(RouteReleaseReason3D::kBlocked, route_generation);
}

ProductionMppiStability
ProductionMppiNode::compareWithPrevious(const mppi::MppiTickResult& result) const {
  ProductionMppiStability stability;
  if (!previous_result_.has_value() || result.controls.empty() ||
      previous_result_->controls.size() < 2U || result.horizon.empty() ||
      previous_result_->horizon.size() < 2U) {
    return stability;
  }
  const mppi::Control& current = result.controls.front();
  const double offset_steps =
      result.warm_start_shift_s / static_cast<double>(mppi_config_.dynamics.dt_s);
  const std::vector<mppi::Control> shifted_controls =
      mppi::shiftControlSequence(previous_result_->controls, mppi_config_.dynamics.dt_s,
                                 result.warm_start_shift_s);
  const mppi::Control& previous = shifted_controls.front();
  stability.first_control_delta =
      std::hypot(std::hypot(current.ax - previous.ax, current.ay - previous.ay),
                 current.az - previous.az);
  const std::size_t count = result.horizon.size();
  double squared_sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const mppi::State previous_state = sampleState(
        previous_result_->horizon, offset_steps + static_cast<double>(index));
    const double difference = distance3(result.horizon[index], previous_state);
    squared_sum += difference * difference;
    stability.position_max_m = std::max(stability.position_max_m, difference);
  }
  stability.position_rms_m = std::sqrt(squared_sum / static_cast<double>(count));
  stability.terminal_shift_m =
      distance3(result.horizon.back(), previous_result_->horizon.back());
  stability.valid = true;
  return stability;
}

void ProductionMppiNode::startPlanningTimer() {
  planning_timer_ = create_wall_timer(
      std::chrono::duration<double>{1.0 / tick_rate_hz_}, [this]() { planningTick(); },
      planning_callback_group_);
}

ProductionMppiNode::~ProductionMppiNode() {
  if (diagnostics_sink_ != nullptr) {
    diagnostics_sink_->stop();
  }
  if (world_pipeline_ != nullptr) {
    world_pipeline_->stop();
  }
  if (route_planning_worker_.joinable()) {
    route_planning_worker_.request_stop();
    route_planning_queue_condition_.notify_all();
    route_planning_worker_.join();
  }
  publishSummary();
}

} // namespace drone_city_nav

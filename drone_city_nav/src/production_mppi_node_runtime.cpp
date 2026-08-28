#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <span>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_world.hpp"

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

void ProductionMppiNode::guideWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::shared_ptr<const ProductionMppiPreparedEsdf> world;
    {
      std::unique_lock lock{guide_queue_mutex_};
      guide_queue_condition_.wait(lock, stop_token,
                                  [this]() { return pending_guide_world_ != nullptr; });
      if (stop_token.stop_requested()) {
        return;
      }
      world = std::exchange(pending_guide_world_, nullptr);
    }
    if (!world) {
      continue;
    }
    if (!productionWorldGenerationCoherent(*world)) {
      const ProductionWorldGenerationStatus status =
          assessProductionWorldGeneration(*world);
      const std::string_view status_name = productionWorldGenerationStatusName(status);
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_GUIDE rejected local_world_generation=%" PRIu64
                   " reason=%.*s",
                   world->local_world_generation.generation,
                   static_cast<int>(status_name.size()), status_name.data());
      finishStaticRouteSearch(*world);
      continue;
    }
    if (world->grid.depth <= 1) {
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_GUIDE rejected local_world_generation=%" PRIu64
                   " reason=full_3d_world_required depth=%d",
                   world->local_world_generation.generation, world->grid.depth);
      finishStaticRouteSearch(*world);
      continue;
    }

    const StaticRouteSearchRequestIdentity request = identifyStaticRouteSearchRequest(
        world->route_generation, world->static_route_extension_request,
        world->static_route_extension_base_generation,
        world->static_route_replan_request, world->static_route_replan_base_generation);
    std::uint64_t resident_route_generation = 0U;
    const std::shared_ptr<const ExecutionRouteSnapshot3D> execution_snapshot =
        execution_route_store_.snapshot();
    if (execution_snapshot != nullptr) {
      resident_route_generation = execution_snapshot->routeGenerationHighWater();
    }
    const StaticRouteSearchCurrencyAssessment currency =
        assessStaticRouteSearchCurrency(request, resident_route_generation);
    if (!currency.current()) {
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_REQUEST status=%.*s kind=%.*s "
          "request_generation=%" PRIu64 " resident_generation=%" PRIu64,
          static_cast<int>(staticRouteSearchCurrencyStatusName(currency.status).size()),
          staticRouteSearchCurrencyStatusName(currency.status).data(),
          static_cast<int>(staticRouteSearchRequestKindName(request.kind).size()),
          staticRouteSearchRequestKindName(request.kind).data(),
          request.base_route_generation, resident_route_generation);
      finishStaticRouteSearch(*world);
      continue;
    }

    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    if (!navigation.valid) {
      finishStaticRouteSearch(*world);
      continue;
    }
    processGuideSearch3D(*world, navigation);
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
    target_source = "global_route_3d";
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
    target_source = "global_route_3d";
    return target;
  }
  return target;
}

void ProductionMppiNode::requestGuideRelease(const GlobalGuideReleaseReason reason,
                                             const std::uint64_t guide_generation) {
  requestStaticRouteReplan(reason, guide_generation);
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
  if (diagnostics_worker_.joinable()) {
    diagnostics_worker_.request_stop();
    diagnostics_mailbox_.notifyAll();
    diagnostics_worker_.join();
  }
  if (esdf_worker_.joinable()) {
    esdf_worker_.request_stop();
    raw_queue_condition_.notify_all();
    esdf_worker_.join();
  }
  if (guide_worker_.joinable()) {
    guide_worker_.request_stop();
    guide_queue_condition_.notify_all();
    guide_worker_.join();
  }
  publishSummary();
}

} // namespace drone_city_nav

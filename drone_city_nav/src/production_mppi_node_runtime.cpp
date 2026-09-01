#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <span>
#include <utility>

#include "execution_horizon_assembler_3d.hpp"
#include "mppi_controller_3d.hpp"
#include "navigation_diagnostics_sink.hpp"
#include "planning_cycle_coordinator_3d.hpp"
#include "production_mppi_node.hpp"
#include "raw_world_ingress_ros_3d.hpp"
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
  if (route_lifecycle_coordinator_ != nullptr) {
    route_lifecycle_coordinator_->stop();
  }
  if (world_pipeline_ != nullptr) {
    world_pipeline_->stop();
  }
  publishSummary();
}

} // namespace drone_city_nav

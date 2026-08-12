#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <ranges>
#include <string>
#include <vector>

#include "cooperative_traffic_referee_node.hpp"
#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double ageSeconds(const std::int64_t now_ns,
                                const std::int64_t stamp_ns) noexcept {
  if (stamp_ns <= 0 || now_ns < stamp_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - stamp_ns) * 1.0e-9;
}

[[nodiscard]] msg::NavigationObjective
immediateHoldObjective(const rclcpp::Time& stamp, const std::uint64_t mission_epoch,
                       const std::uint64_t sequence, const Point3& position) {
  msg::NavigationObjective objective;
  objective.stamp = stamp;
  objective.mission_epoch = mission_epoch;
  objective.sample_sequence = sequence;
  objective.position.x = position.x;
  objective.position.y = position.y;
  objective.position.z = position.z;
  objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
  objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
  objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD;
  return objective;
}

} // namespace

bool CooperativeTrafficRefereeNode::runtimeInputsHealthy(const std::int64_t now_ns) {
  bool all_healthy = true;
  bool continuity_reset = false;
  for (VehicleRuntime& vehicle : vehicles_) {
    if (vehicle.destroyed) {
      continue;
    }
    const bool navigation_fresh =
        vehicle.navigation_state && vehicle.navigation_state->stamp_ns > 0 &&
        now_ns >= vehicle.navigation_state->stamp_ns &&
        now_ns - vehicle.navigation_state->stamp_ns <= maximum_input_age_ns_;
    const bool truth_fresh =
        vehicle.truth_state && vehicle.truth_state->stamp_ns > 0 &&
        now_ns >= vehicle.truth_state->stamp_ns &&
        now_ns - vehicle.truth_state->stamp_ns <= maximum_input_age_ns_;
    const bool intent_fresh =
        vehicle.latest_intent_receive_ns > 0 &&
        now_ns >= vehicle.latest_intent_receive_ns &&
        now_ns - vehicle.latest_intent_receive_ns <= maximum_intent_age_ns_ &&
        now_ns <= vehicle.latest_intent_valid_until_ns;
    if (navigation_fresh && truth_fresh && intent_fresh) {
      if (vehicle.degraded_since_ns > 0) {
        RCLCPP_INFO(get_logger(),
                    "COOPERATIVE_ADJUDICATION vehicle_id='%s' state=recovered "
                    "degraded_duration_s=%.3f",
                    vehicle.id.c_str(),
                    static_cast<double>(now_ns - vehicle.degraded_since_ns) * 1.0e-9);
        vehicle.degraded_since_ns = 0;
        continuity_reset = true;
      }
      continue;
    }
    all_healthy = false;
    if (vehicle.degraded_since_ns <= 0 || now_ns < vehicle.degraded_since_ns) {
      vehicle.degraded_since_ns = now_ns;
      continuity_reset = true;
    }
    const double degraded_duration_s =
        static_cast<double>(now_ns - vehicle.degraded_since_ns) * 1.0e-9;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "COOPERATIVE_ADJUDICATION vehicle_id='%s' state=degraded "
        "navigation_age_ms=%.1f truth_age_ms=%.1f intent_age_ms=%.1f "
        "intent_valid=%s degraded_duration_s=%.3f",
        vehicle.id.c_str(),
        vehicle.navigation_state
            ? ageSeconds(now_ns, vehicle.navigation_state->stamp_ns) * 1000.0
            : std::numeric_limits<double>::infinity(),
        vehicle.truth_state ? ageSeconds(now_ns, vehicle.truth_state->stamp_ns) * 1000.0
                            : std::numeric_limits<double>::infinity(),
        ageSeconds(now_ns, vehicle.latest_intent_receive_ns) * 1000.0,
        now_ns <= vehicle.latest_intent_valid_until_ns ? "true" : "false",
        degraded_duration_s);
    if (now_ns - vehicle.degraded_since_ns >= maximum_degraded_duration_ns_) {
      beginFailure("prolonged_stale_vehicle_state:" + vehicle.id);
      break;
    }
  }
  if (continuity_reset) {
    separation_monitor_->resetTemporalContinuity();
  }
  return all_healthy;
}

void CooperativeTrafficRefereeNode::updateSeparationMetrics() {
  std::vector<TimedVehicleState> states;
  std::vector<std::size_t> indices;
  states.reserve(vehicles_.size());
  indices.reserve(vehicles_.size());
  for (std::size_t index = 0U; index < vehicles_.size(); ++index) {
    if (vehicles_[index].destroyed) {
      continue;
    }
    const std::optional<TimedVehicleState> state = physicalState(index);
    if (!state.has_value() || !state->position_valid || !state->armed ||
        !state->airborne) {
      continue;
    }
    states.push_back(*state);
    indices.push_back(index);
  }
  if (states.size() != vehicles_.size()) {
    separation_monitor_->resetTemporalContinuity();
    return;
  }
  const CooperativeSeparationUpdate update = separation_monitor_->update(states);
  active_desired_violation_count_ = update.active_desired_violation_count;
  if (update.first_minimum_index && update.second_minimum_index) {
    minimum_pair_first_id_ = vehicles_[indices[*update.first_minimum_index]].id;
    minimum_pair_second_id_ = vehicles_[indices[*update.second_minimum_index]].id;
  }
  for (const CooperativePairSeparationUpdate& pair : update.pairs) {
    const std::string& first_id = vehicles_[indices[pair.first_index]].id;
    const std::string& second_id = vehicles_[indices[pair.second_index]].id;
    if (pair.newly_entered_desired_violation) {
      RCLCPP_WARN(get_logger(),
                  "COOPERATIVE_SEPARATION desired_violation_enter=true first='%s' "
                  "second='%s' swept_separation_m=%.3f current_separation_m=%.3f "
                  "desired_m=%.3f event_count=%" PRIu64,
                  first_id.c_str(), second_id.c_str(), pair.separation.minimum_m,
                  pair.separation.current_m,
                  separation_config_.desired_minimum_separation_m,
                  update.desired_violation_event_count);
    } else if (pair.newly_released_desired_violation) {
      RCLCPP_INFO(get_logger(),
                  "COOPERATIVE_SEPARATION desired_violation_released=true first='%s' "
                  "second='%s' current_separation_m=%.3f release_m=%.3f",
                  first_id.c_str(), second_id.c_str(), pair.separation.current_m,
                  separation_config_.release_separation_m);
    }
  }
}

void CooperativeTrafficRefereeNode::updateGoalHolds() {
  for (std::size_t index = 0U; index < vehicles_.size(); ++index) {
    VehicleRuntime& vehicle = vehicles_[index];
    const std::optional<TimedVehicleState> state = physicalState(index);
    if (vehicle.destroyed || vehicle.goal_hold_confirmed || !state.has_value()) {
      continue;
    }
    std::optional<Point3> active_hold_position;
    if (vehicle.hold_horizon && vehicle.hold_horizon->active) {
      active_hold_position = vehicle.hold_horizon->position;
    }
    const CooperativeGoalHoldUpdate update = vehicle.goal_hold_confirmation->update(
        *state, vehicle.goal, active_hold_position);
    if (!update.newly_confirmed) {
      continue;
    }
    vehicle.goal_hold_confirmed = true;
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_GOAL_HOLD_CONFIRMED vehicle_id='%s' "
                "goal_distance_m=%.3f hold_position_error_m=%.3f speed_mps=%.3f "
                "mission_epoch=%" PRIu64,
                vehicle.id.c_str(), update.goal_distance_m,
                update.hold_position_error_m, update.speed_mps, mission_epoch_);
  }
}

bool CooperativeTrafficRefereeNode::allGoalHoldsConfirmed() const noexcept {
  return std::ranges::all_of(vehicles_, [](const VehicleRuntime& vehicle) {
    return !vehicle.destroyed && vehicle.goal_hold_confirmed;
  });
}

void CooperativeTrafficRefereeNode::beginFailure(const std::string& reason) {
  if (failure_reason_.has_value() || result_reported_) {
    return;
  }
  failure_reason_ = reason;
  failure_latched_ns_ = now().nanoseconds();
  RCLCPP_ERROR(get_logger(),
               "COOPERATIVE_TRAFFIC_FAILURE latched=true reason='%s' "
               "mission_epoch=%" PRIu64 " disarm_requested=false",
               reason.c_str(), mission_epoch_);
  requestHoldsForSurvivors(reason);
}

void CooperativeTrafficRefereeNode::requestHold(const std::size_t index,
                                                const std::string& reason) {
  VehicleRuntime& vehicle = vehicles_[index];
  if (vehicle.destroyed || vehicle.hold_requested || !vehicle.navigation_state ||
      !vehicle.navigation_state->position_valid) {
    return;
  }
  vehicle.hold_requested = true;
  vehicle.hold_requested_ns = now().nanoseconds();
  vehicle.hold_request_horizon_sequence =
      vehicle.hold_horizon ? vehicle.hold_horizon->sequence : 0U;
  vehicle.requested_hold_position = vehicle.navigation_state->position;
  vehicle.failure_hold_confirmation =
      std::make_unique<CooperativeGoalHoldConfirmation>(CooperativeGoalHoldConfig{
          .goal_tolerance_m = goal_hold_config_.hold_position_tolerance_m,
          .hold_position_tolerance_m = goal_hold_config_.hold_position_tolerance_m,
          .maximum_speed_mps = goal_hold_config_.maximum_speed_mps,
          .confirmation_duration_s = goal_hold_config_.confirmation_duration_s,
      });
  vehicle.objective_pub->publish(
      immediateHoldObjective(now(), mission_epoch_, ++vehicle.objective_sequence,
                             *vehicle.requested_hold_position));
  RCLCPP_INFO(get_logger(),
              "COOPERATIVE_HOLD requested=true vehicle_id='%s' reason='%s' "
              "position=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64,
              vehicle.id.c_str(), reason.c_str(), vehicle.requested_hold_position->x,
              vehicle.requested_hold_position->y, vehicle.requested_hold_position->z,
              mission_epoch_);
}

void CooperativeTrafficRefereeNode::requestHoldsForSurvivors(
    const std::string& reason) {
  for (std::size_t index = 0U; index < vehicles_.size(); ++index) {
    requestHold(index, reason);
  }
}

bool CooperativeTrafficRefereeNode::allDestroyedVehiclesSettled(
    const std::int64_t now_ns) {
  const auto unsettled =
      std::ranges::find_if(vehicles_, [](const VehicleRuntime& vehicle) {
        return vehicle.destroyed &&
               (!vehicle.navigation_state || vehicle.navigation_state->armed);
      });
  if (unsettled == vehicles_.end()) {
    return true;
  }
  if (unsettled->destroyed_observed_ns > 0 &&
      now_ns - unsettled->destroyed_observed_ns > destruction_settlement_timeout_ns_) {
    finishFailure("vehicle_destruction_not_confirmed:" + unsettled->id);
  }
  return false;
}

bool CooperativeTrafficRefereeNode::allSurvivorsHeld(const std::int64_t now_ns) {
  for (VehicleRuntime& vehicle : vehicles_) {
    if (vehicle.destroyed) {
      continue;
    }
    const std::optional<TimedVehicleState> state =
        detail::physicalState(vehicle.navigation_state, vehicle.truth_state);
    if (!vehicle.navigation_state) {
      return false;
    }
    if (!vehicle.navigation_state->armed || !vehicle.navigation_state->airborne) {
      continue;
    }
    if (!vehicle.hold_requested || !vehicle.failure_hold_confirmation ||
        !state.has_value() || !vehicle.requested_hold_position) {
      return false;
    }
    std::optional<Point3> active_hold_position;
    if (vehicle.hold_horizon && vehicle.hold_horizon->active &&
        vehicle.hold_horizon->sequence > vehicle.hold_request_horizon_sequence) {
      active_hold_position = vehicle.hold_horizon->position;
    }
    const CooperativeGoalHoldUpdate update = vehicle.failure_hold_confirmation->update(
        *state, *vehicle.requested_hold_position, active_hold_position);
    if (update.newly_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "COOPERATIVE_HOLD_CONFIRMED vehicle_id='%s' "
                  "position_error_m=%.3f speed_mps=%.3f mission_epoch=%" PRIu64,
                  vehicle.id.c_str(), update.hold_position_error_m, update.speed_mps,
                  mission_epoch_);
    }
    if (!update.confirmed) {
      if (vehicle.hold_requested_ns > 0 &&
          now_ns - vehicle.hold_requested_ns > hold_timeout_ns_) {
        finishFailure("survivor_hold_not_confirmed:" + vehicle.id);
      }
      return false;
    }
  }
  return true;
}

void CooperativeTrafficRefereeNode::settleFailure(const std::int64_t now_ns) {
  if (!failure_reason_.has_value() || result_reported_) {
    return;
  }
  requestHoldsForSurvivors(*failure_reason_);
  if (allDestroyedVehiclesSettled(now_ns) && allSurvivorsHeld(now_ns)) {
    finishFailure(*failure_reason_);
  } else if (failure_latched_ns_ > 0 &&
             now_ns - failure_latched_ns_ >
                 hold_timeout_ns_ + destruction_settlement_timeout_ns_) {
    finishFailure("failure_settlement_timeout:" + *failure_reason_);
  }
}

void CooperativeTrafficRefereeNode::finishSuccess() {
  if (result_reported_) {
    return;
  }
  result_reported_ = true;
  RCLCPP_INFO(
      get_logger(),
      "MISSION_RESULT success=true mission=cooperative_traffic "
      "outcome=all_goals_reached vehicle_count=%zu "
      "minimum_physical_separation_m=%.3f minimum_pair='%s:%s' "
      "desired_separation_m=%.3f desired_separation_violation_events=%" PRIu64
      " active_desired_violations=%zu physical_collisions=0 building_collisions=0 "
      "mission_epoch=%" PRIu64,
      vehicles_.size(), separation_monitor_->minimumObservedSeparationM(),
      minimum_pair_first_id_.c_str(), minimum_pair_second_id_.c_str(),
      separation_config_.desired_minimum_separation_m,
      separation_monitor_->desiredViolationEventCount(),
      active_desired_violation_count_, mission_epoch_);
  completeResultLifecycle();
}

void CooperativeTrafficRefereeNode::finishFailure(const std::string& reason) {
  if (result_reported_) {
    return;
  }
  result_reported_ = true;
  RCLCPP_ERROR(get_logger(),
               "MISSION_RESULT success=false mission=cooperative_traffic "
               "outcome=technical_failure reason='%s' vehicle_count=%zu "
               "minimum_physical_separation_m=%.3f minimum_pair='%s:%s' "
               "desired_separation_m=%.3f desired_separation_violation_events=%" PRIu64
               " active_desired_violations=%zu disarm_requested_by_referee=false "
               "mission_epoch=%" PRIu64,
               reason.c_str(), vehicles_.size(),
               separation_monitor_->minimumObservedSeparationM(),
               minimum_pair_first_id_.c_str(), minimum_pair_second_id_.c_str(),
               separation_config_.desired_minimum_separation_m,
               separation_monitor_->desiredViolationEventCount(),
               active_desired_violation_count_, mission_epoch_);
  completeResultLifecycle();
}

void CooperativeTrafficRefereeNode::completeResultLifecycle() {
  if (shutdown_on_terminal_outcome_) {
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(),
              "COOPERATIVE_TRAFFIC_MISSION state=terminal_observation "
              "simulation_shutdown_requested=false mission_epoch=%" PRIu64,
              mission_epoch_);
}

void CooperativeTrafficRefereeNode::tick() {
  if (result_reported_) {
    return;
  }
  const std::int64_t now_ns = now().nanoseconds();
  if (readiness_started_ns_ <= 0) {
    readiness_started_ns_ = now_ns;
  }
  if (boundary_check_started_ns_ <= 0) {
    boundary_check_started_ns_ = now_ns;
  }
  if (!verifyGroundTruthBoundary(now_ns)) {
    if (!failure_reason_.has_value() &&
        now_ns - boundary_check_started_ns_ > boundary_startup_timeout_ns_) {
      beginFailure("ground_truth_boundary_not_ready");
    }
    settleFailure(now_ns);
    return;
  }
  if (!mission_started_ && truth_alignment_update_.startup_failure_confirmed) {
    beginFailure("coordinate_alignment_mismatch:" + truth_alignment_vehicle_id_ + ":" +
                 truth_alignment_reason_ +
                 ":error_m=" + std::to_string(truth_alignment_maximum_error_m_));
    settleFailure(now_ns);
    return;
  }
  if (failure_reason_.has_value()) {
    settleFailure(now_ns);
    return;
  }
  if (!mission_started_) {
    if (missionReady(now_ns)) {
      publishMissionStart();
    } else if (readiness_started_ns_ > 0 &&
               now_ns - readiness_started_ns_ > readiness_timeout_ns_) {
      beginFailure("mission_readiness_timeout");
      settleFailure(now_ns);
    }
    return;
  }
  if (now_ns - mission_started_ns_ > mission_timeout_ns_) {
    beginFailure("mission_timeout");
    settleFailure(now_ns);
    return;
  }
  if (!runtimeInputsHealthy(now_ns)) {
    settleFailure(now_ns);
    return;
  }
  updateSeparationMetrics();
  updateGoalHolds();
  if (allGoalHoldsConfirmed()) {
    finishSuccess();
  }
}

} // namespace drone_city_nav

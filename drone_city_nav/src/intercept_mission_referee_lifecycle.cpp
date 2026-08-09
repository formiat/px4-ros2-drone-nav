#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#include "intercept_mission_referee_node.hpp"
#include "intercept_referee_support.hpp"
#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y),
                    first.z - second.z);
}

} // namespace

const char*
InterceptMissionRefereeNode::targetOutcomeName(const TargetOutcome outcome) noexcept {
  switch (outcome) {
    case InterceptMissionRefereeNode::TargetOutcome::kActive:
      return "active";
    case InterceptMissionRefereeNode::TargetOutcome::kIntercepted:
      return "intercepted";
    case InterceptMissionRefereeNode::TargetOutcome::kReachedGoal:
      return "reached_goal";
    case InterceptMissionRefereeNode::TargetOutcome::kDestroyed:
      return "destroyed";
  }
  return "unknown";
}

bool InterceptMissionRefereeNode::updateStateAdjudication(const std::int64_t now_ns) {
  bool all_targets_fresh = true;
  bool continuity_reset = false;
  for (std::size_t interceptor_index = 0U; interceptor_index < interceptors_.size();
       ++interceptor_index) {
    InterceptorRuntime& interceptor = interceptors_[interceptor_index];
    if (interceptor.destroyed || interceptor.destruction_requested ||
        interceptor.disabled) {
      continue;
    }
    const std::optional<TimedVehicleState> interceptor_state =
        interceptorPhysicalState(interceptor_index);
    if (!interceptor_state.has_value()) {
      return false;
    }
    for (std::size_t target_index = 0U; target_index < targets_.size();
         ++target_index) {
      const TargetRuntime& target = targets_[target_index];
      if (target.outcome != TargetOutcome::kActive) {
        continue;
      }
      const std::optional<TimedVehicleState> target_state =
          targetPhysicalState(target_index);
      if (!target_state.has_value()) {
        return false;
      }
      const InterceptStateAdjudicationUpdate update =
          interceptor.target_adjudications[target_index]->update(
              now_ns, *interceptor_state, *target_state);
      continuity_reset = continuity_reset || update.newly_recovered;
      all_targets_fresh = all_targets_fresh && update.evader_fresh;
      if (update.status == InterceptStateAdjudicationStatus::kHealthy) {
        continue;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "INTERCEPT_ADJUDICATION interceptor_id='%s' target_id='%s' state=%s "
          "interceptor_age_ms=%.1f target_age_ms=%.1f degraded_duration_s=%.2f",
          interceptor.id.c_str(), target.id.c_str(),
          update.status == InterceptStateAdjudicationStatus::kProlongedFailure
              ? "prolonged_failure"
              : "degraded",
          update.interceptor_age_s * 1000.0, update.evader_age_s * 1000.0,
          update.degraded_duration_s);
      if (update.newly_prolonged_failure && !update.evader_fresh &&
          !system_failure_reason_.has_value()) {
        system_failure_reason_ = "prolonged_stale_target_state:" + target.id;
        requestHoldsForSurvivors(*system_failure_reason_);
        return false;
      }
      if (update.newly_prolonged_failure && !update.interceptor_fresh) {
        interceptor.disabled = true;
        requestHold(interceptor_index,
                    "prolonged_stale_interceptor_state:" + interceptor.id);
        break;
      }
    }
  }
  if (continuity_reset) {
    resetPhysicalContinuity();
  }
  return all_targets_fresh;
}

void InterceptMissionRefereeNode::resetPhysicalContinuity() {
  for (InterceptorRuntime& interceptor : interceptors_) {
    interceptor.previous_physical_state.reset();
  }
  for (TargetRuntime& target : targets_) {
    target.previous_physical_state.reset();
  }
}

std::vector<InterceptMissionRefereeNode::ContactCandidate>
InterceptMissionRefereeNode::contactCandidates() const {
  std::vector<ContactCandidate> candidates;
  for (std::size_t interceptor_index = 0U; interceptor_index < interceptors_.size();
       ++interceptor_index) {
    const InterceptorRuntime& interceptor = interceptors_[interceptor_index];
    const std::optional<TimedVehicleState> interceptor_state =
        interceptorPhysicalState(interceptor_index);
    if (!interceptor_state.has_value()) {
      continue;
    }
    const TimedVehicleState& resolved_interceptor_state = interceptor_state.value();
    if (interceptor.destroyed || interceptor.destruction_requested ||
        !resolved_interceptor_state.armed || !resolved_interceptor_state.airborne) {
      continue;
    }
    for (std::size_t target_index = 0U; target_index < targets_.size();
         ++target_index) {
      const TargetRuntime& target = targets_[target_index];
      const bool contact_relevant = target.outcome == TargetOutcome::kActive ||
                                    target.outcome == TargetOutcome::kReachedGoal;
      const std::optional<TimedVehicleState> target_state =
          targetPhysicalState(target_index);
      if (!contact_relevant || !target_state.has_value()) {
        continue;
      }
      const TimedVehicleState& resolved_target_state = target_state.value();
      if (target.destroyed || target.destruction_requested ||
          !resolved_target_state.armed || !resolved_target_state.airborne) {
        continue;
      }
      const SweptVehicleSeparation separation = sweptVehicleSeparation(
          resolved_interceptor_state, resolved_target_state,
          interceptor.previous_physical_state, target.previous_physical_state);
      if (separation.minimum_m <= capture_radius_m_) {
        candidates.push_back(ContactCandidate{
            .interceptor_index = interceptor_index,
            .target_index = target_index,
            .separation = separation,
        });
      }
    }
  }
  std::ranges::sort(candidates,
                    [](const ContactCandidate& first, const ContactCandidate& second) {
                      if (first.separation.minimum_m != second.separation.minimum_m) {
                        return first.separation.minimum_m < second.separation.minimum_m;
                      }
                      if (first.target_index != second.target_index) {
                        return first.target_index < second.target_index;
                      }
                      return first.interceptor_index < second.interceptor_index;
                    });
  return candidates;
}

void InterceptMissionRefereeNode::requestCaptureDestruction(
    const ContactCandidate& candidate, const std::string& event_detail) {
  InterceptorRuntime& interceptor = interceptors_[candidate.interceptor_index];
  TargetRuntime& target = targets_[candidate.target_index];
  const std::optional<TimedVehicleState> interceptor_state =
      interceptorPhysicalState(candidate.interceptor_index);
  const std::optional<TimedVehicleState> target_state =
      targetPhysicalState(candidate.target_index);
  if (!interceptor_state.has_value() || !target_state.has_value() ||
      interceptor.destruction_requested || target.destruction_requested) {
    return;
  }
  const TimedVehicleState& resolved_interceptor_state = interceptor_state.value();
  const TimedVehicleState& resolved_target_state = target_state.value();
  const std::int64_t requested_ns = now().nanoseconds();
  interceptor.destruction_requested = true;
  interceptor.destruction_requested_ns = requested_ns;
  target.destruction_requested = true;
  target.destruction_requested_ns = requested_ns;
  if (destruction_requested_ns_ <= 0) {
    destruction_requested_ns_ = requested_ns;
  }
  interceptor.destroyed_pub->publish(makeProximityDestructionEvent(
      now(), mission_epoch_, resolved_interceptor_state,
      msg::VehicleDestroyed::ROLE_INTERCEPTOR, interceptor.id,
      msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT, event_detail));
  target.destroyed_pub->publish(makeProximityDestructionEvent(
      now(), mission_epoch_, resolved_target_state, msg::VehicleDestroyed::ROLE_EVADER,
      target.id, msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT, event_detail));
  logPhysicalProximityIntercept(get_logger(), interceptor.id, target.id,
                                candidate.separation, resolved_interceptor_state,
                                resolved_target_state, capture_radius_m_, requested_ns,
                                mission_epoch_);
}

void InterceptMissionRefereeNode::requestSameRoleCollision(
    const bool interceptor_role, const std::size_t first_index,
    const std::size_t second_index, const double separation_m) {
  const std::int64_t requested_ns = now().nanoseconds();
  if (interceptor_role) {
    InterceptorRuntime& first = interceptors_[first_index];
    InterceptorRuntime& second = interceptors_[second_index];
    const std::optional<TimedVehicleState> first_state =
        interceptorPhysicalState(first_index);
    const std::optional<TimedVehicleState> second_state =
        interceptorPhysicalState(second_index);
    if (!first_state.has_value() || !second_state.has_value() ||
        first.destruction_requested || second.destruction_requested) {
      return;
    }
    const TimedVehicleState& resolved_first_state = first_state.value();
    const TimedVehicleState& resolved_second_state = second_state.value();
    first.destruction_requested = true;
    second.destruction_requested = true;
    first.destruction_requested_ns = requested_ns;
    second.destruction_requested_ns = requested_ns;
    const std::string event_detail =
        "interceptor_collision:" + first.id + ":" + second.id;
    first.destroyed_pub->publish(makeProximityDestructionEvent(
        now(), mission_epoch_, resolved_first_state,
        msg::VehicleDestroyed::ROLE_INTERCEPTOR, first.id,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, event_detail));
    second.destroyed_pub->publish(makeProximityDestructionEvent(
        now(), mission_epoch_, resolved_second_state,
        msg::VehicleDestroyed::ROLE_INTERCEPTOR, second.id,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, event_detail));
    RCLCPP_ERROR(get_logger(),
                 "INTERCEPTOR_PROXIMITY_COLLISION first='%s' second='%s' "
                 "separation_m=%.3f threshold_m=%.3f mission_epoch=%" PRIu64,
                 first.id.c_str(), second.id.c_str(), separation_m, capture_radius_m_,
                 mission_epoch_);
  } else {
    TargetRuntime& first = targets_[first_index];
    TargetRuntime& second = targets_[second_index];
    const std::optional<TimedVehicleState> first_state =
        targetPhysicalState(first_index);
    const std::optional<TimedVehicleState> second_state =
        targetPhysicalState(second_index);
    if (!first_state.has_value() || !second_state.has_value() ||
        first.destruction_requested || second.destruction_requested) {
      return;
    }
    const TimedVehicleState& resolved_first_state = first_state.value();
    const TimedVehicleState& resolved_second_state = second_state.value();
    if (first.outcome == TargetOutcome::kActive) {
      markTargetOutcome(first_index, TargetOutcome::kDestroyed,
                        "target_proximity_collision");
    }
    if (second.outcome == TargetOutcome::kActive) {
      markTargetOutcome(second_index, TargetOutcome::kDestroyed,
                        "target_proximity_collision");
    }
    first.destruction_requested = true;
    second.destruction_requested = true;
    first.destruction_requested_ns = requested_ns;
    second.destruction_requested_ns = requested_ns;
    const std::string event_detail = "target_collision:" + first.id + ":" + second.id;
    first.destroyed_pub->publish(makeProximityDestructionEvent(
        now(), mission_epoch_, resolved_first_state, msg::VehicleDestroyed::ROLE_EVADER,
        first.id, msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, event_detail));
    second.destroyed_pub->publish(makeProximityDestructionEvent(
        now(), mission_epoch_, resolved_second_state,
        msg::VehicleDestroyed::ROLE_EVADER, second.id,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, event_detail));
    RCLCPP_ERROR(get_logger(),
                 "TARGET_PROXIMITY_COLLISION first='%s' second='%s' "
                 "separation_m=%.3f threshold_m=%.3f mission_epoch=%" PRIu64,
                 first.id.c_str(), second.id.c_str(), separation_m, capture_radius_m_,
                 mission_epoch_);
  }
  if (destruction_requested_ns_ <= 0) {
    destruction_requested_ns_ = requested_ns;
  }
}

void InterceptMissionRefereeNode::detectPhysicalContacts() {
  std::vector<bool> used_interceptors(interceptors_.size(), false);
  std::vector<bool> used_targets(targets_.size(), false);
  for (const ContactCandidate& candidate : contactCandidates()) {
    if (used_interceptors[candidate.interceptor_index] ||
        used_targets[candidate.target_index]) {
      continue;
    }
    TargetRuntime& target = targets_[candidate.target_index];
    std::string event_detail{"intercepted"};
    if (target.outcome == TargetOutcome::kActive) {
      markTargetOutcome(candidate.target_index, TargetOutcome::kIntercepted,
                        event_detail, interceptors_[candidate.interceptor_index].id);
    } else {
      const bool legacy_single_target =
          targets_.size() == 1U && mission_name_ == "intercept";
      event_detail = legacy_single_target ? "late_intercept_after_evader_goal"
                                          : "late_intercept_after_target_goal";
      if (legacy_single_target) {
        RCLCPP_INFO(get_logger(),
                    "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal "
                    "interceptor_id='%s' separation_m=%.3f mission_epoch=%" PRIu64,
                    interceptors_[candidate.interceptor_index].id.c_str(),
                    candidate.separation.minimum_m, mission_epoch_);
      } else {
        RCLCPP_INFO(get_logger(),
                    "INTERCEPT_LATE_CAPTURE outcome_preserved=target_reached_goal "
                    "interceptor_id='%s' target_id='%s' separation_m=%.3f "
                    "mission_epoch=%" PRIu64,
                    interceptors_[candidate.interceptor_index].id.c_str(),
                    target.id.c_str(), candidate.separation.minimum_m, mission_epoch_);
      }
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_ABORTED vehicle_id='%s' reason=late_capture "
                  "mission_epoch=%" PRIu64,
                  interceptors_[candidate.interceptor_index].id.c_str(),
                  mission_epoch_);
    }
    requestCaptureDestruction(candidate, event_detail);
    used_interceptors[candidate.interceptor_index] = true;
    used_targets[candidate.target_index] = true;
  }

  for (std::size_t first = 0U; first < interceptors_.size(); ++first) {
    const std::optional<TimedVehicleState> first_state =
        interceptorPhysicalState(first);
    if (!first_state.has_value()) {
      continue;
    }
    const TimedVehicleState& resolved_first_state = first_state.value();
    if (interceptors_[first].destroyed || interceptors_[first].destruction_requested ||
        !resolved_first_state.armed || !resolved_first_state.airborne) {
      continue;
    }
    for (std::size_t second = first + 1U; second < interceptors_.size(); ++second) {
      const std::optional<TimedVehicleState> second_state =
          interceptorPhysicalState(second);
      if (!second_state.has_value()) {
        continue;
      }
      const TimedVehicleState& resolved_second_state = second_state.value();
      if (interceptors_[second].destroyed ||
          interceptors_[second].destruction_requested || !resolved_second_state.armed ||
          !resolved_second_state.airborne) {
        continue;
      }
      const double separation =
          minimumSweptVehicleSeparation(resolved_first_state, resolved_second_state,
                                        interceptors_[first].previous_physical_state,
                                        interceptors_[second].previous_physical_state);
      if (separation <= capture_radius_m_) {
        requestSameRoleCollision(true, first, second, separation);
      }
    }
  }
  for (std::size_t first = 0U; first < targets_.size(); ++first) {
    const std::optional<TimedVehicleState> first_state = targetPhysicalState(first);
    if (!first_state.has_value()) {
      continue;
    }
    const TimedVehicleState& resolved_first_state = first_state.value();
    if (targets_[first].destroyed || targets_[first].destruction_requested ||
        !resolved_first_state.armed || !resolved_first_state.airborne) {
      continue;
    }
    for (std::size_t second = first + 1U; second < targets_.size(); ++second) {
      const std::optional<TimedVehicleState> second_state = targetPhysicalState(second);
      if (!second_state.has_value()) {
        continue;
      }
      const TimedVehicleState& resolved_second_state = second_state.value();
      if (targets_[second].destroyed || targets_[second].destruction_requested ||
          !resolved_second_state.armed || !resolved_second_state.airborne) {
        continue;
      }
      const double separation =
          minimumSweptVehicleSeparation(resolved_first_state, resolved_second_state,
                                        targets_[first].previous_physical_state,
                                        targets_[second].previous_physical_state);
      if (separation <= capture_radius_m_) {
        requestSameRoleCollision(false, first, second, separation);
      }
    }
  }
}

void InterceptMissionRefereeNode::evaluateTargetGoals() {
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    TargetRuntime& target = targets_[index];
    if (target.outcome != TargetOutcome::kActive || target.destroyed ||
        target.destruction_requested) {
      continue;
    }
    const std::optional<TimedVehicleState> state = targetPhysicalState(index);
    if (state.has_value() && state->position_valid && state->armed && state->airborne &&
        distance(state->position, target.goal) <= target_goal_radius_m_) {
      markTargetOutcome(index, TargetOutcome::kReachedGoal, "target_reached_goal");
      requestTargetHold(index, "target_reached_goal");
    }
  }
}

void InterceptMissionRefereeNode::updatePreviousPhysicalStates() {
  for (std::size_t index = 0U; index < interceptors_.size(); ++index) {
    interceptors_[index].previous_physical_state = interceptorPhysicalState(index);
  }
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    targets_[index].previous_physical_state = targetPhysicalState(index);
  }
}

void InterceptMissionRefereeNode::markTargetOutcome(
    const std::size_t index, const TargetOutcome outcome,
    const std::string& event_detail, const std::string& capturing_interceptor_id) {
  TargetRuntime& target = targets_[index];
  if (target.outcome != TargetOutcome::kActive) {
    return;
  }
  target.outcome = outcome;
  target.capturing_interceptor_id = capturing_interceptor_id;
  publishTargetStatus(index, event_detail);
  RCLCPP_INFO(get_logger(),
              "INTERCEPT_TARGET_OUTCOME target_id='%s' detection_id=%" PRIu64
              " outcome=%s first_target_terminal_event=true "
              "capturing_interceptor_id='%s' mission_epoch=%" PRIu64,
              target.id.c_str(), target.detection_id, targetOutcomeName(outcome),
              capturing_interceptor_id.empty() ? "none"
                                               : capturing_interceptor_id.c_str(),
              mission_epoch_);
  if (targets_.size() == 1U && mission_name_ == "intercept") {
    const char* legacy_outcome = "evader_crashed";
    if (outcome == TargetOutcome::kIntercepted) {
      legacy_outcome = "intercepted";
    } else if (outcome == TargetOutcome::kReachedGoal) {
      legacy_outcome = "evader_reached_goal";
    }
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_OUTCOME outcome=%s first_terminal_event=true "
                "reason='%s' live_interceptors=%zu epoch=%" PRIu64,
                legacy_outcome, event_detail.c_str(), operationalInterceptorCount(),
                mission_epoch_);
  }
}

void InterceptMissionRefereeNode::updateAggregateTerminal() {
  if (aggregate_outcome_.has_value() || system_failure_reason_.has_value()) {
    return;
  }
  if (activeTargetCount() == 0U) {
    const std::size_t intercepted = static_cast<std::size_t>(std::ranges::count(
        targets_, TargetOutcome::kIntercepted, &TargetRuntime::outcome));
    const std::size_t reached_goal = static_cast<std::size_t>(std::ranges::count(
        targets_, TargetOutcome::kReachedGoal, &TargetRuntime::outcome));
    if (targets_.size() == 1U) {
      if (intercepted == 1U) {
        aggregate_outcome_ = "intercepted";
      } else if (reached_goal == 1U) {
        aggregate_outcome_ = "evader_reached_goal";
      } else {
        aggregate_outcome_ = "evader_crashed";
      }
    } else if (intercepted == targets_.size()) {
      aggregate_outcome_ = "all_intercepted";
    } else if (reached_goal == targets_.size()) {
      aggregate_outcome_ = "all_reached_goal";
    } else {
      aggregate_outcome_ = "mixed";
    }
    requestHoldsForSurvivors("all_targets_terminal");
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_AGGREGATE_OUTCOME outcome=%s first_terminal_event=true "
                "targets=%zu live_interceptors=%zu mission_epoch=%" PRIu64,
                aggregate_outcome_.value().c_str(), targets_.size(),
                operationalInterceptorCount(), mission_epoch_);
    return;
  }
  if (operationalInterceptorCount() == 0U) {
    aggregate_outcome_ = "no_interceptors_remaining";
    requestHoldsForSurvivors("no_interceptors_remaining");
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_AGGREGATE_OUTCOME outcome=no_interceptors_remaining "
                "first_terminal_event=true active_targets=%zu mission_epoch=%" PRIu64,
                activeTargetCount(), mission_epoch_);
  }
}

bool InterceptMissionRefereeNode::allRequestedDeathsSettled() const {
  const auto settled = [](const auto& runtime) {
    return !runtime.destruction_requested ||
           (runtime.destroyed && runtime.state.has_value() &&
            !runtime.state.value().armed);
  };
  return std::ranges::all_of(interceptors_, settled) &&
         std::ranges::all_of(targets_, settled);
}

bool InterceptMissionRefereeNode::destructionSettlementTimedOut(
    const std::int64_t now_ns) const {
  const auto timed_out = [this, now_ns](const auto& runtime) {
    return runtime.destruction_requested &&
           (!runtime.destroyed || !runtime.state || runtime.state->armed) &&
           runtime.destruction_requested_ns > 0 &&
           now_ns - runtime.destruction_requested_ns >
               destruction_settlement_timeout_ns_;
  };
  return std::ranges::any_of(interceptors_, timed_out) ||
         std::ranges::any_of(targets_, timed_out);
}

bool InterceptMissionRefereeNode::allSurvivorsHeld(const std::int64_t now_ns) {
  for (InterceptorRuntime& interceptor : interceptors_) {
    if (interceptor.destroyed || interceptor.destruction_requested) {
      continue;
    }
    if (!interceptor.hold_confirmation || !interceptor.state) {
      return false;
    }
    std::optional<Point3> active_hold_position;
    if (interceptor.hold_horizon && interceptor.hold_horizon->active &&
        interceptor.hold_horizon->sequence >
            interceptor.hold_request_horizon_sequence) {
      active_hold_position = interceptor.hold_horizon->position;
    }
    const InterceptorHoldUpdate update =
        interceptor.hold_confirmation->update(*interceptor.state, active_hold_position);
    if (update.newly_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='%s' "
                  "position_error_m=%.3f speed_mps=%.3f mission_epoch=%" PRIu64,
                  interceptor.id.c_str(), update.position_error_m, update.speed_mps,
                  mission_epoch_);
    }
    if (!update.confirmed) {
      if (interceptor.hold_requested_ns > 0 &&
          now_ns - interceptor.hold_requested_ns > hold_timeout_ns_) {
        failMission("interceptor_hold_not_confirmed:" + interceptor.id);
      }
      return false;
    }
  }
  return true;
}

void InterceptMissionRefereeNode::settleTerminal(const std::int64_t now_ns) {
  if (!system_failure_reason_.has_value() && !aggregate_outcome_.has_value()) {
    if (destructionSettlementTimedOut(now_ns)) {
      system_failure_reason_ = "vehicle_destruction_not_confirmed";
      requestHoldsForSurvivors(*system_failure_reason_);
    }
    return;
  }
  if (system_failure_reason_.has_value()) {
    requestHoldsForSurvivors(system_failure_reason_.value());
  } else {
    requestHoldsForSurvivors(aggregate_outcome_.value());
  }
  if (destructionSettlementTimedOut(now_ns)) {
    failMission("vehicle_destruction_not_confirmed");
    return;
  }
  if (!allRequestedDeathsSettled() || !allSurvivorsHeld(now_ns)) {
    return;
  }
  if (system_failure_reason_.has_value()) {
    failMission(system_failure_reason_.value());
  } else {
    finishMission();
  }
}

void InterceptMissionRefereeNode::finishMission() {
  if (result_reported_ || !aggregate_outcome_.has_value()) {
    return;
  }
  result_reported_ = true;
  const std::size_t intercepted = static_cast<std::size_t>(std::ranges::count(
      targets_, TargetOutcome::kIntercepted, &TargetRuntime::outcome));
  const std::size_t reached_goal = static_cast<std::size_t>(std::ranges::count(
      targets_, TargetOutcome::kReachedGoal, &TargetRuntime::outcome));
  const std::size_t destroyed = static_cast<std::size_t>(
      std::ranges::count(targets_, TargetOutcome::kDestroyed, &TargetRuntime::outcome));
  const std::string& aggregate_outcome = aggregate_outcome_.value();
  const bool technical_success = aggregate_outcome != "no_interceptors_remaining";
  if (targets_.size() == 1U && mission_name_ == "intercept") {
    const std::string capturing_id = targets_.front().capturing_interceptor_id.empty()
                                         ? "none"
                                         : targets_.front().capturing_interceptor_id;
    if (technical_success) {
      RCLCPP_INFO(get_logger(),
                  "MISSION_RESULT success=true mission=intercept outcome=%s "
                  "intercept_success=%s capturing_interceptor_id='%s' "
                  "mission_epoch=%" PRIu64,
                  aggregate_outcome.c_str(), intercepted == 1U ? "true" : "false",
                  capturing_id.c_str(), mission_epoch_);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "MISSION_RESULT success=false mission=intercept outcome=%s "
                   "intercept_success=false capturing_interceptor_id='%s' "
                   "mission_epoch=%" PRIu64,
                   aggregate_outcome.c_str(), capturing_id.c_str(), mission_epoch_);
    }
  } else if (technical_success) {
    RCLCPP_INFO(get_logger(),
                "MISSION_RESULT success=true mission=%s outcome=%s "
                "intercepted_targets=%zu reached_goal_targets=%zu "
                "destroyed_targets=%zu target_count=%zu mission_epoch=%" PRIu64,
                mission_name_.c_str(), aggregate_outcome.c_str(), intercepted,
                reached_goal, destroyed, targets_.size(), mission_epoch_);
  } else {
    RCLCPP_ERROR(get_logger(),
                 "MISSION_RESULT success=false mission=%s outcome=%s "
                 "intercepted_targets=%zu reached_goal_targets=%zu "
                 "destroyed_targets=%zu target_count=%zu mission_epoch=%" PRIu64,
                 mission_name_.c_str(), aggregate_outcome.c_str(), intercepted,
                 reached_goal, destroyed, targets_.size(), mission_epoch_);
  }
  completeResultLifecycle();
}

void InterceptMissionRefereeNode::failMission(const std::string& reason) {
  if (result_reported_) {
    return;
  }
  result_reported_ = true;
  RCLCPP_ERROR(get_logger(),
               "MISSION_RESULT success=false mission=%s outcome=system_failure "
               "reason='%s' mission_epoch=%" PRIu64 " disarm_requested=false",
               mission_name_.c_str(), reason.c_str(), mission_epoch_);
  completeResultLifecycle();
}

void InterceptMissionRefereeNode::completeResultLifecycle() {
  if (shutdown_on_terminal_outcome_) {
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(),
              "INTERCEPT_MISSION state=terminal_observation "
              "simulation_shutdown_requested=false epoch=%" PRIu64,
              mission_epoch_);
}

void InterceptMissionRefereeNode::tick() {
  if (result_reported_) {
    return;
  }
  const std::int64_t now_ns = now().nanoseconds();
  if (boundary_check_started_ns_ <= 0) {
    boundary_check_started_ns_ = now_ns;
  }
  if (!verifyGroundTruthBoundary(now_ns)) {
    if (!result_reported_ &&
        now_ns - boundary_check_started_ns_ > boundary_startup_timeout_ns_) {
      failMission("ground_truth_boundary_not_ready");
    }
    return;
  }
  if (!mission_started_ && truth_alignment_mission_update_.startup_failure_confirmed) {
    handleStartupCoordinateAlignmentFailure();
    settleTerminal(now_ns);
    return;
  }
  const bool all_states_present =
      std::ranges::all_of(interceptors_,
                          [](const InterceptorRuntime& runtime) {
                            return runtime.state.has_value() &&
                                   runtime.truth_state.has_value();
                          }) &&
      std::ranges::all_of(targets_, [](const TargetRuntime& runtime) {
        return runtime.state.has_value() && runtime.truth_state.has_value();
      });
  if (!all_states_present) {
    return;
  }
  if (!mission_started_) {
    const bool navigation_ready =
        std::ranges::all_of(interceptors_,
                            [](const InterceptorRuntime& runtime) {
                              return runtime.state->navigation_ready;
                            }) &&
        std::ranges::all_of(targets_, [](const TargetRuntime& runtime) {
          return runtime.state->navigation_ready;
        });
    if (navigation_ready && mission_readiness_started_ns_ <= 0) {
      mission_readiness_started_ns_ = now_ns;
    }
    if (missionReady()) {
      publishMissionStart();
    } else if (navigation_ready && mission_readiness_started_ns_ > 0 &&
               now_ns - mission_readiness_started_ns_ > mission_readiness_timeout_ns_) {
      failMission("mission_readiness_timeout");
    }
    return;
  }
  if (aggregate_outcome_.has_value() || system_failure_reason_.has_value()) {
    detectPhysicalContacts();
    updatePreviousPhysicalStates();
    settleTerminal(now_ns);
    return;
  }
  if (!updateStateAdjudication(now_ns)) {
    settleTerminal(now_ns);
    return;
  }
  detectPhysicalContacts();
  evaluateTargetGoals();
  updatePreviousPhysicalStates();
  updateAggregateTerminal();
  settleTerminal(now_ns);
}

} // namespace drone_city_nav

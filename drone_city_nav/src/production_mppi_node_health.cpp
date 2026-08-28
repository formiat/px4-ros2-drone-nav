#include <cinttypes>
#include <limits>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

static_assert(static_cast<std::uint8_t>(NavigationReadinessStage::kProcessAlive) ==
              msg::NavigationHealth::STAGE_PROCESS_ALIVE);
static_assert(static_cast<std::uint8_t>(NavigationReadinessStage::kTerminalFailure) ==
              msg::NavigationHealth::STAGE_TERMINAL_FAILURE);
static_assert(static_cast<std::uint8_t>(NavigationTerminalFailure::kNone) ==
              msg::NavigationHealth::FAILURE_NONE);
static_assert(
    static_cast<std::uint8_t>(NavigationTerminalFailure::kRecoveryBudgetExhausted) ==
    msg::NavigationHealth::FAILURE_RECOVERY_BUDGET_EXHAUSTED);

} // namespace

NavigationHealthAssessment ProductionMppiNode::updateNavigationHealth(
    const std::shared_ptr<const ProductionNavigationObjective>& objective,
    const ProductionMppiAppliedControl& applied_control,
    const ProductionMppiExecutionHorizonOwner& execution_horizon_owner,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& execution_snapshot,
    const bool world_current, const std::int64_t now_ns) {
  if (navigation_health_supervisor_ == nullptr) {
    return {};
  }
  const bool certified_route_ready = execution_snapshot != nullptr &&
                                     execution_snapshot->valid() &&
                                     execution_snapshot->route.has_value();
  const bool horizon_acknowledged =
      certified_route_ready && execution_horizon_owner.valid &&
      execution_horizon_owner.execution_mode ==
          msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
      appliedControlAuthoritativeForExecution(applied_control, execution_horizon_owner,
                                              now_ns,
                                              maximum_control_feedback_age_ms_) &&
      applied_control.execution_mode ==
          msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  const NavigationHealthAssessment assessment =
      navigation_health_supervisor_->update(NavigationHealthObservation{
          .mission_epoch = objective != nullptr ? objective->mission_epoch : 0U,
          .recovery_sequence =
              navigation_recovery_sequence_.load(std::memory_order_acquire),
          .now_ns = now_ns,
          .mission_active = objective != nullptr,
          .process_alive = true,
          .world_ready = world_current,
          .bootstrap_ready = vehicle_navigation_ready_.load(std::memory_order_acquire),
          .certified_route_ready = certified_route_ready,
          .horizon_acknowledged = horizon_acknowledged,
      });
  const bool changed =
      !last_navigation_health_assessment_.has_value() ||
      last_navigation_health_assessment_->stage != assessment.stage ||
      last_navigation_health_assessment_->failure != assessment.failure ||
      last_navigation_health_assessment_->mission_epoch != assessment.mission_epoch ||
      last_navigation_health_assessment_->recovery_attempts !=
          assessment.recovery_attempts ||
      last_navigation_health_assessment_->terminal != assessment.terminal;
  if (changed) {
    publishNavigationHealth(assessment);
    last_navigation_health_assessment_ = assessment;
  }
  return assessment;
}

void ProductionMppiNode::publishNavigationHealth(
    const NavigationHealthAssessment& assessment) {
  if (navigation_health_pub_ == nullptr ||
      navigation_health_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return;
  }
  msg::NavigationHealth message;
  message.header.stamp = get_clock()->now();
  message.header.frame_id = frame_id_;
  message.producer_instance_id = navigation_health_producer_instance_id_;
  message.sequence = ++navigation_health_sequence_;
  message.mission_epoch = assessment.mission_epoch;
  message.stage = static_cast<std::uint8_t>(assessment.stage);
  message.failure_reason = static_cast<std::uint8_t>(assessment.failure);
  message.recovery_attempts = assessment.recovery_attempts;
  message.stage_age_ms = assessment.stage_age_ms;
  message.mission_ready = assessment.mission_ready;
  message.terminal = assessment.terminal;
  navigation_health_pub_->publish(message);
  RCLCPP_INFO(get_logger(),
              "NAVIGATION_HEALTH stage=%s mission_ready=%s terminal=%s "
              "failure=%s mission_epoch=%" PRIu64 " recovery_attempts=%u "
              "stage_age_ms=%.1f",
              navigationReadinessStageName(assessment.stage),
              assessment.mission_ready ? "true" : "false",
              assessment.terminal ? "true" : "false",
              navigationTerminalFailureName(assessment.failure),
              assessment.mission_epoch, assessment.recovery_attempts,
              assessment.stage_age_ms);
}

} // namespace drone_city_nav

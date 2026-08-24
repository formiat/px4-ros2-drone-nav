#include "intercept_mission_referee_node.hpp"

#include "drone_city_nav/execution_horizon_contract_ros.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "intercept_referee_support.hpp"
#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

void requireUniqueIds(const std::vector<std::string>& ids, const std::string& label) {
  std::unordered_set<std::string> unique;
  for (const std::string& id : ids) {
    if (id.empty() || !unique.insert(id).second) {
      throw std::invalid_argument{label + " must be non-empty and unique"};
    }
  }
}

[[nodiscard]] std::vector<std::string>
vehicleTopics(const std::vector<std::string>& ids, const std::string& suffix) {
  std::vector<std::string> topics;
  topics.reserve(ids.size());
  for (const std::string& id : ids) {
    topics.push_back("/vehicles/" + id + suffix);
  }
  return topics;
}

[[nodiscard]] const char* witnessRejectionReason(
    const ExecutionHorizonWitnessAdmissionResult& admission) noexcept {
  if (admission.stale) {
    return "stale";
  }
  if (admission.replay) {
    return "replay";
  }
  if (admission.conflict) {
    return "conflict";
  }
  if (admission.non_current_session) {
    return "non_current_session";
  }
  return "invalid";
}

} // namespace

InterceptMissionRefereeNode::InterceptMissionRefereeNode()
    : Node{"intercept_mission_referee_node"} {
  const std::int64_t mission_epoch =
      declare_parameter<std::int64_t>("mission_epoch", 1);
  if (mission_epoch <= 0) {
    throw std::invalid_argument{"mission epoch must be positive"};
  }
  mission_epoch_ = static_cast<std::uint64_t>(mission_epoch);
  mission_name_ = declare_parameter<std::string>("mission_name", "intercept");
  if (mission_name_.empty()) {
    throw std::invalid_argument{"mission name must be non-empty"};
  }
  capture_radius_m_ = declare_parameter<double>("capture_radius_m", 5.0);
  target_goal_radius_m_ = declare_parameter<double>("evader_goal_radius_m", 2.0);
  if (!(capture_radius_m_ > 0.0) || !(target_goal_radius_m_ > 0.0) ||
      !std::isfinite(capture_radius_m_) || !std::isfinite(target_goal_radius_m_)) {
    throw std::invalid_argument{"mission radii must be finite and positive"};
  }
  state_config_.maximum_state_age_s =
      declare_parameter<double>("maximum_state_age_s", 1.0);
  maximum_offboard_feedback_age_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("maximum_offboard_feedback_age_s", 1.0));
  state_config_.maximum_degraded_duration_s =
      declare_parameter<double>("maximum_degraded_state_duration_s", 5.0);
  hold_config_.position_tolerance_m =
      declare_parameter<double>("interceptor_hold_position_tolerance_m", 2.0);
  hold_config_.maximum_speed_mps =
      declare_parameter<double>("interceptor_hold_maximum_speed_mps", 0.8);
  hold_config_.confirmation_duration_s =
      declare_parameter<double>("interceptor_hold_confirmation_duration_s", 1.0);
  destruction_settlement_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("destruction_settlement_timeout_s", 5.0));
  hold_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("interceptor_hold_timeout_s", 20.0));
  boundary_startup_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("ground_truth_boundary_startup_timeout_s", 10.0));
  mission_readiness_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("mission_readiness_timeout_s", 30.0));
  shutdown_on_terminal_outcome_ =
      declare_parameter<bool>("shutdown_on_terminal_outcome", true);
  target_avoidance_pipeline_enabled_ =
      declare_parameter<bool>("target_avoidance_pipeline_enabled", false);

  target_status_pub_ = create_publisher<msg::InterceptTargetStatus>(
      declare_parameter<std::string>("target_status_topic", "/intercept/target_status"),
      rclcpp::QoS{100}.reliable().transient_local());
  const std::vector<std::string> target_ids =
      declare_parameter<std::vector<std::string>>("target_ids", {"evader"});
  const std::vector<std::int64_t> target_detection_ids =
      declare_parameter<std::vector<std::int64_t>>("target_detection_ids", {1});
  const std::vector<double> target_goals =
      declare_parameter<std::vector<double>>("target_goals_xyz_m", {54.0, 378.0, 18.0});
  configureTargets(target_ids, target_detection_ids, target_goals);
  configureInterceptors(declare_parameter<std::vector<std::string>>(
      "interceptor_ids", {"interceptor_0", "interceptor_1", "interceptor_2"}));

  truth_alignment_sub_ = create_subscription<msg::SimulationTruthAlignment>(
      declare_parameter<std::string>("truth_alignment_status_topic",
                                     "/simulation_truth/alignment"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::SimulationTruthAlignment::SharedPtr status) {
        onTruthAlignmentStatus(*status);
      });
  configureGroundTruthBoundary();
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    publishTargetObjective(index);
  }
  timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
  RCLCPP_INFO(get_logger(),
              "Intercept mission referee ready: mission='%s' epoch=%" PRIu64
              " interceptors=%zu targets=%zu capture_radius_m=%.2f "
              "target_avoidance_pipeline_enabled=%s",
              mission_name_.c_str(), mission_epoch_, interceptors_.size(),
              targets_.size(), capture_radius_m_,
              target_avoidance_pipeline_enabled_ ? "true" : "false");
}

void InterceptMissionRefereeNode::configureInterceptors(
    const std::vector<std::string>& ids) {
  if (ids.empty()) {
    throw std::invalid_argument{"at least one interceptor is required"};
  }
  requireUniqueIds(ids, "interceptor ids");
  const InterceptorTopicConfig topics = declareInterceptorTopicConfig(*this, ids);
  const std::vector<std::string> control_feedback_topics =
      declare_parameter<std::vector<std::string>>(
          "interceptor_applied_control_feedback_topics",
          vehicleTopics(ids, "/mppi/applied_control"));
  if (control_feedback_topics.size() != ids.size()) {
    throw std::invalid_argument{
        "interceptor_applied_control_feedback_topics must match interceptor ids"};
  }
  const auto state_qos = rclcpp::QoS{10}.best_effort();
  const auto control_feedback_qos = rclcpp::QoS{10}.reliable();
  const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
  interceptors_.resize(ids.size());
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    InterceptorRuntime& runtime = interceptors_[index];
    runtime.id = ids[index];
    runtime.radar_simulator_fqn = topics.radar_simulator_fqn[index];
    runtime.truth_state_topic = topics.physical_truth_state[index];
    runtime.target_adjudications.reserve(targets_.size());
    for (std::size_t target_index = 0U; target_index < targets_.size();
         ++target_index) {
      runtime.target_adjudications.push_back(
          std::make_unique<InterceptStateAdjudicationLifecycle>(state_config_));
    }
    runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
        topics.navigation_state[index], state_qos,
        [this, index](const msg::VehicleNavigationState::SharedPtr state) {
          interceptors_[index].state = detail::vehicleState(*state);
        });
    runtime.truth_state_sub = create_subscription<msg::SimulationTruthState>(
        topics.physical_truth_state[index], state_qos,
        [this, index](const msg::SimulationTruthState::SharedPtr state) {
          if (state->vehicle_id == interceptors_[index].id) {
            interceptors_[index].truth_state = detail::physicalTruthState(*state);
          }
        });
    runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
        topics.execution_horizon[index], control_feedback_qos,
        [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          onInterceptorExecutionHorizon(*horizon, index);
        });
    runtime.control_feedback_sub = create_subscription<msg::MppiControlFeedback>(
        control_feedback_topics[index], control_feedback_qos,
        [this, index](const msg::MppiControlFeedback::SharedPtr feedback) {
          onInterceptorControlFeedback(*feedback, index);
        });
    runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.world_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          interceptors_[index].world_ready = ready->data;
        });
    runtime.track_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.track_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          interceptors_[index].track_ready = ready->data;
        });
    runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
        topics.destroyed[index], latched_qos,
        [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, true, index);
        });
    runtime.start_pub =
        create_publisher<std_msgs::msg::Bool>(topics.mission_start[index], latched_qos);
    runtime.destroyed_pub =
        create_publisher<msg::VehicleDestroyed>(topics.destroyed[index], latched_qos);
    runtime.command_pub = create_publisher<msg::InterceptMissionCommand>(
        topics.mission_command[index], latched_qos);
  }
}

void InterceptMissionRefereeNode::configureTargets(
    const std::vector<std::string>& ids, const std::vector<std::int64_t>& detection_ids,
    const std::vector<double>& goals_xyz) {
  if (ids.empty() || detection_ids.size() != ids.size() ||
      goals_xyz.size() != ids.size() * 3U) {
    throw std::invalid_argument{
        "target ids, detection ids, and flattened goals must have matching sizes"};
  }
  requireUniqueIds(ids, "target ids");
  const TargetTopicConfig topics = declareTargetTopicConfig(*this, ids);
  const std::vector<std::string> control_feedback_topics =
      declare_parameter<std::vector<std::string>>(
          "target_applied_control_feedback_topics",
          vehicleTopics(ids, "/mppi/applied_control"));
  if (control_feedback_topics.size() != ids.size()) {
    throw std::invalid_argument{
        "target_applied_control_feedback_topics must match target ids"};
  }
  const auto state_qos = rclcpp::QoS{10}.best_effort();
  const auto control_feedback_qos = rclcpp::QoS{10}.reliable();
  const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
  std::unordered_set<std::uint64_t> unique_detection_ids;
  targets_.resize(ids.size());
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    if (detection_ids[index] <= 0) {
      throw std::invalid_argument{"target detection ids must be positive"};
    }
    const auto detection_id = static_cast<std::uint64_t>(detection_ids[index]);
    if (!unique_detection_ids.insert(detection_id).second) {
      throw std::invalid_argument{"target detection ids must be unique"};
    }
    const Point3 goal{goals_xyz[index * 3U], goals_xyz[index * 3U + 1U],
                      goals_xyz[index * 3U + 2U]};
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.z)) {
      throw std::invalid_argument{"target goals must be finite"};
    }
    TargetRuntime& runtime = targets_[index];
    runtime.id = ids[index];
    runtime.avoidance_radar_simulator_fqn = topics.avoidance_radar_simulator_fqn[index];
    runtime.avoidance_tracker_fqn = topics.avoidance_tracker_fqn[index];
    runtime.state_topic = topics.navigation_state[index];
    runtime.truth_state_topic = topics.physical_truth_state[index];
    runtime.goal = goal;
    runtime.detection_id = detection_id;
    runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
        topics.navigation_state[index], state_qos,
        [this, index](const msg::VehicleNavigationState::SharedPtr state) {
          targets_[index].state = detail::vehicleState(*state);
        });
    runtime.truth_state_sub = create_subscription<msg::SimulationTruthState>(
        topics.physical_truth_state[index], state_qos,
        [this, index](const msg::SimulationTruthState::SharedPtr state) {
          if (state->vehicle_id == targets_[index].id) {
            targets_[index].truth_state = detail::physicalTruthState(*state);
          }
        });
    runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
        topics.execution_horizon[index], control_feedback_qos,
        [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          onTargetExecutionHorizon(*horizon, index);
        });
    runtime.control_feedback_sub = create_subscription<msg::MppiControlFeedback>(
        control_feedback_topics[index], control_feedback_qos,
        [this, index](const msg::MppiControlFeedback::SharedPtr feedback) {
          onTargetControlFeedback(*feedback, index);
        });
    runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.world_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          targets_[index].world_ready = ready->data;
        });
    runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
        topics.destroyed[index], latched_qos,
        [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, false, index);
        });
    runtime.objective_pub = create_publisher<msg::NavigationObjective>(
        topics.objective[index], latched_qos);
    runtime.start_pub =
        create_publisher<std_msgs::msg::Bool>(topics.mission_start[index], latched_qos);
    runtime.destroyed_pub =
        create_publisher<msg::VehicleDestroyed>(topics.destroyed[index], latched_qos);
  }
}

void InterceptMissionRefereeNode::revokeExecutionHorizonEvidence(
    InterceptorRuntime& interceptor) {
  interceptor.hold_horizon.reset();
  interceptor.accepted_horizon.reset();
  interceptor.executable_horizon_ready = false;
  interceptor.executable_horizon_valid_from_ns = 0;
  interceptor.executable_horizon_valid_until_ns = 0;
  if (interceptor.hold_confirmation) {
    interceptor.hold_confirmation =
        std::make_unique<InterceptorHoldConfirmation>(hold_config_);
  }
}

void InterceptMissionRefereeNode::revokeExecutionHorizonEvidence(
    TargetRuntime& target) {
  target.accepted_horizon.reset();
  target.executable_horizon_ready = false;
  target.executable_horizon_valid_from_ns = 0;
  target.executable_horizon_valid_until_ns = 0;
}

void InterceptMissionRefereeNode::refreshExecutionHorizonEvidence(
    InterceptorRuntime& interceptor, const std::int64_t now_ns) {
  if (!interceptor.accepted_horizon.has_value()) {
    interceptor.executable_horizon_ready = false;
    if (interceptor.hold_horizon.has_value()) {
      interceptor.hold_horizon->witnessed = false;
    }
    return;
  }
  AcceptedHorizon& horizon = *interceptor.accepted_horizon;
  const ExecutionHorizonWitnessRequirement requirement{
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .horizon_producer_instance_id = horizon.producer_instance_id,
      .horizon_sequence = horizon.sequence,
      .valid_from_ns = horizon.valid_from_ns,
      .valid_until_ns = horizon.valid_until_ns,
      .execution_mode =
          horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED
              ? ExecutionHorizonWitnessMode::kPlanned
              : ExecutionHorizonWitnessMode::kPositionHold,
  };
  const bool receipt =
      executionHorizonReceiptFreshAt(interceptor.horizon_witness_state, requirement,
                                     now_ns, maximum_offboard_feedback_age_ns_);
  const bool strict_witness =
      executionHorizonWitnessFreshAt(interceptor.horizon_witness_state, requirement,
                                     now_ns, maximum_offboard_feedback_age_ns_);
  const bool ready_witness = mission_started_ ? strict_witness : receipt;
  horizon.witnessed = ready_witness;
  const bool was_hold_actionable = interceptor.hold_horizon.has_value() &&
                                   interceptor.hold_horizon->active &&
                                   interceptor.hold_horizon->witnessed;
  if (interceptor.hold_horizon.has_value()) {
    interceptor.hold_horizon->witnessed = strict_witness;
  }
  const bool hold_actionable = interceptor.hold_horizon.has_value() &&
                               interceptor.hold_horizon->active && strict_witness;
  if (was_hold_actionable && !hold_actionable && interceptor.hold_confirmation) {
    interceptor.hold_confirmation =
        std::make_unique<InterceptorHoldConfirmation>(hold_config_);
  }
  interceptor.executable_horizon_ready =
      ready_witness &&
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED;
}

void InterceptMissionRefereeNode::refreshExecutionHorizonEvidence(
    TargetRuntime& target, const std::int64_t now_ns) {
  if (!target.accepted_horizon.has_value()) {
    target.executable_horizon_ready = false;
    return;
  }
  AcceptedHorizon& horizon = *target.accepted_horizon;
  const ExecutionHorizonWitnessRequirement requirement{
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .horizon_producer_instance_id = horizon.producer_instance_id,
      .horizon_sequence = horizon.sequence,
      .valid_from_ns = horizon.valid_from_ns,
      .valid_until_ns = horizon.valid_until_ns,
      .execution_mode =
          horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED
              ? ExecutionHorizonWitnessMode::kPlanned
              : ExecutionHorizonWitnessMode::kPositionHold,
  };
  const bool receipt =
      executionHorizonReceiptFreshAt(target.horizon_witness_state, requirement, now_ns,
                                     maximum_offboard_feedback_age_ns_);
  const bool strict_witness =
      executionHorizonWitnessFreshAt(target.horizon_witness_state, requirement, now_ns,
                                     maximum_offboard_feedback_age_ns_);
  horizon.witnessed = mission_started_ ? strict_witness : receipt;
  target.executable_horizon_ready =
      horizon.witnessed &&
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED;
}

void InterceptMissionRefereeNode::expireOffboardEvidence(const std::int64_t now_ns) {
  for (InterceptorRuntime& interceptor : interceptors_) {
    refreshExecutionHorizonEvidence(interceptor, now_ns);
  }
  for (TargetRuntime& target : targets_) {
    refreshExecutionHorizonEvidence(target, now_ns);
  }
}

void InterceptMissionRefereeNode::onInterceptorControlFeedback(
    const msg::MppiControlFeedback& feedback, const std::size_t interceptor_index) {
  InterceptorRuntime& interceptor = interceptors_.at(interceptor_index);
  const std::int64_t receive_stamp_ns = now().nanoseconds();
  const ExecutionControlFeedbackAssessment assessment =
      assessExecutionControlFeedback(feedback, "map", receive_stamp_ns);
  if (!assessment.valid()) {
    const ExecutionHorizonWitnessAdmissionResult malformed =
        revokeMalformedExecutionHorizonFeedback(interceptor.horizon_witness_state,
                                                assessment.candidate, receive_stamp_ns);
    if (malformed.state_advanced) {
      interceptor.horizon_witness_state = malformed.next_state;
      if (malformed.witness_revoked) {
        refreshExecutionHorizonEvidence(interceptor, receive_stamp_ns);
      }
    }
    const std::string_view reason =
        executionControlFeedbackStatusName(assessment.status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_CONTROL_FEEDBACK rejected=true role=interceptor "
        "vehicle_id='%s' offboard_producer=%" PRIu64 " horizon_producer=%" PRIu64
        " sequence=%" PRIu64 " reason=%.*s",
        interceptor.id.c_str(), feedback.producer_instance_id,
        feedback.horizon_producer_instance_id, feedback.horizon_sequence,
        static_cast<int>(reason.size()), reason.data());
    return;
  }
  const ExecutionHorizonWitnessAdmissionResult admission =
      admitExecutionHorizonFeedbackPayload(interceptor.horizon_witness_state,
                                           assessment.candidate, assessment.valid());
  if (!admission.accept) {
    if (admission.state_advanced) {
      interceptor.horizon_witness_state = admission.next_state;
      if (admission.witness_revoked) {
        refreshExecutionHorizonEvidence(interceptor, receive_stamp_ns);
      }
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "INTERCEPT_CONTROL_FEEDBACK rejected=true role=interceptor "
                         "vehicle_id='%s' offboard_producer=%" PRIu64
                         " horizon_producer=%" PRIu64 " sequence=%" PRIu64 " reason=%s",
                         interceptor.id.c_str(), feedback.producer_instance_id,
                         feedback.horizon_producer_instance_id,
                         feedback.horizon_sequence, witnessRejectionReason(admission));
    return;
  }
  interceptor.horizon_witness_state = admission.next_state;
  if (admission.session_transitioned && interceptor.accepted_horizon.has_value() &&
      interceptor.accepted_horizon->target_offboard_instance_id !=
          interceptor.horizon_witness_state.offboard_session
              .current_producer_instance_id) {
    revokeExecutionHorizonEvidence(interceptor);
  }
  refreshExecutionHorizonEvidence(interceptor, receive_stamp_ns);
}

void InterceptMissionRefereeNode::onTargetControlFeedback(
    const msg::MppiControlFeedback& feedback, const std::size_t target_index) {
  TargetRuntime& target = targets_.at(target_index);
  const std::int64_t receive_stamp_ns = now().nanoseconds();
  const ExecutionControlFeedbackAssessment assessment =
      assessExecutionControlFeedback(feedback, "map", receive_stamp_ns);
  if (!assessment.valid()) {
    const ExecutionHorizonWitnessAdmissionResult malformed =
        revokeMalformedExecutionHorizonFeedback(target.horizon_witness_state,
                                                assessment.candidate, receive_stamp_ns);
    if (malformed.state_advanced) {
      target.horizon_witness_state = malformed.next_state;
      if (malformed.witness_revoked) {
        refreshExecutionHorizonEvidence(target, receive_stamp_ns);
      }
    }
    const std::string_view reason =
        executionControlFeedbackStatusName(assessment.status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_CONTROL_FEEDBACK rejected=true role=target vehicle_id='%s' "
        "offboard_producer=%" PRIu64 " horizon_producer=%" PRIu64 " sequence=%" PRIu64
        " reason=%.*s",
        target.id.c_str(), feedback.producer_instance_id,
        feedback.horizon_producer_instance_id, feedback.horizon_sequence,
        static_cast<int>(reason.size()), reason.data());
    return;
  }
  const ExecutionHorizonWitnessAdmissionResult admission =
      admitExecutionHorizonFeedbackPayload(target.horizon_witness_state,
                                           assessment.candidate, assessment.valid());
  if (!admission.accept) {
    if (admission.state_advanced) {
      target.horizon_witness_state = admission.next_state;
      if (admission.witness_revoked) {
        refreshExecutionHorizonEvidence(target, receive_stamp_ns);
      }
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_CONTROL_FEEDBACK rejected=true role=target vehicle_id='%s' "
        "offboard_producer=%" PRIu64 " horizon_producer=%" PRIu64 " sequence=%" PRIu64
        " reason=%s",
        target.id.c_str(), feedback.producer_instance_id,
        feedback.horizon_producer_instance_id, feedback.horizon_sequence,
        witnessRejectionReason(admission));
    return;
  }
  target.horizon_witness_state = admission.next_state;
  if (admission.session_transitioned && target.accepted_horizon.has_value() &&
      target.accepted_horizon->target_offboard_instance_id !=
          target.horizon_witness_state.offboard_session.current_producer_instance_id) {
    revokeExecutionHorizonEvidence(target);
  }
  refreshExecutionHorizonEvidence(target, receive_stamp_ns);
}

void InterceptMissionRefereeNode::onInterceptorExecutionHorizon(
    const msg::MppiTrajectoryHorizon& horizon, const std::size_t interceptor_index) {
  InterceptorRuntime& interceptor = interceptors_.at(interceptor_index);
  const std::int64_t now_ns = now().nanoseconds();
  const ExecutionHorizonAdmissionCandidate candidate{
      .producer_instance_id = horizon.producer_instance_id,
      .sequence = horizon.sequence,
      .source_stamp_ns = executionHorizonTimeNanoseconds(horizon.header.stamp),
      .valid_from_ns = executionHorizonTimeNanoseconds(horizon.valid_from),
      .content_fingerprint = executionHorizonContentFingerprint(horizon),
  };
  const ExecutionHorizonPayloadStatus payload_status = assessExecutionHorizonPayload(
      horizon, ExecutionHorizonPayloadValidationConfig{.expected_frame_id = "map"});
  const bool revoked =
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  const std::int64_t valid_until_ns =
      executionHorizonTimeNanoseconds(horizon.valid_until);
  const bool expired = !revoked && now_ns >= valid_until_ns;
  const bool payload_admissible =
      payload_status == ExecutionHorizonPayloadStatus::kValid && !expired;
  const ExecutionHorizonAdmissionResult admission = admitExecutionHorizonIdentity(
      interceptor.horizon_admission, candidate, payload_admissible);
  if (admission.state_advanced) {
    interceptor.horizon_admission = admission.next_state;
  }
  if (admission.revoke) {
    revokeExecutionHorizonEvidence(interceptor);
  }
  if (admission.replay) {
    return;
  }
  if (!admission.accept_identity &&
      (!candidate.valid() || payload_admissible || admission.stale ||
       admission.conflict || admission.retired_capacity_exhausted ||
       admission.prospective_capacity_exhausted)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_EXECUTION_HORIZON rejected=true role=interceptor "
        "vehicle_id='%s' producer=%" PRIu64 " sequence=%" PRIu64
        " current_producer=%" PRIu64 " current_sequence=%" PRIu64 " reason=%s",
        interceptor.id.c_str(), horizon.producer_instance_id, horizon.sequence,
        interceptor.horizon_admission.current_producer_instance_id,
        interceptor.horizon_admission.current_sequence,
        admission.conflict                     ? "identity_content_conflict"
        : admission.stale                      ? "stale_identity"
        : admission.retired_capacity_exhausted ? "retired_identity_capacity_exhausted"
        : admission.prospective_capacity_exhausted
            ? "prospective_identity_capacity_exhausted"
            : "invalid_identity");
    return;
  }
  if (payload_status != ExecutionHorizonPayloadStatus::kValid || expired) {
    const std::string_view reason =
        payload_status == ExecutionHorizonPayloadStatus::kValid
            ? std::string_view{"expired_validity_window"}
            : executionHorizonPayloadStatusName(payload_status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_EXECUTION_HORIZON rejected=true role=interceptor "
        "vehicle_id='%s' producer=%" PRIu64 " sequence=%" PRIu64 " reason=%.*s",
        interceptor.id.c_str(), horizon.producer_instance_id, horizon.sequence,
        static_cast<int>(reason.size()), reason.data());
    return;
  }
  if (!admission.payload_installable) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "INTERCEPT_EXECUTION_HORIZON rejected=true role=interceptor "
                         "vehicle_id='%s' producer=%" PRIu64 " sequence=%" PRIu64
                         " reason=nonadvancing_producer_timestamps",
                         interceptor.id.c_str(), horizon.producer_instance_id,
                         horizon.sequence);
    return;
  }
  if (revoked) {
    revokeExecutionHorizonEvidence(interceptor);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "INTERCEPT_EXECUTION_HORIZON revoked=true role=interceptor vehicle_id='%s' "
        "producer=%" PRIu64 " sequence=%" PRIu64,
        interceptor.id.c_str(), horizon.producer_instance_id, horizon.sequence);
    return;
  }

  const std::int64_t valid_from_ns =
      executionHorizonTimeNanoseconds(horizon.valid_from);
  interceptor.accepted_horizon = AcceptedHorizon{
      .producer_instance_id = horizon.producer_instance_id,
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .sequence = horizon.sequence,
      .execution_mode = horizon.execution_mode,
      .valid_from_ns = valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .witnessed = false,
  };
  interceptor.hold_horizon = HoldHorizon{
      .position =
          Point3{horizon.stationary_hold_position.x, horizon.stationary_hold_position.y,
                 horizon.stationary_hold_position.z},
      .producer_instance_id = horizon.producer_instance_id,
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .sequence = horizon.sequence,
      .execution_mode = horizon.execution_mode,
      .valid_from_ns = valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .active = horizon.stationary_position_hold &&
                horizon.execution_mode ==
                    msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD,
      .witnessed = false,
  };
  interceptor.executable_horizon_ready = false;
  interceptor.executable_horizon_valid_from_ns = valid_from_ns;
  interceptor.executable_horizon_valid_until_ns = valid_until_ns;
  refreshExecutionHorizonEvidence(interceptor, now_ns);
}

void InterceptMissionRefereeNode::onTargetExecutionHorizon(
    const msg::MppiTrajectoryHorizon& horizon, const std::size_t target_index) {
  TargetRuntime& target = targets_.at(target_index);
  const std::int64_t now_ns = now().nanoseconds();
  const ExecutionHorizonAdmissionCandidate candidate{
      .producer_instance_id = horizon.producer_instance_id,
      .sequence = horizon.sequence,
      .source_stamp_ns = executionHorizonTimeNanoseconds(horizon.header.stamp),
      .valid_from_ns = executionHorizonTimeNanoseconds(horizon.valid_from),
      .content_fingerprint = executionHorizonContentFingerprint(horizon),
  };
  const ExecutionHorizonPayloadStatus payload_status = assessExecutionHorizonPayload(
      horizon, ExecutionHorizonPayloadValidationConfig{.expected_frame_id = "map"});
  const bool revoked =
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  const std::int64_t valid_until_ns =
      executionHorizonTimeNanoseconds(horizon.valid_until);
  const bool expired = !revoked && now_ns >= valid_until_ns;
  const bool payload_admissible =
      payload_status == ExecutionHorizonPayloadStatus::kValid && !expired;
  const ExecutionHorizonAdmissionResult admission = admitExecutionHorizonIdentity(
      target.horizon_admission, candidate, payload_admissible);
  if (admission.state_advanced) {
    target.horizon_admission = admission.next_state;
  }
  if (admission.revoke) {
    revokeExecutionHorizonEvidence(target);
  }
  if (admission.replay) {
    return;
  }
  if (!admission.accept_identity &&
      (!candidate.valid() || payload_admissible || admission.stale ||
       admission.conflict || admission.retired_capacity_exhausted ||
       admission.prospective_capacity_exhausted)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_EXECUTION_HORIZON rejected=true role=target vehicle_id='%s' "
        "producer=%" PRIu64 " sequence=%" PRIu64 " current_producer=%" PRIu64
        " current_sequence=%" PRIu64 " reason=%s",
        target.id.c_str(), horizon.producer_instance_id, horizon.sequence,
        target.horizon_admission.current_producer_instance_id,
        target.horizon_admission.current_sequence,
        admission.conflict                     ? "identity_content_conflict"
        : admission.stale                      ? "stale_identity"
        : admission.retired_capacity_exhausted ? "retired_identity_capacity_exhausted"
        : admission.prospective_capacity_exhausted
            ? "prospective_identity_capacity_exhausted"
            : "invalid_identity");
    return;
  }
  if (payload_status != ExecutionHorizonPayloadStatus::kValid || expired) {
    const std::string_view reason =
        payload_status == ExecutionHorizonPayloadStatus::kValid
            ? std::string_view{"expired_validity_window"}
            : executionHorizonPayloadStatusName(payload_status);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_EXECUTION_HORIZON rejected=true role=target vehicle_id='%s' "
        "producer=%" PRIu64 " sequence=%" PRIu64 " reason=%.*s",
        target.id.c_str(), horizon.producer_instance_id, horizon.sequence,
        static_cast<int>(reason.size()), reason.data());
    return;
  }
  if (!admission.payload_installable) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "INTERCEPT_EXECUTION_HORIZON rejected=true role=target vehicle_id='%s' "
        "producer=%" PRIu64 " sequence=%" PRIu64
        " reason=nonadvancing_producer_timestamps",
        target.id.c_str(), horizon.producer_instance_id, horizon.sequence);
    return;
  }
  if (revoked) {
    revokeExecutionHorizonEvidence(target);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "INTERCEPT_EXECUTION_HORIZON revoked=true role=target vehicle_id='%s' "
        "producer=%" PRIu64 " sequence=%" PRIu64,
        target.id.c_str(), horizon.producer_instance_id, horizon.sequence);
    return;
  }

  const std::int64_t valid_from_ns =
      executionHorizonTimeNanoseconds(horizon.valid_from);
  target.accepted_horizon = AcceptedHorizon{
      .producer_instance_id = horizon.producer_instance_id,
      .target_offboard_instance_id = horizon.target_offboard_instance_id,
      .sequence = horizon.sequence,
      .execution_mode = horizon.execution_mode,
      .valid_from_ns = valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .witnessed = false,
  };
  target.executable_horizon_ready = false;
  target.executable_horizon_valid_from_ns = valid_from_ns;
  target.executable_horizon_valid_until_ns = valid_until_ns;
  refreshExecutionHorizonEvidence(target, now_ns);
}

void InterceptMissionRefereeNode::configureGroundTruthBoundary() {
  std::vector<std::string> avoidance_radar_simulator_fqns;
  if (target_avoidance_pipeline_enabled_) {
    avoidance_radar_simulator_fqns.reserve(targets_.size());
    for (const TargetRuntime& target : targets_) {
      avoidance_radar_simulator_fqns.push_back(target.avoidance_radar_simulator_fqn);
    }
  }
  std::vector<InterceptorTruthEndpoint> interceptor_endpoints;
  interceptor_endpoints.reserve(interceptors_.size());
  for (const InterceptorRuntime& interceptor : interceptors_) {
    interceptor_endpoints.push_back(InterceptorTruthEndpoint{
        .physical_truth_topic = interceptor.truth_state_topic,
        .radar_simulator_fqn = interceptor.radar_simulator_fqn,
    });
  }
  std::vector<TargetTruthEndpoint> target_endpoints;
  target_endpoints.reserve(targets_.size());
  for (const TargetRuntime& target : targets_) {
    target_endpoints.push_back(TargetTruthEndpoint{
        .navigation_topic = target.state_topic,
        .physical_truth_topic = target.truth_state_topic,
        .own_radar_simulator_fqn = target_avoidance_pipeline_enabled_
                                       ? target.avoidance_radar_simulator_fqn
                                       : std::string{},
        .own_tracker_fqn = target_avoidance_pipeline_enabled_
                               ? target.avoidance_tracker_fqn
                               : std::string{},
    });
  }
  ground_truth_boundary_ = makeInterceptGroundTruthBoundary(
      get_fully_qualified_name(),
      declare_parameter<std::string>("simulation_truth_adapter_node_fqn",
                                     "/simulation_truth_adapter_node"),
      target_endpoints, interceptor_endpoints, avoidance_radar_simulator_fqns,
      declare_parameter<std::vector<std::string>>("target_navigation_observer_fqns",
                                                  std::vector<std::string>{}));
}

void InterceptMissionRefereeNode::onTruthAlignmentStatus(
    const msg::SimulationTruthAlignment& status) {
  truth_alignment_reason_ = status.reason;
  truth_alignment_vehicle_id_ = status.vehicle_id;
  truth_alignment_maximum_error_m_ = status.maximum_position_error_m;
  truth_alignment_mission_update_ =
      truth_alignment_lifecycle_.update(SimulationTruthAlignmentObservation{
          .ready = status.ready,
          .sample_aligned = status.reason == "aligned",
          .failure_confirmed = status.failure_confirmed,
      });
  logRuntimeTruthAlignmentTransition(get_logger(), status,
                                     truth_alignment_mission_update_);
}

void InterceptMissionRefereeNode::onVehicleDestroyed(
    const msg::VehicleDestroyed& destroyed, const bool interceptor,
    const std::size_t index) {
  const std::uint8_t expected_role = interceptor
                                         ? msg::VehicleDestroyed::ROLE_INTERCEPTOR
                                         : msg::VehicleDestroyed::ROLE_EVADER;
  const std::string& expected_id =
      interceptor ? interceptors_[index].id : targets_[index].id;
  if (!validateVehicleDestroyedEvent(get_logger(), destroyed, expected_role,
                                     expected_id, mission_epoch_)) {
    return;
  }
  bool& already_destroyed =
      interceptor ? interceptors_[index].destroyed : targets_[index].destroyed;
  if (already_destroyed) {
    return;
  }
  bool& destruction_requested = interceptor ? interceptors_[index].destruction_requested
                                            : targets_[index].destruction_requested;
  std::int64_t& requested_ns = interceptor
                                   ? interceptors_[index].destruction_requested_ns
                                   : targets_[index].destruction_requested_ns;
  const bool expected_proximity_event = destruction_requested;
  already_destroyed = true;
  destruction_requested = true;
  if (requested_ns <= 0) {
    requested_ns = now().nanoseconds();
  }
  if (destruction_requested_ns_ <= 0) {
    destruction_requested_ns_ = now().nanoseconds();
  }
  RCLCPP_ERROR(get_logger(),
               "VEHICLE_DESTROYED referee_observed=true role=%s vehicle_id='%s' "
               "cause=%s mission_epoch=%" PRIu64 " detail='%s'",
               detail::vehicleRoleName(destroyed.vehicle_role),
               destroyed.vehicle_id.c_str(),
               detail::deathCauseName(destroyed.death_cause), mission_epoch_,
               destroyed.detail.c_str());

  if (!interceptor &&
      destroyed.death_cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION &&
      targets_[index].outcome == TargetOutcome::kActive) {
    markTargetOutcome(index, TargetOutcome::kDestroyed, "physical_collision");
  }
  if (destroyed.death_cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION) {
    if (!system_failure_reason_.has_value()) {
      system_failure_reason_ = "physical_collision:" + expected_id;
      requestHoldsForSurvivors(*system_failure_reason_);
    }
    return;
  }
  if (!expected_proximity_event && !system_failure_reason_.has_value()) {
    system_failure_reason_ = "unexpected_proximity_destruction:" + expected_id;
    requestHoldsForSurvivors(*system_failure_reason_);
  }
}

bool InterceptMissionRefereeNode::verifyGroundTruthBoundary(const std::int64_t now_ns) {
  if (last_boundary_check_ns_ > 0 && now_ns - last_boundary_check_ns_ < 500'000'000LL) {
    return boundary_verified_;
  }
  last_boundary_check_ns_ = now_ns;
  const GroundTruthBoundaryUpdate update = ground_truth_boundary_->update(*this);
  if (!update.violating_subscriber.empty()) {
    failMission("ground_truth_boundary_violation:" + update.violating_topic + ":" +
                update.violating_subscriber);
    return false;
  }
  boundary_verified_ = update.verified;
  if (update.newly_verified) {
    RCLCPP_INFO(get_logger(),
                "RADAR_DATA_BOUNDARY verified=true physical_truth=true "
                "interceptors=%zu targets=%zu",
                interceptors_.size(), targets_.size());
  }
  return boundary_verified_;
}

bool InterceptMissionRefereeNode::missionReady(const std::int64_t now_ns) const {
  if (!truth_alignment_mission_update_.startup_ready) {
    return false;
  }
  const bool targets_ready =
      std::ranges::all_of(targets_, [now_ns](const TargetRuntime& target) {
        return !target.destroyed && target.state && target.truth_state &&
               target.state->navigation_ready && target.world_ready &&
               target.executable_horizon_ready &&
               target.executable_horizon_valid_from_ns > 0 &&
               target.executable_horizon_valid_until_ns >
                   target.executable_horizon_valid_from_ns &&
               now_ns >= target.executable_horizon_valid_from_ns &&
               now_ns < target.executable_horizon_valid_until_ns;
      });
  const bool interceptors_ready = std::ranges::all_of(
      interceptors_, [now_ns](const InterceptorRuntime& interceptor) {
        return !interceptor.destroyed && interceptor.state && interceptor.truth_state &&
               interceptor.state->navigation_ready && interceptor.world_ready &&
               interceptor.track_ready && interceptor.executable_horizon_ready &&
               interceptor.executable_horizon_valid_from_ns > 0 &&
               interceptor.executable_horizon_valid_until_ns >
                   interceptor.executable_horizon_valid_from_ns &&
               now_ns >= interceptor.executable_horizon_valid_from_ns &&
               now_ns < interceptor.executable_horizon_valid_until_ns;
      });
  return targets_ready && interceptors_ready;
}

std::optional<TimedVehicleState> InterceptMissionRefereeNode::interceptorPhysicalState(
    const std::size_t index) const noexcept {
  return detail::physicalState(interceptors_[index].state,
                               interceptors_[index].truth_state);
}

std::optional<TimedVehicleState> InterceptMissionRefereeNode::targetPhysicalState(
    const std::size_t index) const noexcept {
  return detail::physicalState(targets_[index].state, targets_[index].truth_state);
}

void InterceptMissionRefereeNode::publishTargetObjective(const std::size_t index) {
  TargetRuntime& target = targets_[index];
  target.objective_pub->publish(makePositionHoldObjective(
      now(), mission_epoch_, ++target.objective_sequence, target.goal));
}

void InterceptMissionRefereeNode::publishTargetStatus(const std::size_t index,
                                                      const std::string& detail_text) {
  const TargetRuntime& target = targets_[index];
  msg::InterceptTargetStatus status;
  status.header.stamp = now();
  status.header.frame_id = "map";
  status.mission_epoch = mission_epoch_;
  status.target_id = target.id;
  status.target_detection_id = target.detection_id;
  status.capturing_interceptor_id = target.capturing_interceptor_id;
  switch (target.outcome) {
    case TargetOutcome::kActive:
      status.status = msg::InterceptTargetStatus::STATUS_ACTIVE;
      break;
    case TargetOutcome::kIntercepted:
      status.status = msg::InterceptTargetStatus::STATUS_INTERCEPTED;
      break;
    case TargetOutcome::kReachedGoal:
      status.status = msg::InterceptTargetStatus::STATUS_REACHED_GOAL;
      break;
    case TargetOutcome::kDestroyed:
      status.status = msg::InterceptTargetStatus::STATUS_DESTROYED;
      break;
  }
  status.detail = detail_text;
  target_status_pub_->publish(status);
}

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptMissionRefereeNode>());
  rclcpp::shutdown();
  return 0;
}

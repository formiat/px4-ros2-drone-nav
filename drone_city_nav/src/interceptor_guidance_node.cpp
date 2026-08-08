#include "drone_city_nav/interceptor_guidance_node.hpp"

#include "drone_city_nav/intercept_guidance.hpp"
#include "drone_city_nav/msg/intercept_mission_command.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/radar_track_mode_command.hpp"
#include "drone_city_nav/msg/target_track.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::uint8_t
guidanceModeMessage(const InterceptGuidanceMode mode) noexcept {
  switch (mode) {
    case InterceptGuidanceMode::kDirect:
      return msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
    case InterceptGuidanceMode::kAnalyticIntercept:
      return msg::NavigationObjective::GUIDANCE_MODE_ANALYTIC_INTERCEPT;
    case InterceptGuidanceMode::kAheadIntercept:
      return msg::NavigationObjective::GUIDANCE_MODE_AHEAD_INTERCEPT;
  }
  return msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
}

[[nodiscard]] TimedVehicleState targetState(const msg::TargetTrack& track) noexcept {
  return TimedVehicleState{
      .position = Point3{track.position.x, track.position.y, track.position.z},
      .velocity = Vec3{track.velocity.x, track.velocity.y, track.velocity.z},
      .stamp_ns = detail::timeNanoseconds(track.header.stamp),
      .position_valid = track.position_valid,
      .velocity_valid = track.velocity_valid,
  };
}

} // namespace

class InterceptorGuidanceNode final : public rclcpp::Node {
public:
  explicit InterceptorGuidanceNode(const rclcpp::NodeOptions& options)
      : Node{"interceptor_guidance_node", options} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    expected_track_frame_ =
        declare_parameter<std::string>("expected_track_frame", "map");
    expected_maximum_measurement_age_s_ =
        declare_parameter<double>("expected_maximum_measurement_age_s", 3.5);
    guidance_ = std::make_unique<InterceptGuidance>(InterceptGuidanceConfig{
        .interceptor_speed_mps =
            declare_parameter<double>("intercept_interceptor_speed_mps", 20.0),
        .minimum_prediction_horizon_s =
            declare_parameter<double>("intercept_minimum_prediction_horizon_s", 0.0),
        .maximum_prediction_horizon_s =
            declare_parameter<double>("intercept_maximum_prediction_horizon_s", 15.0),
        .ahead_maximum_prediction_horizon_s = declare_parameter<double>(
            "intercept_ahead_maximum_prediction_horizon_s", 1.0),
        .fallback_prediction_horizon_s =
            declare_parameter<double>("intercept_fallback_prediction_horizon_s", 1.0),
        .minimum_target_speed_mps =
            declare_parameter<double>("intercept_minimum_target_speed_mps", 0.5),
        .ahead_enter_m = declare_parameter<double>("intercept_ahead_enter_m", 5.0),
        .ahead_exit_m = declare_parameter<double>("intercept_ahead_exit_m", 0.0),
        .ahead_corridor_enter_m =
            declare_parameter<double>("intercept_ahead_corridor_enter_m", 15.0),
        .ahead_corridor_exit_m =
            declare_parameter<double>("intercept_ahead_corridor_exit_m", 20.0),
        .horizon_smoothing_time_constant_s = declare_parameter<double>(
            "intercept_horizon_smoothing_time_constant_s", 0.5),
        .prediction_heading_offset_rad =
            declare_parameter<double>("intercept_prediction_heading_offset_rad", 0.0),
        .hypothesis_zero_distance_m =
            declare_parameter<double>("intercept_hypothesis_zero_distance_m", 30.0),
        .hypothesis_full_distance_m =
            declare_parameter<double>("intercept_hypothesis_full_distance_m", 120.0),
        .maximum_hypothesis_lateral_offset_m = declare_parameter<double>(
            "intercept_maximum_hypothesis_lateral_offset_m", 70.0),
        .target_vertical_deceleration_mps2 = declare_parameter<double>(
            "intercept_target_vertical_deceleration_mps2", 4.0),
        .target_flight_envelope =
            FlightEnvelopeConfig{
                .minimum_target_z_m =
                    declare_parameter<double>("minimum_target_z_m", 1.0),
                .maximum_target_z_m =
                    declare_parameter<double>("maximum_target_z_m", 32.0),
            },
    });

    const auto state_qos = rclcpp::QoS{10}.best_effort();
    ownship_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("ownship_state_topic",
                                       "/vehicles/interceptor/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr state) {
          ownship_state_ = detail::vehicleState(*state);
        });
    target_track_sub_ = create_subscription<msg::TargetTrack>(
        declare_parameter<std::string>("target_track_topic",
                                       "/vehicles/interceptor/target_track"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::TargetTrack::SharedPtr track) {
          if (track->header.frame_id != expected_track_frame_ ||
              !track->position_valid) {
            RCLCPP_WARN(get_logger(),
                        "INTERCEPT_GUIDANCE rejected_track=true frame='%s' "
                        "position_valid=%s",
                        track->header.frame_id.c_str(),
                        track->position_valid ? "true" : "false");
            return;
          }
          if (!target_track_.has_value() ||
              target_track_->track_id != track->track_id) {
            guidance_->resetTargetTrack();
          }
          target_track_ = *track;
        });
    radar_track_mode_sub_ = create_subscription<msg::RadarTrackModeCommand>(
        declare_parameter<std::string>(
            "radar_track_mode_command_topic",
            "/vehicles/interceptor/radar/track_mode_command"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::RadarTrackModeCommand::SharedPtr command) {
          if (command->mission_epoch != mission_epoch_ ||
              command->mode > msg::RadarTrackModeCommand::MODE_TRACK ||
              !target_track_.has_value() ||
              command->target_track_id != target_track_->track_id) {
            return;
          }
          guidance_->observeLineOfSight(command->mode ==
                                        msg::RadarTrackModeCommand::MODE_TRACK);
        });
    mission_command_sub_ = create_subscription<msg::InterceptMissionCommand>(
        declare_parameter<std::string>("mission_command_topic",
                                       "/vehicles/interceptor/mission_command"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::InterceptMissionCommand::SharedPtr command) {
          onMissionCommand(*command);
        });
    objective_pub_ = create_publisher<msg::NavigationObjective>(
        declare_parameter<std::string>("navigation_objective_topic",
                                       "/vehicles/interceptor/navigation_objective"),
        rclcpp::QoS{1}.reliable().transient_local());
    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Interceptor guidance ready: source=radar_track update_rate_hz=20 "
                "expected_maximum_measurement_age_s=%.3f",
                expected_maximum_measurement_age_s_);
  }

private:
  void onMissionCommand(const msg::InterceptMissionCommand& command) {
    if (command.mission_epoch != mission_epoch_ ||
        command.command !=
            msg::InterceptMissionCommand::COMMAND_HOLD_CURRENT_POSITION) {
      return;
    }
    hold_requested_ = true;
    hold_reason_ = command.reason;
    hold_position_.reset();
    publishHoldIfReady();
  }

  void publishHoldIfReady() {
    if (!hold_requested_ || hold_position_.has_value() || !ownship_state_.has_value() ||
        !ownship_state_->position_valid) {
      return;
    }
    hold_position_ = ownship_state_->position;
    msg::NavigationObjective objective;
    objective.stamp = now();
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = ++objective_sequence_;
    objective.target_track_id = 0U;
    objective.position.x = hold_position_->x;
    objective.position.y = hold_position_->y;
    objective.position.z = hold_position_->z;
    objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
    objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
    objective.terminal_policy =
        msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD;
    objective_pub_->publish(objective);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD_OBJECTIVE published=true reason='%s' "
                "mission_epoch=%" PRIu64,
                hold_reason_.c_str(), mission_epoch_);
  }

  void tick() {
    if (hold_requested_) {
      publishHoldIfReady();
      return;
    }
    if (!ownship_state_.has_value() || !ownship_state_->position_valid ||
        !target_track_.has_value()) {
      return;
    }
    const auto publication_stamp = now();
    const TimedVehicleState target = targetState(*target_track_);
    const InterceptGuidanceResult guidance =
        guidance_->update(*ownship_state_, target, publication_stamp.nanoseconds());
    if (!guidance.valid) {
      return;
    }

    msg::NavigationObjective objective;
    objective.stamp = publication_stamp;
    objective.observation_stamp = detail::timeMessage(guidance.observation_stamp_ns);
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = ++objective_sequence_;
    objective.target_track_id = target_track_->track_id;
    objective.position.x = guidance.predicted_position.x;
    objective.position.y = guidance.predicted_position.y;
    objective.position.z = guidance.predicted_position.z;
    objective.observed_target_position.x = guidance.observed_position.x;
    objective.observed_target_position.y = guidance.observed_position.y;
    objective.observed_target_position.z = guidance.observed_position.z;
    objective.observed_target_velocity.x = guidance.observed_velocity.x;
    objective.observed_target_velocity.y = guidance.observed_velocity.y;
    objective.observed_target_velocity.z = guidance.observed_velocity.z;
    objective.prediction_horizon_s = guidance.prediction_horizon_s;
    objective.configured_heading_offset_rad = guidance.configured_heading_offset_rad;
    objective.effective_heading_offset_rad = guidance.effective_heading_offset_rad;
    objective.hypothesis_lateral_offset_m = guidance.hypothesis_lateral_offset_m;
    objective.vertical_prediction_limited = guidance.vertical_prediction_limited;
    objective.objective_type =
        msg::NavigationObjective::OBJECTIVE_TYPE_TRACKING_PREDICTION;
    objective.guidance_mode = guidanceModeMessage(guidance.mode);
    objective.terminal_policy =
        msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING;
    objective_pub_->publish(objective);

    if (guidance.prediction_age_s > expected_maximum_measurement_age_s_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "RADAR_TRACK stale_contract_violation=true measurement_age_s=%.3f "
          "expected_maximum_s=%.3f action=continue_coasting",
          guidance.prediction_age_s, expected_maximum_measurement_age_s_);
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "INTERCEPT_GUIDANCE source=radar_track mode=%s track_id=%" PRIu64
        " scan_sequence=%" PRIu64 " target_speed_mps=%.3f horizon_s=%.3f "
        "analytic_intercept_time_s=%.3f measurement_age_s=%.3f ahead_m=%.3f "
        "cross_track_m=%.3f configured_heading_offset_deg=%.1f "
        "effective_heading_offset_deg=%.1f hypothesis_lateral_offset_m=%.3f "
        "hypothesis_converged_latched=%s vertical_prediction_limited=%s",
        interceptGuidanceModeName(guidance.mode), target_track_->track_id,
        target_track_->source_scan_sequence, guidance.target_speed_mps,
        guidance.prediction_horizon_s, guidance.analytic_intercept_time_s,
        guidance.prediction_age_s, guidance.ahead_m, guidance.cross_track_m,
        guidance.configured_heading_offset_rad * 180.0 / std::acos(-1.0),
        guidance.effective_heading_offset_rad * 180.0 / std::acos(-1.0),
        guidance.hypothesis_lateral_offset_m,
        guidance.hypothesis_converged_latched ? "true" : "false",
        guidance.vertical_prediction_limited ? "true" : "false");
  }

  std::unique_ptr<InterceptGuidance> guidance_;
  std::optional<TimedVehicleState> ownship_state_;
  std::optional<msg::TargetTrack> target_track_;
  std::optional<Point3> hold_position_;
  std::string expected_track_frame_;
  std::string hold_reason_;
  double expected_maximum_measurement_age_s_{3.5};
  std::uint64_t mission_epoch_{1U};
  std::uint64_t objective_sequence_{0U};
  bool hold_requested_{false};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr ownship_state_sub_;
  rclcpp::Subscription<msg::TargetTrack>::SharedPtr target_track_sub_;
  rclcpp::Subscription<msg::RadarTrackModeCommand>::SharedPtr radar_track_mode_sub_;
  rclcpp::Subscription<msg::InterceptMissionCommand>::SharedPtr mission_command_sub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr objective_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node>
makeInterceptorGuidanceNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<InterceptorGuidanceNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::InterceptorGuidanceNode)

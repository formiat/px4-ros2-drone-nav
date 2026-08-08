#include "drone_city_nav/msg/radar_scan.hpp"
#include "drone_city_nav/msg/radar_track_mode_command.hpp"
#include "drone_city_nav/msg/simulation_truth_state.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/radar_cadence.hpp"
#include "drone_city_nav/radar_model.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] const char* cadenceReasonName(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE:
      return "no_tracking_objective";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_OCCLUDED:
      return "observed_target_occluded";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_VISIBLE:
      return "observed_target_visible";
    case msg::RadarTrackModeCommand::REASON_WORLD_UNAVAILABLE:
      return "world_unavailable";
    default:
      return "unknown";
  }
}

[[nodiscard]] std::optional<TimedVehicleState>
stateAt(const TimedVehicleState& state, const std::int64_t stamp_ns,
        const double maximum_alignment_s) noexcept {
  if (!state.position_valid || !state.velocity_valid || state.stamp_ns <= 0 ||
      stamp_ns <= 0) {
    return std::nullopt;
  }
  const double delta_s = static_cast<double>(stamp_ns - state.stamp_ns) * 1.0e-9;
  if (std::abs(delta_s) > maximum_alignment_s) {
    return std::nullopt;
  }
  TimedVehicleState aligned = state;
  aligned.position.x += aligned.velocity.x * delta_s;
  aligned.position.y += aligned.velocity.y * delta_s;
  aligned.position.z += aligned.velocity.z * delta_s;
  aligned.stamp_ns = stamp_ns;
  return aligned;
}

} // namespace

class RadarSimulatorNode final : public rclcpp::Node {
public:
  RadarSimulatorNode()
      : Node{"radar_simulator_node"} {
    radar_frame_id_ = declare_parameter<std::string>("radar_frame_id", "radar_yaw");
    maximum_state_alignment_s_ =
        declare_parameter<double>("maximum_state_alignment_s", 0.1);
    if (!(maximum_state_alignment_s_ > 0.0)) {
      throw std::invalid_argument{"maximum radar state alignment must be positive"};
    }
    cadence_ = std::make_unique<CorrelatedRadarCadence>(RadarCadenceConfig{
        .minimum_interval_s = declare_parameter<double>("minimum_scan_interval_s", 0.1),
        .maximum_interval_s = declare_parameter<double>("maximum_scan_interval_s", 3.0),
        .initial_interval_s = declare_parameter<double>("initial_scan_interval_s", 0.1),
        .maximum_step_s = declare_parameter<double>("maximum_interval_step_s", 0.25),
        .step_correlation =
            declare_parameter<double>("interval_step_correlation", 0.85),
        .track_interval_s = declare_parameter<double>("track_interval_s", 0.05),
        .random_seed = static_cast<std::uint64_t>(
            declare_parameter<std::int64_t>("random_seed", 42)),
    });

    const auto state_qos = rclcpp::QoS{10}.best_effort();
    radar_navigation_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("radar_navigation_state_topic",
                                       "/vehicles/interceptor/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr state) {
          radar_navigation_state_ = detail::vehicleState(*state);
        });
    radar_truth_state_sub_ = create_subscription<msg::SimulationTruthState>(
        declare_parameter<std::string>("radar_truth_state_topic",
                                       "/simulation_truth/vehicles/interceptor/state"),
        state_qos, [this](const msg::SimulationTruthState::SharedPtr state) {
          radar_truth_state_ = detail::physicalTruthState(*state);
        });
    target_truth_state_sub_ = create_subscription<msg::SimulationTruthState>(
        declare_parameter<std::string>("target_truth_state_topic",
                                       "/simulation_truth/vehicles/evader/state"),
        state_qos, [this](const msg::SimulationTruthState::SharedPtr state) {
          target_truth_state_ = detail::physicalTruthState(*state);
        });
    scan_pub_ = create_publisher<msg::RadarScan>(
        declare_parameter<std::string>("radar_scan_topic",
                                       "/vehicles/interceptor/radar/scan"),
        rclcpp::QoS{10}.reliable());
    track_mode_command_sub_ = create_subscription<msg::RadarTrackModeCommand>(
        declare_parameter<std::string>(
            "track_mode_command_topic",
            "/vehicles/interceptor/radar/track_mode_command"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::RadarTrackModeCommand::SharedPtr command) {
          onTrackModeCommand(*command);
        });
    timer_ = create_wall_timer(std::chrono::milliseconds{20}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Radar simulator ready: frame='%s' cadence=[%.3f,%.3f]s "
                "track_interval=%.3fs track_control=los_command",
                radar_frame_id_.c_str(), cadenceMinimumInterval(),
                cadenceMaximumInterval(),
                get_parameter("track_interval_s").as_double());
  }

private:
  void onTrackModeCommand(const msg::RadarTrackModeCommand& command) {
    if (command.mode > msg::RadarTrackModeCommand::MODE_TRACK ||
        command.reason > msg::RadarTrackModeCommand::REASON_WORLD_UNAVAILABLE ||
        command.mission_epoch < track_mode_mission_epoch_ ||
        (command.mission_epoch == track_mode_mission_epoch_ &&
         command.objective_sample_sequence <= track_mode_objective_sequence_)) {
      return;
    }
    const bool next_track_mode = command.mode == msg::RadarTrackModeCommand::MODE_TRACK;
    const bool newly_active = next_track_mode && !track_mode_active_;
    track_mode_active_ = next_track_mode;
    track_mode_reason_ = command.reason;
    track_mode_mission_epoch_ = command.mission_epoch;
    track_mode_objective_sequence_ = command.objective_sample_sequence;
    if (newly_active) {
      next_scan_due_ns_ = 0;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RADAR_TRACK_MODE active=%s reason=%s mission_epoch=%lu objective_sample=%lu "
        "immediate_scan=%s",
        track_mode_active_ ? "true" : "false", cadenceReasonName(track_mode_reason_),
        static_cast<unsigned long>(track_mode_mission_epoch_),
        static_cast<unsigned long>(track_mode_objective_sequence_),
        newly_active ? "true" : "false");
  }

  [[nodiscard]] double cadenceMinimumInterval() const noexcept {
    return get_parameter("minimum_scan_interval_s").as_double();
  }

  [[nodiscard]] double cadenceMaximumInterval() const noexcept {
    return get_parameter("maximum_scan_interval_s").as_double();
  }

  void tick() {
    const std::int64_t now_ns = now().nanoseconds();
    if (next_scan_due_ns_ > 0 && now_ns < next_scan_due_ns_) {
      return;
    }
    if (!radar_navigation_state_.has_value() || !radar_truth_state_.has_value() ||
        !target_truth_state_.has_value() || !radar_navigation_state_->heading_valid) {
      return;
    }
    const std::int64_t measurement_stamp_ns = radar_truth_state_->stamp_ns;
    if (measurement_stamp_ns <= previous_scan_stamp_ns_) {
      return;
    }
    const double heading_age_s =
        std::abs(static_cast<double>(measurement_stamp_ns -
                                     radar_navigation_state_->stamp_ns)) *
        1.0e-9;
    if (heading_age_s > maximum_state_alignment_s_) {
      return;
    }
    const std::optional<TimedVehicleState> target =
        stateAt(*target_truth_state_, measurement_stamp_ns, maximum_state_alignment_s_);
    if (!target.has_value()) {
      return;
    }
    TimedVehicleState radar = *radar_truth_state_;
    radar.heading_rad = radar_navigation_state_->heading_rad;
    radar.heading_valid = true;
    const std::optional<RadarDetectionSample> detection =
        simulateIdealRadarDetection(radar, *target, 1U);
    if (!detection.has_value()) {
      return;
    }

    msg::RadarScan scan;
    scan.header.stamp = detail::timeMessage(measurement_stamp_ns);
    scan.header.frame_id = radar_frame_id_;
    scan.scan_sequence = ++scan_sequence_;
    scan.cadence_mode = track_mode_active_ ? msg::RadarScan::CADENCE_MODE_TRACK
                                           : msg::RadarScan::CADENCE_MODE_SEARCH;
    scan.cadence_reason = track_mode_reason_;
    msg::RadarDetection radar_detection;
    radar_detection.detection_id = detection->detection_id;
    radar_detection.range_m = detection->range_m;
    radar_detection.azimuth_rad = detection->azimuth_rad;
    radar_detection.elevation_rad = detection->elevation_rad;
    radar_detection.radial_velocity_mps = detection->radial_velocity_mps;
    scan.detections.push_back(radar_detection);
    scan_pub_->publish(scan);

    const double next_interval_s = cadence_->nextIntervalSeconds(track_mode_active_);
    next_scan_due_ns_ = now_ns + static_cast<std::int64_t>(next_interval_s * 1.0e9);
    const double actual_interval_s =
        previous_scan_stamp_ns_ > 0
            ? static_cast<double>(measurement_stamp_ns - previous_scan_stamp_ns_) *
                  1.0e-9
            : 0.0;
    previous_scan_stamp_ns_ = measurement_stamp_ns;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RADAR_SCAN published=true sequence=%lu detections=1 range_m=%.3f "
        "actual_interval_s=%.3f next_interval_s=%.3f track_mode=%s cadence_reason=%s "
        "source=gazebo_physical_truth",
        static_cast<unsigned long>(scan.scan_sequence), detection->range_m,
        actual_interval_s, next_interval_s, track_mode_active_ ? "true" : "false",
        cadenceReasonName(track_mode_reason_));
  }

  std::unique_ptr<CorrelatedRadarCadence> cadence_;
  std::optional<TimedVehicleState> radar_navigation_state_;
  std::optional<TimedVehicleState> radar_truth_state_;
  std::optional<TimedVehicleState> target_truth_state_;
  std::string radar_frame_id_;
  double maximum_state_alignment_s_{0.1};
  std::int64_t next_scan_due_ns_{0};
  std::int64_t previous_scan_stamp_ns_{0};
  std::uint64_t scan_sequence_{0U};
  std::uint64_t track_mode_mission_epoch_{0U};
  std::uint64_t track_mode_objective_sequence_{0U};
  std::uint8_t track_mode_reason_{
      msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE};
  bool track_mode_active_{false};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr
      radar_navigation_state_sub_;
  rclcpp::Subscription<msg::SimulationTruthState>::SharedPtr radar_truth_state_sub_;
  rclcpp::Subscription<msg::SimulationTruthState>::SharedPtr target_truth_state_sub_;
  rclcpp::Publisher<msg::RadarScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<msg::RadarTrackModeCommand>::SharedPtr track_mode_command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::RadarSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
